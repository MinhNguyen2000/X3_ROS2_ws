import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from x3_nav_interfaces.action import NavigateToGoal

import numpy as np
import copy
import signal
import time             # to maintain constant control sampling time

class APFPlannerNode(Node):
    def __init__(self):
        super().__init__('apf_planner_server')

        # ===== ROS Parameters =====
        self.declare_parameter('agent_name', 'agent0')
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('obstacle_tolerance', 0.21)
        self.declare_parameter('max_lin_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('d_safe', 1.0)           # repulsive field influence radius
        self.declare_parameter('n_ray_groups', 18)
        self.declare_parameter('k_att', 10.0)           # attraction constant 
        self.declare_parameter('k_rep', 0.30)           # repulsion constant
        self.declare_parameter('att_cap', 10.0)
        self.declare_parameter('k_v', 1.0)              # force => lin vel constant
        self.declare_parameter('k_w', 3.0)              # force => ang vel constant

        self.agent_name         = self.get_parameter('agent_name').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value
        self.obstacle_tolerance = self.get_parameter('obstacle_tolerance').value
        self.max_lin_vel        = self.get_parameter('max_lin_vel').value
        self.max_angular_vel    = self.get_parameter('max_angular_vel').value
        self.goal_timeout       = self.get_parameter('goal_timeout').value
        self.d_safe             = self.get_parameter('d_safe').value
        self.n_ray_groups       = self.get_parameter('n_ray_groups').value
        self.k_att              = self.get_parameter('k_att').value
        self.k_rep              = self.get_parameter('k_rep').value
        self.att_cap            = self.get_parameter('att_cap').value
        self.k_v                = self.get_parameter('k_v').value
        self.k_w                = self.get_parameter('k_w').value

        # --- State storage ---
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None

        # ===== Subscribers & Publisher =====
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, f'{self.agent_name}/odom', self.odom_callback, qos)
        self.lidar_sub = self.create_subscription(LaserScan, f'{self.agent_name}/scan', self.lidar_callback, qos)
        self.cmd_pub = self.create_publisher(TwistStamped, f'{self.agent_name}/cmd_vel', 10)

        planner_id_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._planner_id_pub = self.create_publisher(String, 'active_planner', planner_id_qos)
        self._planner_id_pub.publish(String(data='APF'))

        
        # ===== Action server
        # --- Active goal handle ---
        self._current_goal_handle: ServerGoalHandle | None = None

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback
        )

        self.get_logger().info('apf_planner_server started!')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # index of the zero angle in the (-π, π) scan
        zero_idx = round(-msg.angle_min / msg.angle_increment)

        # rotate the ranges from (-π, π) to (0, 2π)
        ranges_drl = np.roll(ranges, -zero_idx)

        # mask out LiDAR ranges hitting the rear antennae
        antennae_mask = ranges_drl <= 0.20
        ranges_drl[antennae_mask] = msg.range_max

        self.latest_scan = copy.deepcopy(msg)
        self.latest_scan.ranges = ranges_drl.tolist() 

    def goal_callback(self, goal_request: NavigateToGoal):
        '''
        Called when a new goal request arrives
        '''
        goal_request_target_pose: PoseStamped = goal_request.target_pose
        self.get_logger().info(
            f"New goal received at: ({goal_request_target_pose.pose.position.x: 5.3f},"
            f"{goal_request_target_pose.pose.position.y: 5.3f})"
        )

        # Overwrite any curent goal
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Changing the current goal.')
            try:
                self._current_goal_handle.abort()
            except:
                pass
        return GoalResponse.ACCEPT

    def cancel_callback (self, goal_handle):
        '''
        Called when a cancel request arrives 
        '''
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        '''
        APF planner that runs while the goal is active
        '''
        self._current_goal_handle = goal_handle
        target: PoseStamped = goal_handle.request.target_pose
        goal_tolerance = getattr(goal_handle.request, 'goal_tolerance', None) or self.goal_tolerance

        # --- Initialize feedback and result messages ---
        result_msg = NavigateToGoal.Result()
        feedback_msg = NavigateToGoal.Feedback()

        # --- Set control timing ----
        ctrl_hz = 50.0
        ctrl_period = 1.0 / ctrl_hz

        # wait for first odom/scam
        while(self.latest_odom is None or self.latest_scan is None) and rclpy.ok():
            self.get_logger().warn('Waiting for odometry and LiDAR...')
            time.sleep(ctrl_period)

        x_prev = self.latest_odom.pose.pose.position.x
        y_prev = self.latest_odom.pose.pose.position.y
        total_distance = 0.0
        start_time = time.time()

        while rclpy.ok():
            ctrl_iter_start = time.time()

            if not goal_handle.is_active:
                self._publish_cmd(0.0, 0.0)
                result_msg.success = False
                result_msg.message = 'Goal aborted/pre-empted'
                return result_msg

            # --- Check for cancellation ---
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_cmd(0.0, 0.0)
                result_msg.success = False
                result_msg.message = 'Cancelled by client.'
                return result_msg

            # --- Check for timeout ---
            elapsed_time = time.time() - start_time
            if elapsed_time >= self.goal_timeout:
                goal_handle.abort()
                self._publish_cmd(0.0, 0.0)
                result_msg.success = False
                result_msg.message = f'Goal timeout after {elapsed_time:.1f}s.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().warn(f'Goal timed out after {elapsed_time:.1f}s')
                return result_msg

            # --- Sensor and odometry data processing ---
            odom = self.latest_odom
            scan = self.latest_scan

            agent_pos = odom.pose.pose.position
            agent_yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)
            goal_pos = target.pose.position

            dx = goal_pos.x - agent_pos.x
            dy = goal_pos.y - agent_pos.y
            d_goal = float(np.sqrt(dx**2 + dy**2))

            # --- Check termination conditions ---
            if d_goal <= goal_tolerance:
                self._publish_cmd(0.0, 0.0)
                goal_handle.succeed()
                result_msg.success=True
                result_msg.message='Goal reached.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().info('Goal reached.')
                return result_msg

            min_lidar = np.min([lidar_range for lidar_range in scan.ranges if lidar_range >= 0.2])
            if min_lidar <= self.obstacle_tolerance:
                self._publish_cmd(0.0, 0.0)
                goal_handle.abort()
                result_msg.success=False
                result_msg.message='Obstacle hit, mission aborted.'
                result_msg.total_distance = float(total_distance)
                self.get_logger().info(f'Obstacle hit with min_lidar={min_lidar: 5.3f}, mission aborted.')
                return result_msg

            # --- Compute APF force in the robot-local frame ---
            fx, fy = self._compute_apf_force(dx, dy, agent_yaw, scan)

            # --- Convert to (v, w) ---
            v = np.clip(self.k_v * fx, 0.0, self.max_lin_vel)
            w = np.clip(self.k_w * np.arctan2(fy,fx if abs(fx) > 1e-6 else 1e-6), 
                        -self.max_angular_vel, self.max_angular_vel)

            self._publish_cmd(float(v), float(w))

            # --- Feedback Message ---
            feedback_msg.distance_to_goal = d_goal
            feedback_msg.elapsed_time = float(elapsed_time)
            feedback_msg.current_pose = odom.pose.pose
            goal_handle.publish_feedback(feedback_msg)

            # --- Incremenent total distance ---
            x, y = agent_pos.x, agent_pos.y
            step_distance = np.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            total_distance += step_distance
            x_prev, y_prev = x, y

            ctrl_iter_elapsed = time.time() - ctrl_iter_start
            ctrl_iter_remain = ctrl_period - ctrl_iter_elapsed
            if ctrl_iter_remain > 0:
                time.sleep(ctrl_iter_remain)
            
    def _compute_apf_force(self, dx: float, dy: float, agent_yaw: float, scan: LaserScan):
        '''
        Computes the net APF force in the robot's local (x-forward, y-left) frame.
        dx, dy are in the agent's odom frame but first rotated into the robot's local frame.
        '''
        # --- rotate world frame (dx,dy) into robot-local frame (dx_r, dy_r)
        c, s = np.cos(agent_yaw), np.sin(agent_yaw)
        dx_r =  c * dx + s * dy
        dy_r = -s * dx + c * dy
        d_goal = np.sqrt(dx_r**2 + dy_r**2) + 1e-6

        # --- attractive force with limited magnitude ---
        att_mag = min(self.k_att * d_goal, self.att_cap)
        fx = att_mag * (dx_r / d_goal)
        fy = att_mag * (dy_r / d_goal)

        # --- repulsive force from min-pooled LiDAR groups ---
        raw = np.array(scan.ranges, dtype=np.float32)
        raw = np.where(np.isfinite(raw), raw, scan.range_max)
        raw = np.clip(raw, scan.range_min, scan.range_max)

        # TODO - antennae masking
        # antennae_mask = raw <= 0.20
        # raw[antennae_mask] = scan.range_max

        # TODO - check if flipping is required
        # raw = np.flip(raw)
        
        n_groups = self.n_ray_groups
        lidar_groups    = np.array_split(raw, n_groups)
        lidar_obs       = np.array([g.min() for g in lidar_groups], dtype=np.float32)

        # bearing of the group center, spanning 0 to 2pi in the robot frame (0 is forward then ccw)
        group_angles = (np.arange(n_groups) + 0.5) * (2 * np.pi / n_groups)

        obstacle_mask = lidar_obs < self.d_safe

        lidar_ranges_repulse = lidar_obs[obstacle_mask]
        group_angles_repulse = group_angles[obstacle_mask]

        f_repulse_x = 0
        f_repulse_y = 0

        for r_i, theta_i in zip(lidar_ranges_repulse, group_angles_repulse):
            r_i = max(r_i, 0.05)    # avoid singularity

            # repulsive force magnitude
            mag = self.k_rep * (1.0 / r_i - 1.0 / self.d_safe) * (1.0 / (r_i**2))

            # repulsive force direction
            ox, oy = np.cos(theta_i), np.sin(theta_i)
            f_repulse_x += mag * ox
            f_repulse_y += mag * oy

        # # DEBUG PRINTS
        # self.get_logger().info(f"LiDAR: {lidar_obs}")
        # self.get_logger().info(f"LiDAR<obstacle index: {obstacle_mask}")
        # self.get_logger().info(f"LiDAR ranges: {[range for range in lidar_ranges_repulse]}")
        # self.get_logger().info(f"LiDAR angles: {[angle/np.pi*180 for angle in group_angles_repulse]}")
        # self.get_logger().info(f"Atraction (fx,fy) = ({fx: 5.3f},{fy: 5.3f}) | "
        #                        f"Repulsion (fx,fy) = ({f_repulse_x: 5.3f},{f_repulse_y: 5.3f})")

        # Calculate net forces
        fx -= f_repulse_x
        fy -= f_repulse_y

        # # DEBUG PRINTS - net forces
        # self.get_logger().info(f"fx: {fx: 5.3f} | fy: {fy: 5.3f}")
        return fx, fy


    def _publish_cmd(self, v: float, w: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = f'{self.agent_name}_base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.cmd_pub.publish(cmd)

    def _yaw_from_quaternion(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return np.arctan2(siny_cosp, cosy_cosp, dtype=np.float32)
        
def main():
    rclpy.init()
    node = APFPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # catch sigterm from GUI:
    signal.signal(signal.SIGTERM, lambda *args: executor.shutdown())

    try: 
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()