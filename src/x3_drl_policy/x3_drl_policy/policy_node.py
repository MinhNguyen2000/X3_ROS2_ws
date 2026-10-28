import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor       # to prevent blocking code while navigating to the goal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from x3_nav_interfaces.action import NavigateToGoal
from x3_drl_policy.lidar_processing import preprocess_scan, pool_groups

import numpy as np
# TODO - import torch related packages for DRL inference (torch and torch.nn)
import torch, torch.nn as nn
# from stable_baselines3 import TD3, SAC
import gymnasium as gym
from stable_baselines3.td3.policies import Actor

from ament_index_python.packages import get_package_share_directory
import os
import json
import time
import copy
import signal
import threading        # Monitor shutdown request, gracefully end processes, and send zero velocity cmd to robot

class DRLPolicyNode(Node):
    def __init__(self):
        super().__init__('drl_policy_server')

        self.declare_parameter('agent_name', 'agent0')
        self.declare_parameter('goal_tolerance', 0.75)
        self.declare_parameter('obstacle_tolerance', 0.21)
        # self.declare_parameter('model_name', 'TD3_00391_1000')
        self.declare_parameter('model_name', 'TD3_00386_1000')
        self.declare_parameter('max_lin_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)  
        self.declare_parameter('goal_timeout', 60.0)
        # True: the policy acts in normalized units and max_*_vel is applied only when publishing (as in the
        # policies' training setup). False: the action is clipped/accumulated directly in m/s and rad/s.
        self.declare_parameter('normalized_actions', True)
        # LiDAR returns closer than this [m] are treated as free space (robot's own antennae); 0 disables
        self.declare_parameter('antennae_mask_dist', 0.20)
        # half-width [deg] of the wedge behind the LiDAR set to range_max; 0 disables (use on hardware only)
        self.declare_parameter('rear_mask_deg', 0.0)
        # 'range_max': masked beams are set to the maximum range. 'train_mean': masked beams are ignored when
        # pooling, and a fully masked group takes this policy's training mean for that group (norm_stats.npz)
        self.declare_parameter('rear_mask_fill', 'range_max')
        # maximum range [m] the policy sees; 0 uses the sensor's range_max. 11.10 is the observation bound of
        # the training environment; invalid and masked returns are set to it as well
        self.declare_parameter('range_clip', 0.0)
        # delta mode: bounds of the actor output (the delta action), matching the Nav2D training default
        # (ROS2_DRL_Navigation nav2d.py: delta_act_linear = delta_act_angular = 0.01 -> [-0.01, 0.01])
        self.declare_parameter('delta_act_linear', 0.01)
        self.declare_parameter('delta_act_angular', 0.01)
        # delta mode: separate gain applied to the actor output before it is accumulated onto the last command.
        # 5.0 keeps the effective +/-0.05 step per control tick that the Sep 2026 results were collected with.
        # TODO - verify which delta bounds / control rate TD3_00386-395 were actually trained with (nav2d.py and
        #        utils.py default to 0.01, but gym_TD3.py / test_TD3.py use 0.05), then revisit these gains.
        self.declare_parameter('delta_act_scale_linear', 5.0)
        self.declare_parameter('delta_act_scale_angular', 5.0)

        self.agent_name         = self.get_parameter('agent_name').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value
        self.obstacle_tolerance = self.get_parameter('obstacle_tolerance').value
        self.model_name         = self.get_parameter('model_name').value
        self.model_type         = self.model_name.split('_')[0]
        self.model_id           = self.model_name.split('_')[1]
        self.max_lin_vel        = self.get_parameter('max_lin_vel').value
        self.max_angular_vel    = self.get_parameter('max_angular_vel').value
        self.goal_timeout       = self.get_parameter('goal_timeout').value
        self.normalized_actions = self.get_parameter('normalized_actions').value
        self.antennae_mask_dist = self.get_parameter('antennae_mask_dist').value
        self.rear_mask_deg      = self.get_parameter('rear_mask_deg').value
        self.rear_mask_fill     = self.get_parameter('rear_mask_fill').value
        self.range_clip         = self.get_parameter('range_clip').value
        self.delta_act_linear  = self.get_parameter('delta_act_linear').value
        self.delta_act_angular  = self.get_parameter('delta_act_angular').value
        self.delta_act_scale    = np.array([self.get_parameter('delta_act_scale_linear').value,
                                            self.get_parameter('delta_act_scale_angular').value], dtype=np.float32)

        # determine the action mode
        if 386 <= int(self.model_id) <= 395 or int(self.model_id) == 384:
            self.action_mode = 'delta'
        else:
            self.action_mode = 'direct'

        # TODO - load the model and extract the number of n_ray_groups for LiDAR group
        # Assume that the models and norm stats are stored under drl_policy/policy/TD3_xxx/model.zip and norm_stats.pkl
        pkg_dir = get_package_share_directory('x3_drl_policy')
        model_dir = os.path.join(pkg_dir, 'policies', self.model_name)
        policy_weight_path = os.path.join(model_dir, "actor_weights.pt")
        norm_stat_path = os.path.join(model_dir, "norm_stats.npz")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ===== Load norm stats =====
        norm = np.load(norm_stat_path)
        self.obs_mean    = norm["mean"].astype(np.float32)
        self.obs_var     = norm["var"].astype(np.float32)
        self.clip_obs    = float(norm["clip_obs"])

        # # (DEBUG) Check the loaded stats
        # print(f"obs_mean shape: {self.obs_mean.shape}")   # should be (27,)
        # print(f"obs_var shape:  {self.obs_var.shape}")    # should be (27,)
        # print(f"clip_obs:       {self.clip_obs}")         # typically 10.0

        # # (DEBUG) Check mean/var per observation component
        # labels = ["dx","dy","dgoal","c_head","s_head","c_bear","s_bear","vx","vyaw"] + [f"lidar_{i}" for i in range(18)]
        # for i, label in enumerate(labels):
        #     print(f"  {label:10s}  mean={self.obs_mean[i]: 7.4f}  std={np.sqrt(self.obs_var[i]):.4f}")

        # ===== Load actor network =====
        self._set_action_space()
        self.policy = Actor(
            observation_space=gym.spaces.Box(low=-np.inf, high=np.inf, shape=(27,), dtype=np.float32),
            action_space=self.action_space,
            net_arch=[512, 256],
            features_extractor=torch.nn.Identity(),   # MlpPolicy uses FlattenExtractor but weights already flattened
            features_dim=27,
            activation_fn=torch.nn.ReLU,
            normalize_images=False,
        ).to(self.device)
        state = torch.load(policy_weight_path, map_location=self.device)
        self.policy.load_state_dict(state)
        self.policy.eval()

        # Test the loaded actor policy

        # TODO - read n_ray_groups and obs_space size dynamically from the loaded model
        self.n_ray_groups = 18
        self._obs_space_size = 27
        self._obs_buffer = np.zeros(self._obs_space_size, dtype=np.float32)

        # Initialize DRL states and parameters
        self._reset_states()
        self.d_safe = 0.5
        self.lidar_idx_threshold = 4

        # intialize the scaled reward components
        self.rew_head_approach_scaled = 0   ; self.rew_head_approach_scale = 200.0
        self.rew_dist_approach_scaled = 0   ; self.rew_dist_approach_scale = 200.0
        self.rew_obs_dist_scaled = 0        ; self.rew_obs_dist_scale = 0.5
        self.rew_obs_align_scaled = 0       ; self.rew_obs_align_scale = 0.5
        self.rew_time = -0.5

        # --- State storage ---
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None

        # --- Subscribers & Publisher ---
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._cb_group = ReentrantCallbackGroup()

        self.odom_sub   = self.create_subscription(Odometry,  f'odom', self.odom_callback, qos, callback_group=self._cb_group)
        self.lidar_sub  = self.create_subscription(LaserScan, f'scan', self.lidar_callback, qos, callback_group=self._cb_group)
        self.cmd_pub    = self.create_publisher(TwistStamped, f'cmd_vel', 10)

        # Broadcast the model name
        planner_id_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._planner_id_pub = self.create_publisher(String, 'active_planner', planner_id_qos)
        self._planner_id_pub.publish(String(data=f'{self.model_name}'))

        # the run's processing/action config, latched, so the sequence server can save it with the trajectory
        self._config_pub = self.create_publisher(String, 'planner_config', planner_id_qos)
        self._config_pub.publish(String(data=json.dumps(dict(
            model_name=self.model_name, action_mode=self.action_mode, normalized_actions=self.normalized_actions,
            max_lin_vel=self.max_lin_vel, max_angular_vel=self.max_angular_vel,
            antennae_mask_dist=self.antennae_mask_dist, rear_mask_deg=self.rear_mask_deg,
            rear_mask_fill=self.rear_mask_fill, range_clip=self.range_clip,
            obstacle_tolerance=self.obstacle_tolerance, goal_timeout=self.goal_timeout,
            delta_act_bounds=[self.delta_act_linear, self.delta_act_angular],
            delta_act_scale=self.delta_act_scale.tolist()))))

        # --- Active goal handle ---
        self._current_goal_handle: ServerGoalHandle | None = None

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(f"Running DRL policy - {self.model_name}")
        self.get_logger().info(f'Action mode {self.action_mode}')
        self.get_logger().info(
            f'Config: normalized_actions={self.normalized_actions} max_lin_vel={self.max_lin_vel} '
            f'max_angular_vel={self.max_angular_vel} antennae_mask_dist={self.antennae_mask_dist} '
            f'rear_mask_deg={self.rear_mask_deg} rear_mask_fill={self.rear_mask_fill} range_clip={self.range_clip} '
            f'delta_act=({self.delta_act_linear}, {self.delta_act_angular}) x {self.delta_act_scale.tolist()}')

    def _reset_states(self):
        self.action_last = np.zeros(2)
        self.action = np.zeros(2)
        self.d_goal_last = 0.0
        self.prev_abs_diff = 0.0
        self.min_dist_last = 0.0
        self._obs_buffer[:] = 0.0

    def _set_action_space(self):
        # set the low and high of the output action:
        self.action_low = np.array([0.0, -1.0], dtype = np.float32)
        self.action_high = np.array([1.0, 1.0], dtype = np.float32)

        if not self.normalized_actions:
            self.action_low = np.array([0.0, -self.max_angular_vel], dtype = np.float32)
            self.action_high = np.array([self.max_lin_vel, self.max_angular_vel], dtype = np.float32)

        if self.action_mode == "direct":    # policy outputs direct velocities
            self.action_space = gym.spaces.Box(low = self.action_low, high = self.action_high, dtype = np.float32)            
        elif self.action_mode == "delta":   # policy outputs change in velocities
            # Bounds on the actor output (delta action); scaled separately by delta_act_scale when accumulated
            self.delta_act_low  = np.array([-self.delta_act_linear, -self.delta_act_angular], dtype = np.float32)
            self.delta_act_high = np.array([ self.delta_act_linear,  self.delta_act_angular], dtype = np.float32)
            self.action_space = gym.spaces.Box(low = self.delta_act_low, high = self.delta_act_high, dtype = np.float32)
        else:
            raise ValueError(f"Unknown action_mode: {self.action_mode!r}, expected 'direct' or 'delta'")
        return self.action_space

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def lidar_callback(self, msg: LaserScan):
        ranges, rear_mask = preprocess_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            antennae_mask_dist=self.antennae_mask_dist, rear_mask_deg=self.rear_mask_deg,
            rear_fill=self.rear_mask_fill, range_clip=self.range_clip)

        self.latest_scan = copy.deepcopy(msg)
        # self.latest_scan.ranges = ranges_drl.tolist()
        self.latest_ranges = ranges
        self.latest_rear_mask = rear_mask

    def goal_callback(self, goal_request: NavigateToGoal):
        '''
        Called when a new goal request arrives
        '''
        goal_request_target_pose: PoseStamped = goal_request.target_pose
        self.get_logger().info(
            f"New goal received at: ({goal_request_target_pose.pose.position.x: 5.3f},"
            f"{goal_request_target_pose.pose.position.y: 5.3f})"
        )

        # Overwrite any current goal
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Changing the current goal.')
            try:
                self._current_goal_handle.abort()
            except:
                pass

        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        '''
        Called when a cancel request arrives 
        
        :param goal_handle: 
        '''
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        '''
        Main control loop that runs while the goal is active
        
        :param self: Description
        :param goal_handle: Description
        :type goal_handle: ServerGoalHandle
        '''
        self._current_goal_handle = goal_handle
        target = goal_handle.request.target_pose
        self.goal_tolerance = getattr(goal_handle.request, 'goal_tolerance', None) or self.goal_tolerance
        start = time.time()

        # Initialize feedback and result message
        feedback_msg = NavigateToGoal.Feedback()
        result_msg = NavigateToGoal.Result()

        # Reset states
        self._reset_states()

        total_distance = 0.0

        ctrl_freq = 50.0
        ctrl_period = 1.0 / ctrl_freq

        while rclpy.ok():
            ctrl_iter_start = time.time()
            elapsed_time = time.time() - start

            if not goal_handle.is_active:       # stop the robot if no goal handle
                self._publish_cmd(0.0, 0.0)
                self.action_last = np.zeros(2)
                result_msg.success = False
                result_msg.message = 'Goal handle inactive'
                return result_msg

            # --- Check for cancellation ---
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_cmd(0.0, 0.0)
                self.action_last = np.zeros(2)
                result_msg.success = False
                result_msg.message = 'Cancelled by client.'
                return result_msg
            
            # --- Check for timeout ---
            if elapsed_time >= self.goal_timeout:
                goal_handle.abort()
                self._publish_cmd(0.0, 0.0)
                self.action_last = np.zeros(2)
                result_msg.success = False
                result_msg.message = f'Goal timeout after {elapsed_time:.1f}s'
                result_msg.total_distance=float(total_distance)
                self.get_logger().warn(f'Goal timed out after {elapsed_time:.1f}s')
                return result_msg

            # --- Wait for first data ---
            if (self.latest_odom is None) or (self.latest_scan is None):
                self.get_logger().warn('Waiting for odometry and LiDAR...')
                time.sleep(ctrl_period)
                continue

            odom_age = self._get_msg_age(self.latest_odom.header.stamp)
            scan_age = self._get_msg_age(self.latest_scan.header.stamp)
            # self.get_logger().info(
            #     f'data age: odom={odom_age:.3f}s scan={scan_age:.3f}s',
            #     throttle_duration_sec=0.1
            # )

            x_prev = self.latest_odom.pose.pose.position.x
            y_prev = self.latest_odom.pose.pose.position.y

            # --- 1. Extract + Normalize observation ---
            obs = self._get_obs(self.latest_odom, target, self.latest_scan)

            obs_normed = self._normalize_obs(obs)

            # --- 2. Check termination conditions ---
            d_goal = obs[2]
            min_lidar = np.min(obs[9:])
            
            # self.get_logger().info(f'Minimum LiDAR reading: {min_lidar}')
            if d_goal <= self.goal_tolerance:
                goal_handle.succeed()
                self._publish_cmd(0.0, 0.0)
                self.action_last = np.zeros(2)
                result_msg.success=True
                result_msg.message='Goal reached.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().info(f'Goal reached. Distance to goal {d_goal}')
                return result_msg
            
            if min_lidar <= self.obstacle_tolerance:
                goal_handle.abort()
                self._publish_cmd(0.0, 0.0)
                self.action_last = np.zeros(2)
                result_msg.success=False
                result_msg.message=f'Obstacle hit with distance {min_lidar:6.3f}, index {np.argmin(obs[9:])}. Mission aborted.'
                result_msg.total_distance=float(total_distance)
                self.get_logger().info(result_msg.message)
                return result_msg
            
            # --- 3. DRL policy inference => action ---
            self.action = self._run_policy(obs_normed)          # vx, vyaw in moving agent frame

            
            if self.action_mode == 'direct':
                self.action = np.clip(self.action, self.action_low, self.action_high)
            elif self.action_mode == 'delta':
                self.action = np.clip(self.action_last + self.delta_act_scale * self.action, self.action_low, self.action_high)

            self._get_rewards(obs)

            if self.normalized_actions:
                v = float(self.action[0]) * self.max_lin_vel
                w = float(self.action[1]) * self.max_angular_vel
            else:
                v = float(self.action[0])
                w = float(self.action[1])

            # # --- DEBUG prints ---
            self.get_logger().info(
                # f'obs → (dx={obs[0]: 5.3f} | dy={obs[1]: 5.3f} | dg={obs[2]: 5.3f})'
                # f'\n(theta={p.arctan2(obs[4], obs[3])/np.pi*180: 5.2f} | phi={np.arctan2(obs[6], obs[5])/np.pi*180: 5.2f})'
                # f'\n(vx={obs[7]: 5.3f} | vyaw={obs[8]: 5.3f})'
                # f'Min LiDAR idx: {np.argmin(obs[9:]): 2d} | {np.min(obs[9:]): 5.4f} | '
                # f'\nlidar:{obs[9:]}'
                f'Policy: ({self.action[0]: 5.3f}, {self.action[1]: 5.3f}) | Commanded: ({v: 5.3f}, {w: 5.3f})'
            )
            self._publish_cmd(v, w)

            # --- Extra: Publish Feedback
            feedback_msg.distance_to_goal = float(d_goal)
            feedback_msg.elapsed_time = float(elapsed_time)
            feedback_msg.current_pose = self.latest_odom.pose.pose
            goal_handle.publish_feedback(feedback_msg)

            # --- Extra: Find total distance travelled over the last step
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            step_distance = np.sqrt((x - x_prev) ** 2 + (y - y_prev) ** 2)
            total_distance += step_distance
            x_prev, y_prev = x, y

            # --- Manage control frequency
            ctrl_iter_elapsed = time.time() - ctrl_iter_start
            ctrl_iter_remain = ctrl_period - ctrl_iter_elapsed
            if ctrl_iter_remain > 0:
                time.sleep(ctrl_iter_remain)

    def _publish_cmd(self, v: float, w: float):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = f'{self.agent_name}_base_link'
            cmd.twist.linear.x = v
            cmd.twist.angular.z = w
            self.cmd_pub.publish(cmd)

    def _get_msg_age(self, stamp_msg) -> float:
        return (self.get_clock().now() - Time.from_msg(stamp_msg)).nanoseconds * 1e-9

    def _yaw_from_quaternion(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return np.arctan2(siny_cosp, cosy_cosp, dtype=np.float32)

    def _get_obs(self, odom: Odometry, target: PoseStamped, scan: LaserScan) -> np.ndarray:
        '''
        Build the observation from /odom messages to match the DRL policy's observation
        The observation space contains
        (dx, dy, dgoal, s_theta, c_theta, s_phi, c_phi, vx, vy, LiDAR scans)
        
        :param odom: odometry message from /odom (filtered wheel odometry, LiDAR matching, and IMU)
        :type odom: Odometry

        :return: Description
        :rtype: ndarray
        '''

        # --- Extract raw odometry and sensor data ---
        agent_pos = odom.pose.pose.position                 # agent (x,y,z) in initial odom frame
        ori_quat = odom.pose.pose.orientation
        agent_yaw = self._yaw_from_quaternion(ori_quat)     # agent yaw in initial odom frame
        # agent_vx = odom.twist.twist.linear.x                # agent's velocity in the moving frame TODO - verify whether this is in the agent frame or world frame
        # agent_vyaw = odom.twist.twist.angular.z             # agent's yaw velocity along z in the moving frame
        agent_vx = self.action[0]
        agent_vyaw = self.action[1]

        goal_pos = target.pose.position

        # TODO - perform the calculations required to form the observation
        dx = goal_pos.x - agent_pos.x
        dy = goal_pos.y - agent_pos.y
        dgoal = np.sqrt(dx**2 + dy**2)

        bearing = np.arctan2(dy, dx, dtype=np.float32) % (2 * np.pi)
        heading = agent_yaw
        rel_bearing = - ((bearing - heading + np.pi) % (2*np.pi) - np.pi)
        
        c_heading = np.cos(heading, dtype=np.float32)
        s_heading = np.sin(heading, dtype=np.float32) 
        c_bearing = np.cos(rel_bearing, dtype=np.float32)
        s_bearing = np.sin(rel_bearing, dtype=np.float32)

        # LIDAR min pooling
        # raw = np.array(scan.ranges, dtype=np.float32)
        # raw = np.where(np.isfinite(raw), raw, scan.range_max)       # replace inf/nan with max LiDAR range values
        # self.get_logger().info(f"{np.where(np.array(scan.ranges) < 0.2)}")
        # self.get_logger().info(f"LiDAR scan points: {len(scan.ranges)}")

        # TODO - tune this masking
        # antennae_mask = raw <= 0.20
        # raw[antennae_mask] = scan.range_max                  # drop anything below the minimum LiDAR scan range 
        # raw[750:1050] = scan.range_max                       # negate influence of points behind the LiDAR 
        # # raw = np.clip(raw, 0.0, scan.range_max)
        # raw = np.clip(raw, scan.range_min, scan.range_max)
        # raw = np.flip(raw)
        n_groups = 18           # TODO - dynamically modify this according to the model loaded
        lidar_obs = pool_groups(self.latest_ranges, self.latest_rear_mask, n_groups,
                                fill_values=self.obs_mean[9:9 + n_groups])

        self._obs_buffer[0:3] = dx, dy, dgoal
        self._obs_buffer[3:5] = c_heading, s_heading
        self._obs_buffer[5:7] = c_bearing, s_bearing
        self._obs_buffer[7:9] = agent_vx, agent_vyaw
        self._obs_buffer[9:]  = lidar_obs

        return self._obs_buffer
        
    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        obs_normed = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        return np.clip(obs_normed, -self.clip_obs, self.clip_obs).astype(np.float32)

    def _get_rewards(self, obs):
        d_goal = obs[2]
        abs_diff = np.arctan2(obs[6], obs[5], dtype=np.float32)
        lidar_obs = obs[9:]
        min_dist = np.min(lidar_obs)
        min_dist_idx = np.argmin(lidar_obs)
        v_lin, v_ang = obs[7:9]

        if min_dist >= self.d_safe:
            #--- DISTANCE APPROACH REWARD:
            # this reward term incentivizes approaching the goal, and rewards 0 otherwise:
            rew_dist_approach = max((self.d_goal_last - d_goal), 0)

            #--- HEADING APPROACH REWARD:
            # this reward term incentivizes approaching the required heading, and rewards 0 otherwise
            rew_head_approach = max((self.prev_abs_diff - abs_diff), 0) if self.action[0] >= 0.05 else 0

            # zero the obstacle terms:
            rew_obs_dist = 0
            rew_obs_align = 0
        
        # when near an obstacle, focus on moving away
        else:
            rew_dist_approach = 0
            rew_head_approach = 0
            #--- OBSTACLE APPROACH PENALTY:
            rew_obs_dist = min((min_dist / (self.min_dist_last + 1e-6) - 1), 0)

            # REWARD FOR NOT BEING ALIGNED WITH OBSTACLES:
            if min_dist_idx >= self.lidar_idx_threshold and min_dist_idx < self.n_ray_groups - self.lidar_idx_threshold and v_lin >= 0.05:
                rew_obs_align = 1
            else:
                if (min_dist_idx < self.lidar_idx_threshold and v_ang > 0) or (min_dist_idx >= self.n_ray_groups - self.lidar_idx_threshold and v_ang < 0):
                    rew_obs_align = min(1, np.abs(v_ang))
                else:
                    rew_obs_align = 0

        #--- PENALIZE ABRUPT CHANGES IN VELOCITY
        act_diff = np.abs(self.action - self.action_last)          # penalize both abrupt changes in linear and angular velocities
        rew_act_diff = -0.5 * np.sum(act_diff ** 2)

        #--- PENALIZE STALLING:
        if abs(self.action[0]) <= 0.05:
            rew_time = 2 * self.rew_time
        else:
            rew_time = self.rew_time

        # Scaling 
        self.rew_dist_approach_scaled   = self.rew_dist_approach_scale  * rew_dist_approach
        self.rew_head_approach_scaled   = self.rew_head_approach_scale  * rew_head_approach
        self.rew_obs_dist_scaled        = self.rew_obs_dist_scale       * rew_obs_dist
        self.rew_obs_align_scaled       = self.rew_obs_align_scale      * rew_obs_align

        #--- TOTAL REWARD:
        rew = 0
        rew += self.rew_dist_approach_scaled + self.rew_head_approach_scaled
        rew += self.rew_obs_dist_scaled + self.rew_obs_align_scaled
        rew += rew_act_diff
        rew += rew_time

        self.d_goal_last = d_goal
        self.prev_abs_diff = abs_diff
        self.min_dist_last = min_dist
        self.action_last = self.action

        # self.get_logger().info(f"r_dist_app: {self.rew_dist_approach_scaled: 5.3f} | "
        #                        f"r_head_app: {self.rew_head_approach_scaled: 5.3f} | "
        #                        f"r_obs_dist: {self.rew_obs_dist_scaled: 5.3f} | "
        #                        f"r_obs_align: {self.rew_obs_align_scaled: 5.3f} | "
        #                        f"rew_act_diff: {rew_act_diff: 5.3f} | "
        #                        f"r_time: {self.rew_time: 4.2f} | "
        #                        f"r_total: {rew: 5.3f}  ")

    def _run_policy(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)
            raw_action = self.policy(obs_tensor).squeeze(0).cpu().numpy()

        # rescale from tanh output range [-1,1] to the actual action_space bounds
        low, high = self.action_space.low, self.action_space.high
        action = low + 0.5 * (raw_action + 1.0) * (high - low)
        return action

def main():
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = DRLPolicyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    stop_event = threading.Event()

    def _request_shutdown(*args):
        '''Set the threading event to signal a shutdown request'''
        stop_event.set()

    # catch sigterm from GUI:
    signal.signal(signal.SIGINT, _request_shutdown)     # from Ctrl+C in the terminal
    signal.signal(signal.SIGTERM, _request_shutdown)    # from OS PID kill commands

    try: 
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        for _ in range(5):
            node._publish_cmd(0.0, 0.0)
            time.sleep(0.02)

        executor.shutdown()
        node.destroy_node()
        # if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
