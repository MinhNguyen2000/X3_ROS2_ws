#!/usr/bin/env python3
"""
trial_orchestrator.py
 
Point-to-point navigation trial runner for comparing planners (TD3 / APF) across a shared, fixed set
of goal poses in a single persistent headless Gazebo world.

Persistence model
    - Gazebo server, robot_state_publisher, ros2_gz_bridge, controller spawners, and odom_to_vel_raw
    - agent_spawner run ONCE at test batch start
    - odom_offset_node run ONCE at test batch start (fixed agent start pose, only varied goal pose)
      TODO - consider change up the start poses, means odom_offset_node need to be restarted 
             to accept new x_offset and y_offset
    - policy_node / apf_node are killed and restarted for every trial to reset stale states
      (self.action / self.d_goal_last / self.prev_abs_diff / self.min_dist_last across goals)
    - rf2o_laser_odometry + covariance_filter + ekf_node (lauched via launch_odom.py) are killed
      and restarted every trial. All planners use the ground truth odometry from /odom published by
      Gazebo's OdometryPublisher plugin instead of the /odom_ekf from EKF odometry fusion. However,
      EKF odometry estimations are logged to compared against ground truth (related to agressiveness
      of the policy and sim2real considerations). This restart does not have bearing on the success 
      and distance scoring, and exists purely so that EKF/rf2o divergence vs ground truth is logged
      fresh, matching real deployment.

Odometry sources:
    - Odometry ground truth from the /odom topic is used for scoring the policies (for example
      success/collision/distance) and not from /odom_ekf to avoid conflating localization error with 
      navigation performance.
    - /odom_ekf is passively logged in parallel for diagnostic (per-trial deviation from ground truth) 
      to be analyzed against commanded / actual velocity aggressiveness (TD3_direct bang bang behavior 
      vs TD3_delta with limited velocity change vs APF) in a separate analysis.

Per-trial output
    - One row appended to a batch-level summary CSV (numeric/scalr fields only, logged as TrialResult
      data structure) for statistical analysis across all trials
    - One JSON file with the full time-series buffers for each trial, including ground-truth odom, EKF
      odom estimation, commanded cmd_vel, actual/measured wheel velocity. Each JSON is correlated to a 
      row in the CSV via TrialResult.timeseries_log_path 
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field, fields
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Odometry

from x3_nav_interfaces.action import NavigateToGoal

from ament_index_python.packages import get_package_share_directory

# ===== DATA STRUCTURE (config/result) =====
@dataclass
class TrialConfig:
    world_name:                 str
    agent_name:                 str = "agent0"

    # Policy node parameters
    planner:                    str = "DRL"
    drl_model_name:             str = "TD3_00404_1000"
    goal_tolerance:             float = 0.2
    goal_timeout_s:             float = 90.0
    max_lin_vel:                float = 0.5
    max_angular_vel:            float = 1.0

    settle_steps_s:             float = 1.0
    velocity_zero_threshold:    float = 0.02   # make sure zero velocity after agent set pose teleport
    velocity_zero_timeout_s:    float = 5.0
    node_ready_timeout_s:       float = 15.0
    output_dir:                 str = "trial_results"

    # Gazebo spawn initial conditions
    agent_start_x:              float = -3.0
    agent_start_y:              float = -3.0
    agent_start_yaw:            float = 0.0

@dataclass
class TrialResult:
    trial_idx:                  int
    world_name:                 str
    planner:                    str
    drl_model_name:             str
    goal_x:                     float
    goal_y:                     float
    straight_line_distance:     float       # start-to-goal straight-line distance (m)
    success:                    bool
    failure_reason:             str
    distance_traveled_odom:     float       # traveled distance calculated from ground truth odom buffer
    distance_traveled_planner:  float    # traveled distance reported by the planner
    distance_diff_percent:      float       # Discrepancy between odom-derived distance and planner distance
    travel_time:                float       # time from goal-send to termination
    final_distance_to_goal:     float
    timeseries_log_path:        str = ""
    timestamp:                  str = field(default_factory=lambda: datetime.now().isoformat())

class ManagedProcess:
    '''
    Wraps a `ros2 run` / `ros2 launch` subproces in its own process group so
    it (and any child node spawn) can be cleanly interrupted with a SIGINT.

    Use for the following, which should be restarted every trial:
    - planner (policy_node/apf_node)
    - launch_odom.py stack (rf2o + covariance_filter + ekf_node)
    '''

    def __init__(self, cmd: list[str], name: str):
        self.cmd = cmd
        self.name = name
        self.proc: Optional[subprocess.Popen] = None

    def start(self):
        self.proc = subprocess.Popen(
            self.cmd,
            preexec_fn=os.setsid,
        )

    def stop(self, timeout: float=5.0):
        if self.proc is None or self.proc.poll() is not None:
            return
        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            self.proc.wait()

    def is_alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

class GazeboInterface:
    '''
    Wrapper around Gazebo transport service calls for teleporting the agent.
    '''

    def __init__(self, world_name: str, agent_name: str):
        self.world_name = world_name
        self.agent_name = agent_name

    def teleport(self, x: float, y: float, yaw: float = 0.0):
        # TODO - properly convert yaw -> quaternion (currently placeholder 
        # for straight-ahead orientation)
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        req = (
            f"name: '{self.agent_name}', position: {{x: {x}, y: {y}, z: {0.0}}}"
            # f", orientation: {{z: {qz}, w: {qw}}}"
        )
        cmd = [
            "ign", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000",
            "--req", req
        ]

        # TODO - check subprocess return code instead of assuming success
        subprocess.run(cmd, check=True)

    def zero_velocity(self):
        # TODO - implement a set_twist / set_velocity call to eliminate residual motion
        # from previous trial does not carry into the teleport
        raise NotImplementedError

# ===== HELPER FUNCTIONS =====
def yaw_from_quaternion(q: Quaternion) -> float:
    '''Extract yaw angle from (x,y,z,w) quaternion'''
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
    return math.atan2(siny_cosp, cosy_cosp)

def stamp_to_sec(stamp: Time) -> float:
    '''Converts a builtin_interfaces/Time header stamp to float seconds'''
    return stamp.sec + stamp.nanosec * 1e-9

# ===== TIME SERIES LOGGER =====
class TrialTimeseriesLogger:
    '''
    Class to accumulate time series buffers for a single trial and writes to a JSON file
    '''

    def __init__(self):
        self.reset()

    def reset(self):
        self.gt_odom:       list[dict] = []     # ground truth: t, x, y, yaw
        self.ekf_odom:      list[dict] = []     # EKF odometry estimation: t, x, y, yaw
        self.cmd_vel:       list[dict] = []     # commanded: t, vx. vyaw
        self.actual_vel:    list[dict] = []     # measured wheel odom: t, vx, vyaw

    def log_gt_odom(self, t, x, y, yaw):
        self.gt_odom.append({"t": t, "x": x, "y": y, "yaw": yaw})

    def log_ekf_odom(self, t, x, y, yaw):
        self.ekf_odom.append({"t": t, "x": x, "y": y, "yaw": yaw})

    def log_cmd_vel(self, t, vx, vyaw):
        self.cmd_vel.append({"t": t, "vx": vx, "vyaw": vyaw})

    def log_actual_vel(self, t, vx, vyaw):
        self.actual_vel.append({"t": t, "vx": vx, "vyaw": vyaw})

    def save(self, path: str, metadata: dict):
        data = {
            "metadata": metadata,
            "gt_odom":  self.gt_odom,
            "ekf_odom": self.ekf_odom,
            "cmd_vel":  self.cmd_vel,
            "actual_vel": self.actual_vel,
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

# ===== ORCHESTRATOR NODE =====
class TrialOrchestrator(Node):
    def __init__(self, config: TrialConfig):
        super().__init__("trial_orchestrator")
        self.cfg = config
        self.gz = GazeboInterface(config.world_name, config.agent_name)

        # Ground truth trajectory buffer used for scoring, separated from TrialTimeseriesLogger
        # since this scorings only needs (t, x, y)
        self._gt_trajectory: list[tuple[float, float, float, float]] = []     # (t, x, y, yaw)

        # Per-trial time-series logger (ground truth odom, EKF odom, cmd_vel, actual velocity)
        # Save to one JSON file per trial and reset at the start of every trial
        self.ts_logger = TrialTimeseriesLogger()

        # === ROS Interfaces ===
        self._nav_client = ActionClient(self, NavigateToGoal, f"{self.cfg.agent_name}/navigate_to_goal")

        self._odom_sub = self.create_subscription(
            Odometry,
            f"{self.cfg.agent_name}/odom",
            self._odom_callback,
            10
        )
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            f"{self.cfg.agent_name}/odom_ekf",
            self._ekf_odom_callback,
            10
        )
        self._cmd_vel_sub = self.create_subscription(
            TwistStamped,
            f"{self.cfg.agent_name}/cmd_vel",
            self._cmd_vel_callback,
            10
        )
        self._actual_vel_sub = self.create_subscription(
            TwistStamped,
            f"{self.cfg.agent_name}/vel_raw",
            self._actual_vel_callback,
            10
        )

        # Lifecycle processes to be restarted after each trial
        self._planner_proc:         Optional[ManagedProcess] = None
        self._ekf_odom_stack_proc:  Optional[ManagedProcess] = None

        # Safeguard measures to warn zero/unset header stamp or non-monotonic
        # timestamps (received stamp earlier than last stamp). A warning is logged
        # at most once to avoidd flooding the log
        self._last_stamp:   dict[str, float] = {}   # last time stamp of each topic (odom, odom_ekf, cmd_vel, vel_raw)
        self._stamp_warned: set[str] = set()        # check if warning has been added at least once per topic

        # Logging gating for the subscription callback
        # (only set to True to log navigation data immediately before send_goal_async())
        self._logging_active = False

    def _check_stamp(self, topic: str, t: float):
        '''
        Check the header timestamp to warn:
        1. zero/unset stamp,
        2. non-monotonic arrival (received stamp earlier than last stamp)
        
        Does not drop the sample or stop the trial, only debug measures to warn
        misaligned timestamp per topic => negative effect on downstream interpolation
        and analysis
        '''

        # Check for zero/unset header stamp
        if t <= 0.0 and f"{topic}:zero" not in self._stamp_warned:
            self.get_logger().warn(f"{topic}: header stamp is zero/unset")
            self._stamp_warned.add(f"{topic}:zero")

        # Check for non-monotonic time stamp
        last = self._last_stamp.get(topic)
        if last is not None and t < last and f"{topic}:nonmonotonic" not in self._stamp_warned:
            self.get_logger().warn(f"{topic}: non-monotonic header stamp detected ({t: .4f} < {last: .4f})")
            self._stamp_warned.add(f"{topic}:nonmonotonic")
        self._last_stamp[topic] = t


    # === Subscription callbacks ===
    def _odom_callback(self, msg: Odometry):
        if not self._logging_active:
            return
        t = stamp_to_sec(msg.header.stamp)
        self._check_stamp("odom", t)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._gt_trajectory.append((t, x, y, yaw))
        self.ts_logger.log_gt_odom(t, x, y, yaw)

    def _ekf_odom_callback(self, msg: Odometry):
        if not self._logging_active:
            return
        t = stamp_to_sec(msg.header.stamp)
        self._check_stamp("odom_ekf", t)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.ts_logger.log_ekf_odom(t, x, y, yaw)

    def _cmd_vel_callback(self, msg: TwistStamped):
        if not self._logging_active:
            return
        t = stamp_to_sec(msg.header.stamp)
        self._check_stamp("cmd_vel", t)
        vx = msg.twist.linear.x
        vyaw = msg.twist.angular.z
        self.ts_logger.log_cmd_vel(t, vx, vyaw)

    def _actual_vel_callback(self, msg: TwistStamped):
        if not self._logging_active:
            return
        t = stamp_to_sec(msg.header.stamp)
        self._check_stamp("vel_raw", t)
        vx = msg.twist.linear.x
        vyaw = msg.twist.angular.z
        self.ts_logger.log_actual_vel(t, vx, vyaw)

    # ===== Navigation Planner Lifecycle =====
    def start_planner(self):
        '''Restart policy_node (DRL) or apf_node, depending on self.cfg.planner'''
        if self.cfg.planner == "DRL":
            # TODO - design decision: should I also include model_name in the self.cfg
            cmd = [
                "ros2", "run", "x3_drl_policy", "policy_node",
                "--ros-args",
                "--remap", f"__ns:=/{self.cfg.agent_name}",
                "-p", "use_sim_time:=true",
                "-p", f"agent_name:={self.cfg.agent_name}",
                "-p", f"model_name:={self.cfg.drl_model_name}",
                "-p", f"goal_tolerance:={self.cfg.goal_tolerance}",
                "-p", f"goal_timeout:={self.cfg.goal_timeout_s}",
                "-p", f"max_lin_vel:={self.cfg.max_lin_vel}",
                "-p", f"max_angular_vel:={self.cfg.max_angular_vel}",
            ]
        elif self.cfg.planner == "APF":
            # TODO - design decision: should I also include d_safe, n_ray_groups, k_att, ... in self.cfg
            cmd = [
                "ros2", "run", "x3_apf_planner", "apf_node",
                "--ros-args",
                "--remap", f"__ns:=/{self.cfg.agent_name}",
                "-p", "use_sim_time:=true",
                "-p", f"agent_name:={self.cfg.agent_name}",
                "-p", f"goal_tolerance:={self.cfg.goal_tolerance}",
                "-p", f"goal_timeout:={self.cfg.goal_timeout_s}",
                "-p", f"max_lin_vel:={self.cfg.max_lin_vel}",
                "-p", f"max_angular_vel:={self.cfg.max_angular_vel}",
            ]
        else:
            raise ValueError(f"Unknown planner: {self.cfg.planner}")

        self._planner_proc = ManagedProcess(cmd, f"planner_{self.cfg.planner}")
        self._planner_proc.start()
        self._wait_for_planner_ready()

    def stop_planner(self):
        '''Stop the planner ManagedProcess object (if exists)'''
        if self._planner_proc:
            self._planner_proc.stop()
            self._planner_proc = None

    def _wait_for_planner_ready(self):
        '''Block until planner node is ready'''
        if not self._nav_client.wait_for_server(timeout_sec=self.cfg.node_ready_timeout_s):
            raise TimeoutError("navigate_to_goal action server did not become available in time")

    # ===== EKF Odometry Estimation LifeCycle =====
    def start_odom_stack(self):
        '''Restart rf2o + covariance_filter + ekf_node (via x3_description's launch_odom.py script)'''
        cmd = [
            "ros2", "launch", "x3_description", "launch_odom.py",
            f"agent_name:={self.cfg.agent_name}",
            "use_sim_time:=true",
        ]

        self._ekf_odom_stack_proc = ManagedProcess(cmd, "odom_stack")
        self._ekf_odom_stack_proc.start()
        self._wait_for_ekf_ready()

    def stop_odom_stack(self):
        '''Stop the odometry stack (rf2o + covariance_filter + ekf)'''
        if self._ekf_odom_stack_proc:
            self._ekf_odom_stack_proc.stop()
            self._ekf_odom_stack_proc = None
        

    def _wait_for_ekf_ready(self):
        '''
        Block until the freshly-restarted EKF stack is up and publishing
        Does NOT wait for EKF convergence
        '''
        deadline = time.time() + self.cfg.node_ready_timeout_s
        while time.time() < deadline:
            if self.count_publishers(f"{self.cfg.agent_name}/odom_ekf") > 0:
                return
            time.sleep(0.2)
        raise TimeoutError("EKF /odom_ekf did not become available in time")

    # ===== Teleport / reset between trials =====
    def reset_scene_for_next_trial(self):
        ''' 
        Full per-trial reset sequence:
        1. Publish stop command: should already be stopped from goal termination
           TODO - check if agent is actually stop when goal reached
        2. Wait for measured velocity to settle near zero
        3. Teleport agent back to the fixed start pose + zero velocity
        4. Step/settle sim briefly to clear any teleport contact transient
        5. Restart the planner and odom stack'''

        # TODO: 1 - publish a zero velocity cmd_vel to stop the robot

        # TODO: 2 - poll wheel/odom twist until below self.cfg.velocity_zero_threshold
        # with a timeout of self.cfg.velocity_zero_timeout_s. Log a warning if timeout hit

        self.get_logger().info('Teleporting agent to start pose')
        self.gz.teleport(
            self.cfg.agent_start_x,
            self.cfg.agent_start_y,
            self.cfg.agent_start_yaw
        )


        # TODO - callc self.gz.zero_velocity() once implemented

        time.sleep(self.cfg.settle_steps_s)

        self.get_logger().info('Stopping the odometry stack')
        self.stop_odom_stack()
        self.get_logger().info('Stopping navigation planner')
        self.stop_planner()
        self.get_logger().info('Starting the odometry stack')
        self.start_odom_stack()
        self.get_logger().info('Starting navigation planner')
        self.start_planner()

        time.sleep(self.cfg.settle_steps_s)

    # ===== Single trial execution =====
    def run_single_trial(self, trial_idx: int, goal_x: float, goal_y: float) -> TrialResult:
        # Reset time-series buffers and TrialTimeseriesLogger
        self._gt_trajectory = []
        self.ts_logger.reset()

        # Reset last time stamp and header stamp warnings
        self._last_stamp = {}
        self._stamp_warned = set()

        straight_line_distance = round(math.hypot(goal_x, goal_y), 3)

        goal_msg = NavigateToGoal.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = f"{self.cfg.agent_name}_odom"
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = goal_x
        goal_msg.target_pose.pose.position.y = goal_y
        goal_msg.goal_tolerance = self.cfg.goal_tolerance

        start_time = time.time()

        # Only starts logging from here until the outcome is known 
        # (failure modes or normal completion) in this trial. Using 
        # try/finally to ensure the flag is cleared on every exit.
        try: 
            self._logging_active = True
            send_future: Future = self._nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle: ClientGoalHandle = send_future.result()

            # Failure mode 1 - Goal rejected by the action server
            if not goal_handle.accepted:
                result = TrialResult(
                    trial_idx                   = trial_idx,
                    world_name                  = self.cfg.world_name,
                    planner                     = self.cfg.planner,
                    drl_model_name              = self.cfg.drl_model_name if self.cfg.planner == "DRL" else None,
                    goal_x                      = goal_x,
                    goal_y                      = goal_y,
                    straight_line_distance      = straight_line_distance,
                    success                     = False,
                    failure_reason              = "goal_rejected",
                    distance_traveled_odom      = round(0.0, 3.0),
                    distance_traveled_planner   = round(0.0, 3.0),
                    distance_diff_percent       = float("nan"),
                    travel_time                 = round(0.0, 3.0),
                    final_distance_to_goal      = float("nan")
                )
                return self._finalize_trial(result)

            result_future: Future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=self.cfg.goal_timeout_s + 10.0
            )
            self.get_logger().info(f"Total distance traveled: {result_future.result().result.total_distance: 5.3f}")

            travel_time = time.time() - start_time

            # Failure model 2 - Task never finished (any of succeed / abort / canceled)
            # Possible reasons: 
            # 1. Task still running, 
            # 2. Server crashed/was killed/is stuck
            # 3. Single threaded server executor deadlock
            # 4. Action server never returned a legitimate goal_handle
            if not result_future.done():
                result = TrialResult(
                    trial_idx                   = trial_idx,
                    world_name                  = self.cfg.world_name,
                    planner                     = self.cfg.planner,
                    drl_model_name              = self.cfg.drl_model_name if self.cfg.planner == "DRL" else None,
                    goal_x                      = goal_x, 
                    goal_y                      = goal_y,
                    straight_line_distance      = straight_line_distance,
                    success                     = False, 
                    failure_reason              = "action_request_unfinished",
                    distance_traveled_odom      = round(0.0, 3.0),
                    distance_traveled_planner   = round(0.0, 3.0),
                    distance_diff_percent       = float("nan"),
                    travel_time                 = round(travel_time, 3.0),
                    final_distance_to_goal      = float("nan"),
                )
                return self._finalize_trial(result)

            nav_result: NavigateToGoal.Result = result_future.result().result
        finally:
            # Stop logging new samples as soon as the outcome is known to
            # prevent data from post-trial processes (teardown, scene reset)
            # from leaking into this trial's buffers.
            self._logging_active = False

        # Calculate total distance traveled from ground truth odometry trajectory
        # buffer collected throughout this run. Should roughly agree with the total
        # distance in nav_result.total_distance
        distance_traveled_odom = self._compute_path_length(self._gt_trajectory)
        distance_traveled_planner = nav_result.total_distance
        distance_diff = abs(distance_traveled_odom - distance_traveled_planner)
        distance_diff_percent = distance_diff / distance_traveled_odom * 100 if distance_traveled_odom > 0 else float("nan")
        final_distance_to_goal = self._compute_final_distance(
            self._gt_trajectory, goal_x, goal_y
        )

        result = TrialResult(
            trial_idx                   = trial_idx,
            world_name                  = self.cfg.world_name,
            planner                     = self.cfg.planner,
            drl_model_name              = self.cfg.drl_model_name if self.cfg.planner == "DRL" else None,
            goal_x                      = goal_x,
            goal_y                      = goal_y,
            straight_line_distance      = straight_line_distance,
            success                     = nav_result.success,
            failure_reason              = "" if nav_result.success else nav_result.message,
            distance_traveled_odom      = round(distance_traveled_odom,     3),
            distance_traveled_planner   = round(distance_traveled_planner,  3),
            distance_diff_percent       = round(distance_diff_percent,      3),
            travel_time                 = round(travel_time,                3),
            final_distance_to_goal      = round(final_distance_to_goal,     3),
        )
        return self._finalize_trial(result)

    def _finalize_trial(self, result: TrialResult) -> TrialResult:
        '''Save the timeseries JSON (odom/odom_ekf/cmd_vel/actual_vel) with metadata'''
        timeseries_dir = os.path.join(self.cfg.output_dir, "timeseries")
        os.makedirs(timeseries_dir, exist_ok=True)

        world_name = result.world_name
        planner_name = result.drl_model_name or result.planner
        timeseries_log_path = os.path.join(timeseries_dir, f"{world_name}_{planner_name}_trial{result.trial_idx:03d}.json")

        self.ts_logger.save(
            timeseries_log_path,
            metadata={
                "trial_idx":        result.trial_idx,
                "world_name":       result.world_name,
                "planner":          result.planner,
                "drl_model_name":   result.drl_model_name,
                "goal_x":           result.goal_x,
                "goal_y":           result.goal_y,
            }
        )
        result.timeseries_log_path = timeseries_log_path
        return result


    @staticmethod
    def _compute_path_length(trajectory: list[tuple[float, float, float, float]]) -> float:
        if len(trajectory) < 2:
            return 0.0
        total = 0.0
        for (_, x0, y0, _), (_, x1, y1, _) in zip(trajectory, trajectory[1:]):
            total += math.hypot(x1 - x0, y1 - y0)
        return total

    @staticmethod
    def _compute_final_distance(
        trajectory: list[tuple[float, float, float, float]],
        goal_x: float,
        goal_y: float
    ) -> float:
        if not trajectory:
            return float("nan")
        _, x, y, _ = trajectory[-1]
        return math.hypot(goal_x - x, goal_y - y)

    # Run a batch of N trials
    def run_batch(self, goals: list[dict]):
        os.makedirs(self.cfg.output_dir, exist_ok=True)
        if self.cfg.planner == "DRL":
            csv_filename = f"{self.cfg.world_name}_{self.cfg.drl_model_name}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        elif self.cfg.planner == "APF":
            csv_filename = f"{self.cfg.world_name}_{self.cfg.planner}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        else:
            raise ValueError(f"Unknown planner: {self.cfg.planner}")
        
        out_path = os.path.join(
            self.cfg.output_dir,
            csv_filename,
        )

        results: list[TrialResult] = []

        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=[f.name for f in fields(TrialResult)])
            writer.writeheader()

            for idx, goal in enumerate(goals):
                # TODO: wrap this in try/except so a single trial's exception (e.g. a failed teleport call) 
                # doesn't kill the whole batch => log it as a failed trial and continue, but make sure the 
                # exception is visible afterward (don't swallow silently).
                self.reset_scene_for_next_trial()
                self.get_logger().info(
                    f"Goal [{idx + 1}/{len(goals)}] | Planner={self.cfg.planner} | "
                    f"goal=({goal['x']: .2f},{goal['y']: .2f})"
                )
                result = self.run_single_trial(idx, goal["x"], goal["y"])
                results.append(result)
 
                writer.writerow(result.__dict__)
                f.flush()  # so a crash mid-batch doesn't lose completed trials

        self.get_logger().info(f"Batch complete. Results written to {out_path}")
        return results


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--world_name", default="world_1", help="Gazebo world name", 
        choices=["world_1", "world_2", "world_3", "world_4", "world_5"]
    )
    parser.add_argument("--agent_name", default="agent0")
    parser.add_argument("--planner", required=True, choices=["DRL", "APF"])
    parser.add_argument("--output_dir", default="trial_results")

    return parser.parse_args()

def load_goals(path: str) -> list[dict]:
    '''
    Expected JSON format:
    [
      {"x": 1.0, "y": 2.0},
      {"x": 5.0, "y": 1.0},
      ...
    ]'''
    with open(path) as f:
        return json.load(f)

def main():
    args = parse_arguments()

    pkg_dir = get_package_share_directory("x3_experiment_runner")
    goals_path = os.path.join(pkg_dir, "goals", f"goals_{args.world_name}.json")
    goals = load_goals(goals_path)

    cfg = TrialConfig(
        world_name=args.world_name,
        agent_name=args.agent_name,
        planner=args.planner,
        output_dir=args.output_dir,
    )

    # TODO: this script assumes Gazebo + rsp + bridges + spawner +
    # controllers are ALREADY running (i.e. you've separately launched a
    # trimmed version of gazebo.launch.py with odom_nodes / the planner
    # node removed, since both are managed per-trial by this script).
    # Consider adding a --launch-gazebo flag that shells out to that
    # trimmed launch file first and waits for it to be ready, so the whole
    # batch is a single command.

    rclpy.init()
    node = TrialOrchestrator(cfg)

    try:
        node.run_batch(goals)
    finally:
        node.stop_planner()
        node.stop_odom_stack()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()