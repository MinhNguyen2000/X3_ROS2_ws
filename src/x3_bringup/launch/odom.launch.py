from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution

from launch_ros.descriptions import ParameterFile


from launch import LaunchDescription

from launch_ros.actions import Node 

import os

def generate_launch_description():
    # ===== SET REQUIRED PATHS =====
    pkg_path = get_package_share_directory("x3_bringup")
    config_dir = os.path.join(pkg_path, "config")

    # ===== DEFINE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )

    rf2o_scan_topic = LaunchConfiguration("rf2o_scan_topic")
    rf2o_scan_topic_arg = DeclareLaunchArgument(
        "rf2o_scan_topic",
        default_value = "scan",
        description = "LaserScan topic rf2o reads; robot.launch.py passes scan_filtered (rear returns zeroed)"
    )

    # '' selects config/ekf_params.yaml and covariance_params.yaml, '_robot_20261029' the robot-local pair
    odom_profile = LaunchConfiguration("odom_profile")
    odom_profile_arg = DeclareLaunchArgument(
        "odom_profile",
        default_value = "",
        description = "Suffix of the EKF and covariance parameter files in config/"
    )
    ekf_params_path = [config_dir, "/ekf_params", odom_profile, ".yaml"]
    covariance_params_path = [config_dir, "/covariance_params", odom_profile, ".yaml"]

    # ===== NODES & LAUNCH DESCRIPTION =====
    # LiDAR scan matcher package
    laser_scan_matcher_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        namespace = agent_name,
        output="log",
        arguments = ["--ros-args", "--log-level", 
                     PythonExpression(["'", agent_name, ".rf2o_laser_odometry:=error'"])],
        parameters=[{
            "laser_scan_topic" : rf2o_scan_topic,
            "odom_topic" : "odom_rf2o",
            "publish_tf" : False,            # the EKF is the only publisher of odom -> base_footprint
            "base_frame_id" : PythonExpression(["'", agent_name, "_base_footprint'"]),
            "odom_frame_id" : PythonExpression(["'", agent_name, "_odom'"]),
            "init_pose_from_topic" : "",
            "freq" : 10.0}],                 # the RPLidar scan rate
    )

    # Covariance filter node to publish IMU + LiDAR + wheel encoder covariance
    covariance_filter_node = Node(
        package="x3_covariance_filter",
        executable="covariance_filter_node",
        name="covariance_filter_node",
        namespace = agent_name,
        output="screen",
        parameters = [{
            "agent_name": agent_name
        }, ParameterFile(covariance_params_path, allow_substs=True)]
    )

    # EKF node
    ekf_odom_node = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_odom_node",
        namespace = agent_name,
        output = "screen",
        parameters = [ParameterFile(ekf_params_path, allow_substs=True)],
        remappings = [("odometry/filtered", "odom")]
    )

    return LaunchDescription([
        agent_name_arg,
        rf2o_scan_topic_arg,
        odom_profile_arg,
        laser_scan_matcher_node,
        covariance_filter_node,
        ekf_odom_node
    ])