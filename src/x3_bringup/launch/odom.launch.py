from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.descriptions import ParameterFile


from launch import LaunchDescription

from launch_ros.actions import Node 

import os

def generate_launch_description():
    # ===== SET REQUIRED PATHS =====
    pkg_path = get_package_share_directory("x3_bringup")
    ekf_params_path = os.path.join(pkg_path, "config", "ekf_params.yaml")

    # ===== DEFINE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )

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
            "laser_scan_topic" : "scan",
            "odom_topic" : "odom_rf2o",
            "publish_tf" : True,
            "base_frame_id" : PythonExpression(["'", agent_name, "_base_footprint'"]),
            "odom_frame_id" : PythonExpression(["'", agent_name, "_odom'"]),
            "init_pose_from_topic" : "",
            "freq" : 60.0}],
    )

    # Covariance filter node to publish IMU + LiDAR + wheel encoder covariance
    covariance_filter_node = Node(
        package="x3_covariance_filter",
        executable="covariance_filter",
        name="covariance_filter_node",
        namespace = agent_name,
        output="screen",
        parameters = [{
            "agent_name": agent_name
        }]
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
        laser_scan_matcher_node,
        covariance_filter_node,
        ekf_odom_node
    ])