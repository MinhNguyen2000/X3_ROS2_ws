"""Odometry chain of the robot for a bag replay (scripts/sim2real/ekf_compare.py): lidar_rear_mask, rf2o, covariance
filter and EKF of x3_bringup on the bag's clock, with the robot's TF tree from the URDF.

  ros2 launch scripts/sim2real/replay_odom.launch.py odom_profile:=_robot_20261029
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

import math


def generate_launch_description():
    agent = 'agent0'
    bringup = get_package_share_directory('x3_bringup')
    urdf = os.path.join(get_package_share_directory('x3_description'), 'urdf', 'x3.urdf.xacro')
    odom_profile = LaunchConfiguration('odom_profile')

    description = ParameterValue(
        Command(['xacro ', urdf, f' agent_name:={agent} use_ros_control:=false is_gazebo:=false']), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('odom_profile', default_value=''),
        SetParameter(name='use_sim_time', value=True),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=agent,
             parameters=[{'robot_description': description}]),
        # same parameters as robot.launch.py
        Node(package='x3_bringup', executable='lidar_rear_mask', name='lidar_filter', namespace=agent,
             parameters=[{'input_topic': 'scan', 'output_topic': 'scan_filtered', 'mask_center_angle': math.pi,
                          'mask_half_width': 0.90}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'odom.launch.py')),
            launch_arguments={'rf2o_scan_topic': 'scan_filtered', 'odom_profile': odom_profile}.items()),
    ])
