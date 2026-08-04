import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== DEFINE REQUIRED PATHS =====
    description_pkg_dir = get_package_share_directory("x3_description")

    # ===== DECLARE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(description_pkg_dir, "rviz", "real.rviz")]
    )

    image_uncompress_node = Node(
        package="x3_bringup",
        executable="image_republisher",
        name="image_republisher",
        namespace=agent_name
    )

    return LaunchDescription([
        agent_name_arg,
        rviz,
        image_uncompress_node
    ])