import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_rviz(context, *args, **kwargs):
    description_pkg_dir = get_package_share_directory("x3_description")

    # Fast DDS profile for rviz2 only; fastdds_profile:=none leaves DDS at its defaults
    fastdds_profile = LaunchConfiguration("fastdds_profile").perform(context)
    rviz_env = {} if fastdds_profile in ("", "none") else {"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile}

    return [Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(description_pkg_dir, "rviz", "real.rviz")],
        additional_env=rviz_env
    )]


def generate_launch_description():
    # ===== DEFINE REQUIRED PATHS =====
    bringup_pkg_dir = get_package_share_directory("x3_bringup")

    # ===== DECLARE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )

    fastdds_profile_arg = DeclareLaunchArgument(
        "fastdds_profile",
        default_value = os.path.join(bringup_pkg_dir, "config", "fastdds_wifi_only.xml"),
        description = "Fast DDS profile for rviz2 that limits DDS to the lab Wi-Fi address (halves the camera "
                      "stream from the Pi); pass 'none' on any other network"
    )

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    rviz = OpaqueFunction(function=launch_rviz)

    image_uncompress_node = Node(
        package="x3_bringup",
        executable="image_republisher",
        name="image_republisher",
        namespace=agent_name
    )

    return LaunchDescription([
        agent_name_arg,
        fastdds_profile_arg,
        rviz,
        # image_uncompress_node
    ])