import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory

# what does this file launch?
# - robot_state_publisher
# - gazebo
# - spawner
# - ros_gz_bridge
# - diff_drive_spawner
# - joint_broadcaster_spawner
# - laser_scan_matcher
# - IMU filter node
# - extended kalman filter

def generate_launch_description():
    # ===== SET REQUIRED PATH =====
    pkg_path = get_package_share_directory("x3_description")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "x3.urdf.xacro"])

    # path to launch files
    gazebo_launch_path = PathJoinSubstitution([pkg_path, "launch", "launch_world.py"])
    odom_launch_path = PathJoinSubstitution([pkg_path, "launch", "launch_odom.py"])

    # path to ROS-Gazebo bridges
    bridge_path_1 = os.path.join(pkg_path, "config", "gz_bridge_ros_control.yaml")
    bridge_path_2 = os.path.join(pkg_path, "config", "gz_bridge_gazebo_control.yaml")
    controllers_template_path = os.path.join(pkg_path, "config", "controllers.yaml.template")

    # ===== DEFINE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true",
        description = "Whether or not to use sim time, defaulting to false."
    )

    use_ros_control = LaunchConfiguration("use_ros_control")
    use_ros_control_arg = DeclareLaunchArgument(
        "use_ros_control",
        default_value = "true",
        description = "If true, control the agent using ros2_control, otherwise, use Gazebo plugins. Defaults to false"
    )

    use_odom_publisher = LaunchConfiguration("use_odom_publisher")
    use_odom_publisher_arg = DeclareLaunchArgument(
        "use_odom_publisher",
        default_value = "true",
        description = "True means we are using Gazebo OdometryPublisher plugin, " \
        "which needs proper offset to match agent spawn offset"
    )

    world = PathJoinSubstitution([pkg_path, "worlds", LaunchConfiguration("world")])
    world_arg = DeclareLaunchArgument(
        "world",
        default_value = "world_1.sdf",
        description = "Name of the world to be loaded, defaulting to empty_world.sdf"
    )

    gazebo_config = PathJoinSubstitution([pkg_path, "config", LaunchConfiguration("gazebo_config")])
    gazebo_config_arg = DeclareLaunchArgument(
        "gazebo_config",
        default_value = "gazebo.config",
        description = "Name of the .config file for Gazebo configurations"
    )

    x_offset_arg = DeclareLaunchArgument(
        "x_offset",
        default_value = "-3.0",
        description = "The robot x position at spawn"
    )

    y_offset_arg = DeclareLaunchArgument(
        "y_offset",
        default_value = "-3.0",
        description = "The robot y position at spawn"
    )

    def launch_setup(context, *args, **kwargs):
        agent_name_str = LaunchConfiguration("agent_name").perform(context)
        x_offset = LaunchConfiguration("x_offset").perform(context)
        y_offset = LaunchConfiguration("y_offset").perform(context)

        # Render controllers.yaml template and populate with agent_name
        with open(controllers_template_path, "r") as f:
            rendered = f.read().replace("__AGENT_NAME__", agent_name_str)
        rendered_controllers_path = os.path.join(tempfile.gettempdir(), f"{agent_name_str}_controllers.yaml")
        with open(rendered_controllers_path, "w") as f:
            f.write(rendered)

        print(f"Rendered ros2_control YAML file at {rendered_controllers_path}")

        # set the required parameters:
        robot_description = Command(["xacro ", xacro_path, 
                                     " agent_name:=", agent_name, 
                                     " use_ros_control:=", use_ros_control,
                                     " controllers_yaml:=", rendered_controllers_path])

        # ===== NODE DEFINITION =====
        rsp = Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            name = "robot_state_publisher",
            namespace = agent_name_str, 
            parameters = [
                {"robot_description": ParameterValue(robot_description, value_type = str), 
                 "use_sim_time" : use_sim_time},
                # {"qos_overrides": {
                #     f"/{agent_name_str}/joint_states": {
                #         "subscription": {
                #             "reliability": "reliable",
                #             "durability": "volatile"
                #         }
                #     }
                # }}
            ]
        )

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_path]),
            launch_arguments = {"world" : world, "config": gazebo_config}.items()
        )

        agent_spawner = Node(
            package = "ros_gz_sim",
            executable = "create",
            namespace = agent_name, 
            arguments = [
                        "-topic", "robot_description",
                        "-name", agent_name,
                        "-x", x_offset,
                        "-y", y_offset,
                        "-z", "0.0"],
            output = "screen"
        )

        odom_offset_node = Node(
            package = "x3_description",
            executable = "odom_offset_node.py",
            name = "odom_offset",
            namespace = agent_name_str,
            parameters = [{
                "agent_name": agent_name_str,
                "x_offset": float(x_offset),
                "y_offset": float(y_offset),
                "use_sim_time": use_sim_time,
            }],
            condition = IfCondition(use_odom_publisher)
        )

        static_tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{agent_name_str}_world_to_odom_tf',
            arguments=[
                '--x', PythonExpression(['1.0 * ', x_offset]),
                '--y', PythonExpression(['1.0 * ', y_offset]),
                '--frame-id', 'world',
                '--child-frame-id', f'{agent_name_str}_odom',
            ],
            parameters=[{'use_sim_time': True}],
        )

        ros_gz_bridge_ros_control = Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            name = "ros_gz_bridge_ros_control",
            namespace = agent_name,
            arguments = ["--ros-args", "-p", f"config_file:={bridge_path_1}"],
            condition = IfCondition(use_ros_control)
        )

        ros_gz_bridge_gazebo_control = Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            name = "ros_gz_bridge_gazebo_control",
            namespace = agent_name,
            arguments = ["--ros-args", "-p", f"config_file:={bridge_path_2}"],
            condition = UnlessCondition(use_ros_control)
        )

        diff_drive_spawner = Node(
            package = "controller_manager",
            executable = "spawner",
            namespace = agent_name,
            arguments = ["diff_controller"],
            condition = IfCondition(use_ros_control)
        )

        joint_broadcaster_spawner = Node(
            package = "controller_manager",
            executable = "spawner",
            namespace = agent_name,
            arguments = ["joint_broad"],
            condition = IfCondition(use_ros_control)
        )

        # node to map from diff drive controller wheel odom (nav_msgs/odometry)
        # to vel_raw (geometry_msgs/TwistStamped)
        odom_to_vel_raw = Node(
            package = "x3_description",
            executable = "odom_to_vel_raw_node.py",
            name = "odom_to_vel_raw",
            namespace = agent_name,
            parameters = [{"agent_name": agent_name}],
            condition = IfCondition(use_ros_control)
        )

        odom_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([odom_launch_path]),
            launch_arguments = {"agent_name": agent_name,
                                "use_sim_time" : use_sim_time}.items(),
            condition = IfCondition(use_ros_control)
        )

        return [
            rsp, 
            gazebo,
            agent_spawner,
            # static_tf_publisher,
            odom_offset_node,
            ros_gz_bridge_gazebo_control, 
            ros_gz_bridge_ros_control,
            diff_drive_spawner,
            joint_broadcaster_spawner,
            odom_to_vel_raw,
            odom_nodes
        ]

    return LaunchDescription([
        agent_name_arg,
        use_sim_time_arg,
        use_ros_control_arg,
        use_odom_publisher_arg,
        world_arg,
        gazebo_config_arg,
        x_offset_arg,
        y_offset_arg,
        OpaqueFunction(function=launch_setup)
    ])
    