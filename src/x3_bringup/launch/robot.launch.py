from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os, tempfile
import math

def generate_launch_description():
    '''
    Function to launch the following processes/hardware on the robot
      1. LiDAR
      2. Camera - using the orbbec camera package
      3. Low-level interface (IMU, wheel encoder, wheel motors), handled by mcnamu_driver
      4. Odometry components - handled by a separate odom.launch.py file'''
    
    # ===== DEFINE REQUIRED PATHS =====
    description_pkg_dir = get_package_share_directory("x3_description")
    model_dir = os.path.join(description_pkg_dir, "urdf", "x3.urdf.xacro")

    camera_pkg_dir = get_package_share_directory("astra_camera")
    camera_launch_path = os.path.join(camera_pkg_dir, "launch", "astra_pro_plus.launch.xml")

    bringup_pkg_dir = get_package_share_directory("x3_bringup")
    odom_launch_path = os.path.join(bringup_pkg_dir, "launch", "odom.launch.py")
    controllers_template_path = os.path.join(bringup_pkg_dir, "config", "controllers.yaml.template")

    # ===== DECLARE LAUNCH ARGUMENTS =====
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent0",
        description = "Namespace of the launching agent"
    )

    model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=model_dir,
        description="Absolute path to robot URDF/xacro file"
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name namespace'
    )

    is_gazebo_arg = DeclareLaunchArgument(
        name="is_gazebo",
        default_value="false",
        description="Whether to load Gazebo-specific plugins/properties"
    )

    policy_model_arg = DeclareLaunchArgument(
        "policy_model",
        default_value="TD3_00386_1000",
        description="Policy folder the DRL policy node loads (x3_drl_policy/policies)"
    )

    start_policy_arg = DeclareLaunchArgument(
        "start_policy",
        default_value="true",
        description="Start the DRL policy node with this launch; false lets scripts/sim2real/run_robot_schedule.py manage it"
    )

    odom_profile_arg = DeclareLaunchArgument(
        "odom_profile",
        default_value="",
        description="EKF and covariance parameter files of the odometry: '' (main) or _robot_20261029 (config/)"
    )

    enable_face_emotion_arg = DeclareLaunchArgument(
        "enable_face_emotion",
        default_value="true",
        description="Start the face detection and emotion recognition nodes (false saves battery for pure navigation runs)"
    )

    robot_model = LaunchConfiguration("robot_model")
    camera_name = LaunchConfiguration("orbbec_cameracamera_name")


    robot_description = ParameterValue(
        Command(["xacro ", robot_model,
                " agent_name:=", agent_name,
                " use_ros_control:=false",
                " is_gazebo:=", LaunchConfiguration("is_gazebo")]),
        value_type = str
    )

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    # robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=agent_name,
        parameters=[{"robot_description": robot_description}],
    )

    # camera launch file
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': agent_name,
            'depth_registration': 'true',
            'color_depth_synchronization': 'true',
            'enable_d2c_viewer': 'true',
        }.items()
    )

    # face detection node
    face_detection_node = Node(
        package="x3_visual",
        executable="face_detection_node",
        namespace=agent_name,
        arguments = [
            "--ros-args", "--log-level", 
            PythonExpression(["'", agent_name, ".face_detection_node:=error'"])
        ],
        condition=IfCondition(LaunchConfiguration("enable_face_emotion")),
    )

    emotion_recognition_node = Node(
        package = "x3_visual",
        executable="emotion_recognition_node",
        namespace=agent_name,
        condition=IfCondition(LaunchConfiguration("enable_face_emotion")),
    )

    # image transport republisher
    # subscribes to: <agent_name>/color/image_raw
    # publishes to: <agent_name>/color/image_raw/compressed
    image_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='color_image_republisher',
        namespace=agent_name,
        arguments=['raw', 'compressed'],
        remappings=[
            ('in',  ['color/image_raw']),
            ('out/compressed', ['color/image_raw/compressed']),
        ],
        parameters=[{
            # JPEG quality 0-100: lower = smaller packets, higher = better image quality.
            'compressed.jpeg_quality': 60,
            'compressed.format': 'jpeg',
        }],
    )

    # lidar launch file
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        namespace=agent_name,
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'scan_frequency': 10.0,
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 1000000,
            'frame_id': PythonExpression(["'", agent_name, "_lidar_link'"]),
            'inverted': False,
            'flip_x_axis': True,
            'angle_compensate': True,
            'scan_mode': 'Standard',      # the mode of the static scan the policy's rear wedge was set from
        }]
    )

    lidar_rear_mask_node = Node(
        package='x3_bringup',
        executable='lidar_rear_mask',
        name='lidar_filter',
        namespace=agent_name,
        parameters=[{
            'input_topic': 'scan',
            'output_topic': 'scan_filtered',
            'mask_center_angle': math.pi,
            'mask_half_width': 0.90,     # rad; self returns span -29.1..+36.9 deg, rf2o only
        }]
    )

    # Launch the odometry nodes
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([odom_launch_path]),
        launch_arguments={
            "rf2o_scan_topic": "scan_filtered",
            "odom_profile": LaunchConfiguration("odom_profile"),
        }.items()
    )

    # Low level driver node - IMU, wheel encoder, and wheel motors
    driver_node = Node(
        package='x3_bringup',
        executable='mcnamu_driver',
        namespace=agent_name,
        parameters=[{
            'Prefix': agent_name,
        }]
    )  

    # DRL policy node with the frozen hardware config: parameters live in drl_policy.launch.py
    drl_ctrl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(bringup_pkg_dir, "launch", "drl_policy.launch.py")]),
        launch_arguments={"agent_name": agent_name, "model_name": LaunchConfiguration("policy_model")}.items(),
        condition=IfCondition(LaunchConfiguration("start_policy"))
    )

    return LaunchDescription([
        agent_name_arg,
        model_arg,
        policy_model_arg,
        start_policy_arg,
        enable_face_emotion_arg,
        odom_profile_arg,
        camera_name_arg,
        is_gazebo_arg,
        robot_state_publisher_node,
        lidar_node,
        lidar_rear_mask_node,
        camera_launch,
        image_republisher_node,
        face_detection_node,
        emotion_recognition_node,
        odom_launch,

        driver_node,
        drl_ctrl_node
    ])
