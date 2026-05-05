from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os

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

    # ===== DECLARE LAUNCH ARGUMENTS =====
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

    robot_model = LaunchConfiguration("robot_model")
    camera_name = LaunchConfiguration("camera_name")

    robot_description = ParameterValue(
        Command(["xacro ", robot_model]),
        value_type = str
    )

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    # robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher", 
    )

    # camera launch file
    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': camera_name,
        }.items()
    )

    # image transport republisher
    # subscribes to: /<camera_name>/color/image_raw
    # publishes to: /<camera_color>/image_raw/compressed
    image_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='color_image_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in',  [camera_name, '/color/image_raw']),
            ('out/compressed', [camera_name, '/color/image_raw/compressed']),
        ],
        parameters=[{
            # JPEG quality 0-100: lower = smaller packets, higher = better image quality.
            'compressed.jpeg_quality': 80,
            'compressed.format': 'jpeg',
        }],
    )

    # lidar launch file
    lidar_launch = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 1000000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
        }]
    )

    lidar_tf_rotate = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_transform',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'laser']
        #                                      ^ yaw 180°
    )

    # Low level driver node - IMU, wheel encoder, and wheel motors
    driver_node = Node(
        package='x3_bringup',
        executable='mcnamu_driver',
    )

    return LaunchDescription([
        model_arg,
        camera_name_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        camera_launch,
        image_republisher_node,
        lidar_launch,
        lidar_tf_rotate,
        driver_node,
    ])