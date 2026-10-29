from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    '''
    DRL policy node with the frozen hardware config of the confirmatory robot runs
    (notes/seed_selection_protocol_20261028.md section 3, docs/sim2real_deployment_config.md).
    Single source of these parameters: robot.launch.py includes this file, and the robot-side runner
    (scripts/sim2real/run_robot_schedule.py) launches it alone to switch the policy between runs.

      ros2 launch x3_bringup drl_policy.launch.py model_name:=TD3_00390_1000
    '''
    agent_name = LaunchConfiguration('agent_name')
    model_name = LaunchConfiguration('model_name')

    agent_name_arg = DeclareLaunchArgument(
        'agent_name', default_value='agent0',
        description='Namespace of the launching agent')
    model_name_arg = DeclareLaunchArgument(
        'model_name', default_value='TD3_00386_1000',
        description='Policy folder under x3_drl_policy/policies (delta ids 384 and 386-395, direct otherwise)')

    drl_ctrl_node = Node(
        package='x3_drl_policy',
        executable='policy_node',
        namespace=agent_name,
        parameters=[{
            'agent_name':       agent_name,
            'model_name':       model_name,
            'goal_tolerance':   0.75,
            'obstacle_tolerance': 0.205,
            # 0.2 m/s and 0.3 rad/s reproduce the per-step motion of the 500 Hz, 2 m/s, 3 rad/s training at 50 Hz
            'normalized_actions': True,
            'max_lin_vel':      0.2,
            'max_angular_vel':  0.3,
            'goal_timeout':     90.0,
            'antennae_mask_dist': 0.0,          # no direction-blind short-range mask; the rear wedge is masked by angle
            'rear_mask_deg':    40.0,           # static scan 2026-10-28 (scripts/sim2real/static_scan_20261028.json): self returns span -29.1..+36.9 deg behind the sensor, +3 deg margin
            'rear_mask_fill':   'train_mean',
            'range_clip':       11.10,          # observation bound of the training environment
        }]
    )

    return LaunchDescription([
        agent_name_arg,
        model_name_arg,
        drl_ctrl_node,
    ])
