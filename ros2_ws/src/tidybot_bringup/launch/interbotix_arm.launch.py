"""
Interbotix WX250s Arm Launch File for TidyBot2.

This launch file uses the official Interbotix xs_sdk node to control arms
and pan-tilt on separate U2D2 hubs.

Usage:
    # Right arm + pan-tilt only (single U2D2 on /dev/ttyUSB0):
    ros2 launch tidybot_bringup interbotix_arm.launch.py use_left_arm:=false

    # Both arms + pan-tilt (dual U2D2 setup):
    ros2 launch tidybot_bringup interbotix_arm.launch.py

Hardware Setup (Dual U2D2):
    - U2D2 #1 (/dev/ttyUSB0): Right arm (IDs 1-9) + Pan-tilt (IDs 21-22)
    - U2D2 #2 (/dev/ttyUSB1): Left arm (IDs 11-19)

Prerequisites:
    - Interbotix SDK installed (source ~/interbotix_humble_ws/install/setup.bash)
    - U2D2 adapters connected
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_bringup = FindPackageShare('tidybot_bringup')

    use_left_arm = LaunchConfiguration('use_left_arm').perform(context) == 'true'
    use_pan_tilt = LaunchConfiguration('use_pan_tilt').perform(context) == 'true'
    load_configs = LaunchConfiguration('load_configs').perform(context) == 'true'

    nodes = []

    # Right arm + pan-tilt on U2D2 #1 (/dev/ttyUSB0)
    if use_pan_tilt:
        right_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm_pantilt.yaml'])
        right_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm_pantilt_modes.yaml'])
    else:
        right_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm.yaml'])
        right_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'modes.yaml'])

    right_xs_sdk = Node(
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace='right_arm',
        parameters=[{
            'motor_configs': right_motor_config,
            'mode_configs': right_mode_config,
            'load_configs': load_configs,
        }],
        output='screen',
    )
    nodes.append(right_xs_sdk)

    # Left arm on U2D2 #2 (/dev/ttyUSB1) - separate xs_sdk node
    if use_left_arm:
        left_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'left_arm.yaml'])
        left_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'left_arm_modes.yaml'])

        left_xs_sdk = Node(
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='xs_sdk',
            namespace='left_arm',
            parameters=[{
                'motor_configs': left_motor_config,
                'mode_configs': left_mode_config,
                'load_configs': load_configs,
            }],
            output='screen',
        )
        nodes.append(left_xs_sdk)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_left_arm',
            default_value='true',
            description='Also control left arm on U2D2 #2 (/dev/ttyUSB1)'
        ),
        DeclareLaunchArgument(
            'use_pan_tilt',
            default_value='true',
            description='Also control pan-tilt on U2D2 #1 (with right arm)'
        ),
        DeclareLaunchArgument(
            'load_configs',
            default_value='true',
            description='Load motor configs from YAML'
        ),
        OpaqueFunction(function=launch_setup),
    ])
