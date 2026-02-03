"""
Connect to Robot Launch File for TidyBot2 Clients.

Sets up the ROS2 environment for connecting to a remote TidyBot2 robot.
This launch file primarily configures environment variables and can optionally
start a local robot state publisher for TF visualization.

Usage:
    # Basic connection (just sets environment)
    ros2 launch tidybot_client connect_robot.launch.py robot_ip:=192.168.1.100

    # With local TF publisher (for RViz)
    ros2 launch tidybot_client connect_robot.launch.py robot_ip:=192.168.1.100 start_tf:=true

After launching, you can:
    ros2 topic list              # See robot topics
    ros2 topic echo /joint_states  # See joint states
    ros2 topic pub /cmd_vel ...  # Send commands

Environment Variables Set:
    ROS_DOMAIN_ID - Must match robot (default 42)
    RMW_IMPLEMENTATION - DDS implementation
    CYCLONEDDS_URI - DDS configuration file (if using Cyclone)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_client = FindPackageShare('tidybot_client')
    pkg_description = FindPackageShare('tidybot_description')

    # Declare arguments
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip', default_value='',
        description='Robot mini PC IP address (required for FastDDS discovery server)'
    )

    declare_domain_id = DeclareLaunchArgument(
        'domain_id', default_value='42',
        description='ROS domain ID (must match robot)'
    )

    declare_dds = DeclareLaunchArgument(
        'dds', default_value='cyclone',
        description='DDS implementation: cyclone or fastdds'
    )

    declare_start_tf = DeclareLaunchArgument(
        'start_tf', default_value='false',
        description='Start local robot state publisher for TF'
    )

    # Set ROS_DOMAIN_ID
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=LaunchConfiguration('domain_id')
    )

    # Log connection info
    log_info = LogInfo(
        msg=['Connecting to TidyBot2 robot with ROS_DOMAIN_ID=', LaunchConfiguration('domain_id')]
    )

    # Optional: Local robot state publisher for TF
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx200.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_local',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tf')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        declare_robot_ip,
        declare_domain_id,
        declare_dds,
        declare_start_tf,
        set_domain_id,
        log_info,
        robot_state_publisher,
    ])
