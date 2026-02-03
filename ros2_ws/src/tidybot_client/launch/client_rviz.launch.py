"""
Client-side RViz Launch File for TidyBot2.

Launch RViz on a remote client machine to visualize the robot state
received over the network from the TidyBot2 mini PC.

Prerequisites:
1. Robot must be running robot.launch.py on mini PC
2. Client must have same ROS_DOMAIN_ID as robot
3. Network must allow ROS2 DDS discovery (multicast or discovery server)

Usage:
    # Set up environment first
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    # Launch RViz
    ros2 launch tidybot_client client_rviz.launch.py

    # Or with custom domain ID
    ros2 launch tidybot_client client_rviz.launch.py domain_id:=99
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_client = FindPackageShare('tidybot_client')
    pkg_description = FindPackageShare('tidybot_description')
    pkg_bringup = FindPackageShare('tidybot_bringup')

    # Declare arguments
    declare_domain_id = DeclareLaunchArgument(
        'domain_id', default_value='42',
        description='ROS domain ID (must match robot)'
    )

    # Set ROS_DOMAIN_ID environment variable
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=LaunchConfiguration('domain_id')
    )

    # URDF for robot model visualization
    # Note: We load the URDF locally so RViz can display the robot model
    # without requiring the full description to be transmitted over network
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx200.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    # Robot state publisher (local, for RViz visualization only)
    # This reads /joint_states from robot over network and publishes TF locally
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_local',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # RViz
    rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'tidybot.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        declare_domain_id,
        set_domain_id,
        robot_state_publisher,
        rviz,
    ])
