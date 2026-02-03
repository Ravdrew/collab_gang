"""
Simulation Launch File for TidyBot2.

Launches:
- MuJoCo bridge (physics simulation)
- Robot state publisher (URDF + TF)
- Arm controllers (left + right)
- RViz (optional)

Usage:
    ros2 launch tidybot_bringup sim.launch.py
    ros2 launch tidybot_bringup sim.launch.py use_rviz:=false
    ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=false
    ros2 launch tidybot_bringup sim.launch.py scene:=scene_pickup.xml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function that runs after arguments are resolved."""
    # Get resolved argument values
    scene = LaunchConfiguration('scene').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    show_mujoco_viewer = LaunchConfiguration('show_mujoco_viewer')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motion_planner = LaunchConfiguration('use_motion_planner')

    # Package paths
    pkg_bringup = FindPackageShare('tidybot_bringup')
    pkg_description = FindPackageShare('tidybot_description')

    # Get model path from TIDYBOT_REPO_ROOT environment variable
    repo_root = os.environ.get('TIDYBOT_REPO_ROOT', '')
    if not repo_root:
        repo_root = os.path.expanduser('~/Documents/collaborative-robotics-2026')

    # Construct model path using scene argument
    model_path = os.path.join(repo_root, 'simulation', 'assets', 'mujoco', scene)
    ik_model_path = os.path.join(repo_root, 'simulation', 'assets', 'mujoco', 'tidybot_wx250s_bimanual.xml')

    # URDF from xacro
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx250s.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # MuJoCo Bridge
    mujoco_bridge = Node(
        package='tidybot_mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'sim_rate': 500.0,
            'publish_rate': 100.0,
            'camera_rate': 30.0,
            'show_viewer': show_mujoco_viewer,
        }]
    )

    # Right arm controller
    right_arm_controller = Node(
        package='tidybot_control',
        executable='arm_controller_node',
        name='right_arm_controller',
        output='screen',
        parameters=[{'arm_name': 'right', 'control_rate': 50.0}]
    )

    # Left arm controller
    left_arm_controller = Node(
        package='tidybot_control',
        executable='arm_controller_node',
        name='left_arm_controller',
        output='screen',
        parameters=[{'arm_name': 'left', 'control_rate': 50.0}]
    )

    # Motion planner
    motion_planner = Node(
        package='tidybot_ik',
        executable='motion_planner_node',
        name='motion_planner',
        output='screen',
        condition=IfCondition(use_motion_planner),
        parameters=[{
            'model_path': ik_model_path,
            'ik_dt': 0.002,
            'ik_max_iterations': 500,
            'position_tolerance': 0.01,
            'orientation_tolerance': 0.1,
            'min_collision_distance': 0.05,
        }]
    )

    # RViz
    rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'tidybot.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        robot_state_publisher,
        mujoco_bridge,
        right_arm_controller,
        left_arm_controller,
        motion_planner,
        rviz,
    ]


def generate_launch_description():
    # Declare arguments
    declare_scene = DeclareLaunchArgument(
        'scene', default_value='scene_wx250s_bimanual.xml',
        description='MuJoCo scene file (e.g., scene_pickup.xml, scene_wx250s_bimanual.xml)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization'
    )

    declare_show_viewer = DeclareLaunchArgument(
        'show_mujoco_viewer', default_value='true',
        description='Show MuJoCo viewer window'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    declare_use_planner = DeclareLaunchArgument(
        'use_motion_planner', default_value='true',
        description='Launch motion planner for IK and trajectory planning'
    )

    return LaunchDescription([
        # Arguments
        declare_scene,
        declare_use_rviz,
        declare_show_viewer,
        declare_use_sim_time,
        declare_use_planner,
        # Nodes via OpaqueFunction (resolved after arguments)
        OpaqueFunction(function=launch_setup),
    ])
