"""
Main launch file for Unitree G1 MuJoCo simulation with ROS 2.

This launches:
- MuJoCo simulation with ROS 2 bridge
- Robot state publisher
- RViz2 for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    g1_description_dir = get_package_share_directory('g1_description')
    g1_sim_bridge_dir = get_package_share_directory('g1_sim_bridge')

    # URDF file
    urdf_file = os.path.join(g1_description_dir, 'urdf', 'g1.urdf.xacro')

    # RViz config
    rviz_config = os.path.join(g1_description_dir, 'rviz', 'g1_display.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_viewer = LaunchConfiguration('use_viewer')
    headless = LaunchConfiguration('headless')
    fixed_base = LaunchConfiguration('fixed_base')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (requires /clock topic)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'use_viewer',
            default_value='true',
            description='Launch MuJoCo viewer window'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run in headless mode (no viewer, no RViz)'
        ),
        DeclareLaunchArgument(
            'fixed_base',
            default_value='true',
            description='Pin robot base (true=stable, false=floating/requires balance controller)'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
                'use_sim_time': use_sim_time
            }]
        ),

        # MuJoCo Simulation Bridge
        Node(
            package='g1_sim_bridge',
            executable='mujoco_ros2_bridge',
            name='g1_mujoco_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_viewer': use_viewer,
                'fixed_base': fixed_base,
                'sim_rate': 500.0,
                'publish_rate': 100.0,
            }]
        ),

        # RViz2 (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '", headless, "' != 'true'"
                ])
            )
        ),
    ])
