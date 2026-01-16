"""
Launch file for G1 simulation with demo motions.

This launches:
- Full G1 simulation
- Joint command publisher with predefined motions
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    g1_sim_bringup_dir = get_package_share_directory('g1_sim_bringup')

    # Launch arguments
    mode = LaunchConfiguration('mode')

    # Include main simulation launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(g1_sim_bringup_dir, 'launch', 'g1_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'true',
            'use_viewer': 'true',
        }.items()
    )

    # Joint command publisher (delayed start to let simulation initialize)
    joint_cmd_node = TimerAction(
        period=2.0,  # Wait 2 seconds
        actions=[
            Node(
                package='g1_sim_bridge',
                executable='joint_command_publisher',
                name='joint_command_publisher',
                output='screen',
                parameters=[{
                    'mode': mode,
                    'rate': 50.0,
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='wave',
            description='Demo mode: standing, wave, squat, arms_up'
        ),
        sim_launch,
        joint_cmd_node,
    ])
