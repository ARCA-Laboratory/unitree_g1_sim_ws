"""
Launch file for G1 simulation with balance controller.

This launches:
- Full G1 simulation with floating base (fixed_base:=false)
- Balance controller node that tries to keep the robot upright
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
    kp_pitch = LaunchConfiguration('kp_pitch')
    kd_pitch = LaunchConfiguration('kd_pitch')
    kp_roll = LaunchConfiguration('kp_roll')
    kd_roll = LaunchConfiguration('kd_roll')

    # Include main simulation launch with floating base
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(g1_sim_bringup_dir, 'launch', 'g1_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': 'true',
            'use_viewer': 'true',
            'fixed_base': 'false',  # Floating base for balance demo
        }.items()
    )

    # Balance controller (delayed start to let simulation initialize)
    balance_node = TimerAction(
        period=2.0,  # Wait 2 seconds
        actions=[
            Node(
                package='g1_sim_bridge',
                executable='balance_controller',
                name='balance_controller',
                output='screen',
                parameters=[{
                    'rate': 100.0,
                    'kp_pitch': kp_pitch,
                    'kd_pitch': kd_pitch,
                    'kp_roll': kp_roll,
                    'kd_roll': kd_roll,
                    'max_ankle_adjust': 0.3,
                    'max_hip_adjust': 0.2,
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'kp_pitch', default_value='3.0',
            description='Proportional gain for pitch balance'
        ),
        DeclareLaunchArgument(
            'kd_pitch', default_value='0.8',
            description='Derivative gain for pitch balance'
        ),
        DeclareLaunchArgument(
            'kp_roll', default_value='3.0',
            description='Proportional gain for roll balance'
        ),
        DeclareLaunchArgument(
            'kd_roll', default_value='0.8',
            description='Derivative gain for roll balance'
        ),
        sim_launch,
        balance_node,
    ])
