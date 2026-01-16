"""
Launch file for G1 simulation with keyboard teleoperation.

This launches:
- Full G1 simulation
- Keyboard teleoperation node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    g1_sim_bringup_dir = get_package_share_directory('g1_sim_bringup')

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

    # Keyboard teleop node (run separately in terminal for keyboard input)
    # Note: This is just for reference - run it separately with:
    # ros2 run g1_sim_bridge keyboard_teleop.py

    return LaunchDescription([
        sim_launch,
    ])
