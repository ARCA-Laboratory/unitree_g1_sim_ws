"""
Launch file for G1 RL-based walking with visualization.

Launches:
- Robot state publisher (for URDF)
- RL locomotion controller (MuJoCo + policy)
- RViz2 for visualization

Note: Keyboard teleop runs separately via run_sim.sh wrapper.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    g1_description_dir = get_package_share_directory('g1_description')

    # URDF file
    urdf_file = os.path.join(g1_description_dir, 'urdf', 'g1.urdf.xacro')

    # RViz config
    rviz_config = os.path.join(g1_description_dir, 'rviz', 'g1_display.rviz')

    return LaunchDescription([
        # Robot State Publisher (provides URDF for RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
                'use_sim_time': False
            }]
        ),

        # RL Locomotion Controller (runs MuJoCo and publishes joint states/TF)
        Node(
            package='g1_sim_bridge',
            executable='rl_locomotion',
            name='rl_locomotion',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
