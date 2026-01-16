"""
Launch file for MuJoCo-ROS2 bridge only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_viewer',
            default_value='true',
            description='Launch MuJoCo viewer'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Path to custom MuJoCo XML model'
        ),

        Node(
            package='g1_sim_bridge',
            executable='mujoco_ros2_bridge.py',
            name='g1_mujoco_simulator',
            output='screen',
            parameters=[{
                'use_viewer': LaunchConfiguration('use_viewer'),
                'model_path': LaunchConfiguration('model_path'),
                'sim_rate': 500.0,
                'publish_rate': 100.0,
            }]
        ),
    ])
