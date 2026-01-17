from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'g1_sim_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install model files
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Bridge between MuJoCo simulation and ROS 2 for Unitree G1',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_ros2_bridge = g1_sim_bridge.mujoco_ros2_bridge:main',
            'joint_command_publisher = g1_sim_bridge.joint_command_publisher:main',
            'keyboard_teleop = g1_sim_bridge.keyboard_teleop:main',
            'balance_controller = g1_sim_bridge.balance_controller:main',
            'rl_locomotion = g1_sim_bridge.rl_locomotion:main',
        ],
    },
)
