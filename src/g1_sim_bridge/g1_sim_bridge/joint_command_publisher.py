#!/usr/bin/env python3
"""
Joint Command Publisher for Unitree G1.

Publishes predefined joint commands or sinusoidal motions for testing.

Author: Generated for Unitree G1 Simulation
License: BSD-3-Clause
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointCommandPublisher(Node):
    """Node that publishes joint commands to control the G1 robot."""

    JOINT_NAMES = [
        # Waist (3 DOF)
        'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
        # Left Arm (7 DOF)
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
        'left_elbow_joint', 'left_wrist_yaw_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint',
        # Right Arm (7 DOF)
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
        'right_elbow_joint', 'right_wrist_yaw_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint',
        # Left Leg (6 DOF)
        'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        # Right Leg (6 DOF)
        'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    ]

    # Standing pose with bent knees for balance
    STANDING_POSE = {
        # Waist
        'waist_yaw_joint': 0.0,
        'waist_roll_joint': 0.0,
        'waist_pitch_joint': 0.0,
        # Left Arm (relaxed at sides)
        'left_shoulder_pitch_joint': 0.0,
        'left_shoulder_roll_joint': 0.2,
        'left_shoulder_yaw_joint': 0.0,
        'left_elbow_joint': -0.3,
        'left_wrist_yaw_joint': 0.0,
        'left_wrist_roll_joint': 0.0,
        'left_wrist_pitch_joint': 0.0,
        # Right Arm (relaxed at sides)
        'right_shoulder_pitch_joint': 0.0,
        'right_shoulder_roll_joint': -0.2,
        'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.3,
        'right_wrist_yaw_joint': 0.0,
        'right_wrist_roll_joint': 0.0,
        'right_wrist_pitch_joint': 0.0,
        # Left Leg (bent knee stance)
        'left_hip_yaw_joint': 0.0,
        'left_hip_roll_joint': 0.0,
        'left_hip_pitch_joint': -0.4,
        'left_knee_joint': 0.8,
        'left_ankle_pitch_joint': -0.4,
        'left_ankle_roll_joint': 0.0,
        # Right Leg (bent knee stance)
        'right_hip_yaw_joint': 0.0,
        'right_hip_roll_joint': 0.0,
        'right_hip_pitch_joint': -0.4,
        'right_knee_joint': 0.8,
        'right_ankle_pitch_joint': -0.4,
        'right_ankle_roll_joint': 0.0,
    }

    def __init__(self):
        super().__init__('joint_command_publisher')

        # Declare parameters
        self.declare_parameter('mode', 'standing')  # 'standing', 'wave', 'squat'
        self.declare_parameter('rate', 50.0)  # Hz

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_command)

        # Time tracking
        self.start_time = self.get_clock().now()

        self.get_logger().info(f'Joint Command Publisher started in {self.mode} mode')

    def get_time(self):
        """Get elapsed time in seconds."""
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def publish_command(self):
        """Publish joint command based on current mode."""
        msg = Float64MultiArray()

        if self.mode == 'standing':
            msg.data = [self.STANDING_POSE[name] for name in self.JOINT_NAMES]

        elif self.mode == 'wave':
            # Wave the left arm
            t = self.get_time()
            positions = dict(self.STANDING_POSE)

            # Raise left arm and wave
            positions['left_shoulder_pitch_joint'] = -1.5  # Arm up
            positions['left_shoulder_roll_joint'] = 0.3
            positions['left_elbow_joint'] = -0.5 + 0.3 * math.sin(2 * math.pi * 0.5 * t)  # Wave

            msg.data = [positions[name] for name in self.JOINT_NAMES]

        elif self.mode == 'squat':
            # Simple squat motion
            t = self.get_time()
            positions = dict(self.STANDING_POSE)

            # Squat depth oscillation
            squat_depth = 0.2 * (1 - math.cos(2 * math.pi * 0.25 * t))

            positions['left_hip_pitch_joint'] = -0.3 - squat_depth
            positions['left_knee_joint'] = 0.6 + 2 * squat_depth
            positions['left_ankle_pitch_joint'] = -0.3 - squat_depth

            positions['right_hip_pitch_joint'] = -0.3 - squat_depth
            positions['right_knee_joint'] = 0.6 + 2 * squat_depth
            positions['right_ankle_pitch_joint'] = -0.3 - squat_depth

            msg.data = [positions[name] for name in self.JOINT_NAMES]

        elif self.mode == 'arms_up':
            # Raise both arms
            positions = dict(self.STANDING_POSE)

            positions['left_shoulder_pitch_joint'] = -2.5
            positions['left_shoulder_roll_joint'] = 0.0
            positions['left_elbow_joint'] = 0.0

            positions['right_shoulder_pitch_joint'] = -2.5
            positions['right_shoulder_roll_joint'] = 0.0
            positions['right_elbow_joint'] = 0.0

            msg.data = [positions[name] for name in self.JOINT_NAMES]

        else:
            # Default to standing
            msg.data = [self.STANDING_POSE[name] for name in self.JOINT_NAMES]

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
