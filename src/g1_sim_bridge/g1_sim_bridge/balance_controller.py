#!/usr/bin/env python3
"""
Simple COM-based Balance Controller for Unitree G1.

This implements a basic center-of-mass (COM) balance controller that adjusts
ankle and hip joints to keep the robot's COM over its support polygon.

This is a simplified controller for demonstration - real humanoid balancing
requires more sophisticated approaches (whole-body control, MPC, or RL).

Author: Generated for Unitree G1 Simulation
License: BSD-3-Clause
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3


class BalanceController(Node):
    """Simple COM-based balance controller for G1 humanoid."""

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

    # Default standing pose
    STANDING_POSE = {
        'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0, 'waist_pitch_joint': 0.0,
        'left_shoulder_pitch_joint': 0.0, 'left_shoulder_roll_joint': 0.2, 'left_shoulder_yaw_joint': 0.0,
        'left_elbow_joint': -0.3, 'left_wrist_yaw_joint': 0.0, 'left_wrist_roll_joint': 0.0, 'left_wrist_pitch_joint': 0.0,
        'right_shoulder_pitch_joint': 0.0, 'right_shoulder_roll_joint': -0.2, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.3, 'right_wrist_yaw_joint': 0.0, 'right_wrist_roll_joint': 0.0, 'right_wrist_pitch_joint': 0.0,
        'left_hip_yaw_joint': 0.0, 'left_hip_roll_joint': 0.0, 'left_hip_pitch_joint': -0.4,
        'left_knee_joint': 0.8, 'left_ankle_pitch_joint': -0.4, 'left_ankle_roll_joint': 0.0,
        'right_hip_yaw_joint': 0.0, 'right_hip_roll_joint': 0.0, 'right_hip_pitch_joint': -0.4,
        'right_knee_joint': 0.8, 'right_ankle_pitch_joint': -0.4, 'right_ankle_roll_joint': 0.0,
    }

    def __init__(self):
        super().__init__('balance_controller')

        # Declare parameters
        self.declare_parameter('rate', 100.0)  # Control rate Hz
        self.declare_parameter('kp_pitch', 2.0)  # Proportional gain for pitch
        self.declare_parameter('kd_pitch', 0.5)  # Derivative gain for pitch
        self.declare_parameter('kp_roll', 2.0)   # Proportional gain for roll
        self.declare_parameter('kd_roll', 0.5)   # Derivative gain for roll
        self.declare_parameter('max_ankle_adjust', 0.3)  # Max ankle adjustment (rad)
        self.declare_parameter('max_hip_adjust', 0.2)    # Max hip adjustment (rad)

        # Get parameters
        rate = self.get_parameter('rate').get_parameter_value().double_value
        self.kp_pitch = self.get_parameter('kp_pitch').get_parameter_value().double_value
        self.kd_pitch = self.get_parameter('kd_pitch').get_parameter_value().double_value
        self.kp_roll = self.get_parameter('kp_roll').get_parameter_value().double_value
        self.kd_roll = self.get_parameter('kd_roll').get_parameter_value().double_value
        self.max_ankle_adjust = self.get_parameter('max_ankle_adjust').get_parameter_value().double_value
        self.max_hip_adjust = self.get_parameter('max_hip_adjust').get_parameter_value().double_value

        # State storage
        self.current_positions = dict(self.STANDING_POSE)
        self.target_positions = dict(self.STANDING_POSE)

        # IMU data
        self.pitch = 0.0  # Forward/backward tilt
        self.roll = 0.0   # Left/right tilt
        self.pitch_rate = 0.0
        self.roll_rate = 0.0

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', qos)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos)

        # Control timer
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('Balance Controller initialized')
        self.get_logger().info(f'  Pitch gains: Kp={self.kp_pitch}, Kd={self.kd_pitch}')
        self.get_logger().info(f'  Roll gains: Kp={self.kp_roll}, Kd={self.kd_roll}')

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from joint states."""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def imu_callback(self, msg: Imu):
        """Extract orientation and angular velocity from IMU."""
        # Get angular velocity (gyro)
        self.pitch_rate = msg.angular_velocity.y
        self.roll_rate = msg.angular_velocity.x

        # Convert quaternion to roll/pitch
        # Using simple approximation for small angles
        q = msg.orientation
        # Roll (rotation around X)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation around Y)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.pitch = np.copysign(np.pi / 2, sinp)
        else:
            self.pitch = np.arcsin(sinp)

    def control_loop(self):
        """Main balance control loop."""
        # Start with standing pose
        cmd = dict(self.STANDING_POSE)

        # PD control for pitch (forward/backward balance)
        # If leaning forward (positive pitch), push ankles back and hips forward
        pitch_correction = -(self.kp_pitch * self.pitch + self.kd_pitch * self.pitch_rate)
        pitch_correction = np.clip(pitch_correction, -self.max_ankle_adjust, self.max_ankle_adjust)

        # Apply ankle pitch correction (both ankles move same direction)
        cmd['left_ankle_pitch_joint'] = self.STANDING_POSE['left_ankle_pitch_joint'] + pitch_correction
        cmd['right_ankle_pitch_joint'] = self.STANDING_POSE['right_ankle_pitch_joint'] + pitch_correction

        # Apply hip pitch correction (opposite to ankle for balance)
        hip_pitch_correction = np.clip(-pitch_correction * 0.5, -self.max_hip_adjust, self.max_hip_adjust)
        cmd['left_hip_pitch_joint'] = self.STANDING_POSE['left_hip_pitch_joint'] + hip_pitch_correction
        cmd['right_hip_pitch_joint'] = self.STANDING_POSE['right_hip_pitch_joint'] + hip_pitch_correction

        # PD control for roll (left/right balance)
        roll_correction = -(self.kp_roll * self.roll + self.kd_roll * self.roll_rate)
        roll_correction = np.clip(roll_correction, -self.max_ankle_adjust, self.max_ankle_adjust)

        # Apply ankle roll correction (opposite directions)
        cmd['left_ankle_roll_joint'] = self.STANDING_POSE['left_ankle_roll_joint'] - roll_correction
        cmd['right_ankle_roll_joint'] = self.STANDING_POSE['right_ankle_roll_joint'] + roll_correction

        # Apply hip roll correction
        hip_roll_correction = np.clip(roll_correction * 0.3, -self.max_hip_adjust, self.max_hip_adjust)
        cmd['left_hip_roll_joint'] = self.STANDING_POSE['left_hip_roll_joint'] + hip_roll_correction
        cmd['right_hip_roll_joint'] = self.STANDING_POSE['right_hip_roll_joint'] - hip_roll_correction

        # Publish command
        msg = Float64MultiArray()
        msg.data = [cmd[name] for name in self.JOINT_NAMES]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
