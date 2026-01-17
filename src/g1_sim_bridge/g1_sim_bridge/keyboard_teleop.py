#!/usr/bin/env python3
"""
Keyboard Teleoperation for Unitree G1.

Control the G1 robot's upper body using keyboard inputs.

Keys:
  q/a: Left shoulder pitch up/down
  w/s: Left elbow flex/extend
  e/d: Left wrist pitch up/down

  u/j: Right shoulder pitch up/down
  i/k: Right elbow flex/extend
  o/l: Right wrist pitch up/down

  r: Reset to standing pose
  SPACE: Emergency stop (zero velocities)
  ESC: Quit

Author: Generated for Unitree G1 Simulation
License: BSD-3-Clause
"""

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class KeyboardTeleop(Node):
    """Keyboard teleoperation node for G1 robot."""

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

    # Joint limits (approximate)
    JOINT_LIMITS = {
        'left_shoulder_pitch_joint': (-2.87, 2.87),
        'left_elbow_joint': (-2.35, 0.0),
        'left_wrist_pitch_joint': (-1.0, 1.0),
        'right_shoulder_pitch_joint': (-2.87, 2.87),
        'right_elbow_joint': (0.0, 2.35),
        'right_wrist_pitch_joint': (-1.0, 1.0),
        'waist_yaw_joint': (-0.75, 0.75),
    }

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Current joint positions
        self.positions = dict(self.STANDING_POSE)

        # Step size for joint movement
        self.step_size = 0.1

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

        # Timer for publishing
        self.timer = self.create_timer(0.05, self.publish_command)  # 20 Hz

        # Running flag
        self.running = True

        self.print_instructions()

    def print_instructions(self):
        """Print control instructions."""
        msg = """
╔══════════════════════════════════════════════════════════════════╗
║            Unitree G1 Keyboard Teleoperation                     ║
╠══════════════════════════════════════════════════════════════════╣
║  Left Arm:                    Right Arm:                         ║
║    q/a: Shoulder up/down        u/j: Shoulder up/down            ║
║    w/s: Elbow flex/extend       i/k: Elbow flex/extend           ║
║    e/d: Wrist up/down           o/l: Wrist up/down               ║
║                                                                  ║
║  Waist:                                                          ║
║    z/x: Yaw left/right                                           ║
║                                                                  ║
║  Other:                                                          ║
║    r: Reset to standing pose                                     ║
║    +/-: Increase/decrease step size                              ║
║    ESC or Ctrl+C: Quit                                           ║
╚══════════════════════════════════════════════════════════════════╝
        """
        print(msg)

    def get_key(self):
        """Get a single keypress."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def clamp_joint(self, name, value):
        """Clamp joint value to its limits."""
        if name in self.JOINT_LIMITS:
            low, high = self.JOINT_LIMITS[name]
            return max(low, min(high, value))
        return value

    def process_key(self, key):
        """Process a keypress and update positions."""
        # Left arm
        if key == 'q':
            self.positions['left_shoulder_pitch_joint'] -= self.step_size
        elif key == 'a':
            self.positions['left_shoulder_pitch_joint'] += self.step_size
        elif key == 'w':
            self.positions['left_elbow_joint'] -= self.step_size
        elif key == 's':
            self.positions['left_elbow_joint'] += self.step_size
        elif key == 'e':
            self.positions['left_wrist_pitch_joint'] -= self.step_size
        elif key == 'd':
            self.positions['left_wrist_pitch_joint'] += self.step_size

        # Right arm
        elif key == 'u':
            self.positions['right_shoulder_pitch_joint'] -= self.step_size
        elif key == 'j':
            self.positions['right_shoulder_pitch_joint'] += self.step_size
        elif key == 'i':
            self.positions['right_elbow_joint'] += self.step_size
        elif key == 'k':
            self.positions['right_elbow_joint'] -= self.step_size
        elif key == 'o':
            self.positions['right_wrist_pitch_joint'] -= self.step_size
        elif key == 'l':
            self.positions['right_wrist_pitch_joint'] += self.step_size

        # Waist
        elif key == 'z':
            self.positions['waist_yaw_joint'] += self.step_size
        elif key == 'x':
            self.positions['waist_yaw_joint'] -= self.step_size

        # Reset
        elif key == 'r':
            self.positions = dict(self.STANDING_POSE)
            self.get_logger().info('Reset to standing pose')

        # Step size
        elif key == '+' or key == '=':
            self.step_size = min(0.5, self.step_size + 0.05)
            self.get_logger().info(f'Step size: {self.step_size:.2f}')
        elif key == '-':
            self.step_size = max(0.01, self.step_size - 0.05)
            self.get_logger().info(f'Step size: {self.step_size:.2f}')

        # Quit
        elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
            self.running = False
            return False

        # Apply limits
        for name in self.positions:
            self.positions[name] = self.clamp_joint(name, self.positions[name])

        return True

    def publish_command(self):
        """Publish current joint positions."""
        msg = Float64MultiArray()
        msg.data = [self.positions[name] for name in self.JOINT_NAMES]
        self.cmd_pub.publish(msg)

    def run(self):
        """Main loop for keyboard input."""
        while self.running and rclpy.ok():
            key = self.get_key()
            if not self.process_key(key):
                break


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Run keyboard loop in separate thread
    kb_thread = threading.Thread(target=node.run)
    kb_thread.daemon = True
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
