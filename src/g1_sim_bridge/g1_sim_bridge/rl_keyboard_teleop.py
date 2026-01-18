#!/usr/bin/env python3
"""
Keyboard teleop for RL locomotion velocity commands.

This node provides keyboard control for velocity commands and publishes them
to /cmd_vel_rl for the RL locomotion controller to consume.

Usage:
    ros2 run g1_sim_bridge rl_keyboard_teleop

Author: Adapted for Unitree G1 simulation
License: BSD-3-Clause
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_TEXT = """
╔════════════════════════════════════════════════════════════╗
║              RL Locomotion Keyboard Controls               ║
╠════════════════════════════════════════════════════════════╣
║  Movement:                                                 ║
║    W / S  - Forward / Backward                             ║
║    A / D  - Turn Left / Turn Right                         ║
║    Q / E  - Strafe Left / Strafe Right                     ║
║                                                            ║
║  Commands:                                                 ║
║    SPACE  - Walk in place (minimal velocity)               ║
║    R      - Reset to default forward walk (0.5 m/s)        ║
║    +/-    - Increase/Decrease speed                        ║
║    H      - Show this help                                 ║
║    ESC    - Quit                                           ║
║                                                            ║
║  Note: Robot will always walk - policy doesn't stand still ║
╚════════════════════════════════════════════════════════════╝
"""


class RLKeyboardTeleop(Node):
    """Keyboard teleop node for RL locomotion velocity commands."""

    def __init__(self):
        super().__init__('rl_keyboard_teleop')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_rl', 10)

        # Velocity limits
        self.max_forward = 1.0   # m/s
        self.max_lateral = 0.5   # m/s
        self.max_yaw = 1.0       # rad/s

        # Current velocities
        self.forward_vel = 0.0
        self.lateral_vel = 0.0
        self.yaw_rate = 0.0

        # Speed multiplier
        self.speed_scale = 1.0

        # Store original terminal settings
        self.old_settings = None

        self.get_logger().info('RL Keyboard Teleop node initialized')
        self.get_logger().info('Publishing velocity commands to /cmd_vel_rl')

    def setup_terminal(self):
        """Set terminal to raw mode for non-blocking input."""
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        """Restore terminal settings."""
        if self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """Get a single keypress (non-blocking)."""
        if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1).lower()
        return None

    def handle_key(self, key):
        """Handle a single keypress."""
        step_forward = 0.1 * self.speed_scale
        step_lateral = 0.1 * self.speed_scale
        step_yaw = 0.2 * self.speed_scale

        if key == 'w':
            self.forward_vel = min(self.forward_vel + step_forward, self.max_forward)
        elif key == 's':
            self.forward_vel = max(self.forward_vel - step_forward, -self.max_forward)
        elif key == 'a':
            self.yaw_rate = min(self.yaw_rate + step_yaw, self.max_yaw)
        elif key == 'd':
            self.yaw_rate = max(self.yaw_rate - step_yaw, -self.max_yaw)
        elif key == 'q':
            self.lateral_vel = min(self.lateral_vel + step_lateral, self.max_lateral)
        elif key == 'e':
            self.lateral_vel = max(self.lateral_vel - step_lateral, -self.max_lateral)
        elif key == ' ':
            # Stop
            self.forward_vel = 0.0
            self.lateral_vel = 0.0
            self.yaw_rate = 0.0
        elif key == 'r':
            # Reset to default forward walk
            self.forward_vel = 0.5
            self.lateral_vel = 0.0
            self.yaw_rate = 0.0
            self.speed_scale = 1.0
        elif key == '+' or key == '=':
            self.speed_scale = min(self.speed_scale + 0.1, 2.0)
            print(f"\rSpeed scale: {self.speed_scale:.1f}x                    ")
        elif key == '-':
            self.speed_scale = max(self.speed_scale - 0.1, 0.2)
            print(f"\rSpeed scale: {self.speed_scale:.1f}x                    ")
        elif key == 'h':
            print(HELP_TEXT)
        elif key == '\x1b':  # ESC
            return False

        # Publish velocity command
        self.publish_cmd()
        return True

    def publish_cmd(self):
        """Publish current velocity command."""
        msg = Twist()
        msg.linear.x = self.forward_vel
        msg.linear.y = self.lateral_vel
        msg.angular.z = self.yaw_rate
        self.cmd_pub.publish(msg)

        # Print status
        print(f"\rCmd: fwd={self.forward_vel:+.2f} m/s, lat={self.lateral_vel:+.2f} m/s, yaw={self.yaw_rate:+.2f} rad/s  ", end='', flush=True)

    def run(self):
        """Main loop."""
        print(HELP_TEXT)
        print("Robot starts with slow forward walk (0.3 m/s). Use WASD to control.")
        print("")

        try:
            self.setup_terminal()

            while rclpy.ok():
                key = self.get_key()
                if key is not None:
                    if not self.handle_key(key):
                        break
                else:
                    # Keep publishing current command even without key press
                    self.publish_cmd()

        except KeyboardInterrupt:
            pass
        finally:
            self.restore_terminal()
            print("\nKeyboard teleop stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = RLKeyboardTeleop()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
