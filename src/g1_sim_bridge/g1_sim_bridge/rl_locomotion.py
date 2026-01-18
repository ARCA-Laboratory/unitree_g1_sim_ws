#!/usr/bin/env python3
"""
RL-based Locomotion Controller for Unitree G1.

This runs a pre-trained reinforcement learning policy from unitree_rl_gym
to make the G1 walk. The policy controls 12 DOF (legs only).

The pre-trained model is from:
https://github.com/unitreerobotics/unitree_rl_gym

Requirements:
    pip install torch

Usage:
    ros2 run g1_sim_bridge rl_locomotion

Author: Adapted from unitree_rl_gym deploy_mujoco.py
License: BSD-3-Clause
"""

import os
import time
import numpy as np

import mujoco
import mujoco.viewer
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster


def get_gravity_orientation(quaternion):
    """Convert quaternion to gravity vector in body frame."""
    qw, qx, qy, qz = quaternion
    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """PD position control."""
    return (target_q - q) * kp + (target_dq - dq) * kd


def load_g1_model(use_official_model: bool = False):
    """Load G1 model for RL locomotion.

    Args:
        use_official_model: If True, try to load official model with meshes (slower but prettier).
                           If False, use simplified model (faster, default).
    """
    if use_official_model:
        # Try to find the official model
        model_paths = [
            "/tmp/unitree_rl_gym/resources/robots/g1_description/scene.xml",
            os.path.expanduser("~/unitree_rl_gym/resources/robots/g1_description/scene.xml"),
        ]

        for path in model_paths:
            if os.path.exists(path):
                print(f"Loading official G1 model from: {path}")
                model = mujoco.MjModel.from_xml_path(path)
                model.opt.timestep = 0.002
                return model

        print("WARNING: Official G1 model not found, using simplified model")

    print("Using simplified G1 model (faster)")
    return create_simplified_g1_model()


def create_simplified_g1_model():
    """Create a simplified G1 model as fallback."""
    xml = """
    <mujoco model="g1_12dof">
        <compiler angle="radian" autolimits="true"/>
        <option timestep="0.002" iterations="50" solver="Newton" tolerance="1e-10" gravity="0 0 -9.81"/>

        <default>
            <joint damping="0.001" armature="0.01" frictionloss="0.1"/>
            <geom friction="1.0 0.5 0.5" margin="0.001"/>
        </default>

        <worldbody>
            <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
            <geom name="floor" type="plane" size="10 10 0.1" rgba="0.9 0.9 0.9 1"/>

            <!-- Base/Pelvis - floating -->
            <body name="pelvis" pos="0 0 0.793">
                <freejoint name="floating_base_joint"/>
                <inertial pos="0.0145 0.00015 0.144" quat="0.9999 -0.0005 -0.0154 0.0003" mass="17.73" diaginertia="0.553 0.454 0.212"/>
                <geom type="box" size="0.12 0.1 0.1" rgba="0.2 0.2 0.2 1"/>

                <!-- LEFT LEG -->
                <body name="left_hip_pitch_link" pos="0 0.064452 -0.1027">
                    <joint name="left_hip_pitch_joint" type="hinge" axis="0 1 0" range="-2.5307 2.8798"/>
                    <inertial pos="0.00274 0.0478 -0.0261" quat="0.955 0.294 0.030 0.030" mass="1.35" diaginertia="0.00182 0.00153 0.00116"/>
                    <geom type="cylinder" size="0.04 0.03" pos="0 0.05 -0.03" euler="1.57 0 0" rgba="0.2 0.2 0.2 1"/>

                    <body name="left_hip_roll_link" pos="0 0.052 -0.030465" quat="0.996179 0 -0.0873386 0">
                        <joint name="left_hip_roll_joint" type="hinge" axis="1 0 0" range="-0.5236 2.9671"/>
                        <inertial pos="0.0298 -0.001 -0.0879" quat="0.978 0 0.206 -0.040" mass="1.52" diaginertia="0.00255 0.00241 0.00149"/>
                        <geom type="cylinder" size="0.035 0.04" pos="0.03 0 -0.05" rgba="0.7 0.7 0.7 1"/>

                        <body name="left_hip_yaw_link" pos="0.025001 0 -0.12412">
                            <joint name="left_hip_yaw_joint" type="hinge" axis="0 0 1" range="-2.7576 2.7576"/>
                            <inertial pos="-0.0577 -0.011 -0.151" quat="0.601 0.158 0.223 0.751" mass="1.702" diaginertia="0.00776 0.00718 0.00160"/>
                            <geom type="box" size="0.035 0.035 0.12" pos="-0.04 0 -0.12" rgba="0.7 0.7 0.7 1"/>

                            <body name="left_knee_link" pos="-0.078273 0.0021489 -0.17734" quat="0.996179 0 0.0873386 0">
                                <joint name="left_knee_joint" type="hinge" axis="0 1 0" range="-0.087267 2.8798"/>
                                <inertial pos="0.00546 0.00396 -0.121" quat="0.923 -0.033 0.016 0.382" mass="1.932" diaginertia="0.0114 0.0113 0.00146"/>
                                <geom type="box" size="0.025 0.025 0.12" pos="0.01 0 -0.12" rgba="0.7 0.7 0.7 1"/>

                                <body name="left_ankle_pitch_link" pos="0 -9.4445e-05 -0.30001">
                                    <joint name="left_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87267 0.5236"/>
                                    <inertial pos="-0.00727 0 0.0111" mass="0.074" diaginertia="1.89e-05 1.41e-05 6.92e-06"/>
                                    <geom type="sphere" size="0.02" rgba="0.7 0.7 0.7 1"/>

                                    <body name="left_ankle_roll_link" pos="0 0 -0.017558">
                                        <joint name="left_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.2618 0.2618"/>
                                        <inertial pos="0.0265 0 -0.0164" mass="0.608" diaginertia="0.00167 0.00162 0.000218"/>
                                        <geom type="box" size="0.085 0.055 0.015" pos="0.035 0 -0.015" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="-0.05 0.025 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="-0.05 -0.025 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="0.12 0.03 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="0.12 -0.03 -0.03" rgba="0.2 0.2 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>

                <!-- RIGHT LEG -->
                <body name="right_hip_pitch_link" pos="0 -0.064452 -0.1027">
                    <joint name="right_hip_pitch_joint" type="hinge" axis="0 1 0" range="-2.5307 2.8798"/>
                    <inertial pos="0.00274 -0.0478 -0.0261" quat="0.955 -0.294 0.030 -0.030" mass="1.35" diaginertia="0.00182 0.00153 0.00116"/>
                    <geom type="cylinder" size="0.04 0.03" pos="0 -0.05 -0.03" euler="1.57 0 0" rgba="0.2 0.2 0.2 1"/>

                    <body name="right_hip_roll_link" pos="0 -0.052 -0.030465" quat="0.996179 0 -0.0873386 0">
                        <joint name="right_hip_roll_joint" type="hinge" axis="1 0 0" range="-2.9671 0.5236"/>
                        <inertial pos="0.0298 0.001 -0.0879" quat="0.978 0 0.206 0.040" mass="1.52" diaginertia="0.00255 0.00241 0.00149"/>
                        <geom type="cylinder" size="0.035 0.04" pos="0.03 0 -0.05" rgba="0.7 0.7 0.7 1"/>

                        <body name="right_hip_yaw_link" pos="0.025001 0 -0.12412">
                            <joint name="right_hip_yaw_joint" type="hinge" axis="0 0 1" range="-2.7576 2.7576"/>
                            <inertial pos="-0.0577 0.011 -0.151" quat="0.751 0.223 0.158 0.601" mass="1.702" diaginertia="0.00776 0.00718 0.00160"/>
                            <geom type="box" size="0.035 0.035 0.12" pos="-0.04 0 -0.12" rgba="0.7 0.7 0.7 1"/>

                            <body name="right_knee_link" pos="-0.078273 -0.0021489 -0.17734" quat="0.996179 0 0.0873386 0">
                                <joint name="right_knee_joint" type="hinge" axis="0 1 0" range="-0.087267 2.8798"/>
                                <inertial pos="0.00546 -0.00396 -0.121" quat="0.923 0.035 0.012 -0.382" mass="1.932" diaginertia="0.0114 0.0113 0.00146"/>
                                <geom type="box" size="0.025 0.025 0.12" pos="0.01 0 -0.12" rgba="0.7 0.7 0.7 1"/>

                                <body name="right_ankle_pitch_link" pos="0 9.4445e-05 -0.30001">
                                    <joint name="right_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87267 0.5236"/>
                                    <inertial pos="-0.00727 0 0.0111" mass="0.074" diaginertia="1.89e-05 1.41e-05 6.92e-06"/>
                                    <geom type="sphere" size="0.02" rgba="0.7 0.7 0.7 1"/>

                                    <body name="right_ankle_roll_link" pos="0 0 -0.017558">
                                        <joint name="right_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.2618 0.2618"/>
                                        <inertial pos="0.0265 0 -0.0164" mass="0.608" diaginertia="0.00167 0.00162 0.000218"/>
                                        <geom type="box" size="0.085 0.055 0.015" pos="0.035 0 -0.015" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="-0.05 0.025 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="-0.05 -0.025 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="0.12 0.03 -0.03" rgba="0.2 0.2 0.2 1"/>
                                        <geom size="0.005" pos="0.12 -0.03 -0.03" rgba="0.2 0.2 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>

        <actuator>
            <motor name="left_hip_pitch_joint" joint="left_hip_pitch_joint"/>
            <motor name="left_hip_roll_joint" joint="left_hip_roll_joint"/>
            <motor name="left_hip_yaw_joint" joint="left_hip_yaw_joint"/>
            <motor name="left_knee_joint" joint="left_knee_joint"/>
            <motor name="left_ankle_pitch_joint" joint="left_ankle_pitch_joint"/>
            <motor name="left_ankle_roll_joint" joint="left_ankle_roll_joint"/>
            <motor name="right_hip_pitch_joint" joint="right_hip_pitch_joint"/>
            <motor name="right_hip_roll_joint" joint="right_hip_roll_joint"/>
            <motor name="right_hip_yaw_joint" joint="right_hip_yaw_joint"/>
            <motor name="right_knee_joint" joint="right_knee_joint"/>
            <motor name="right_ankle_pitch_joint" joint="right_ankle_pitch_joint"/>
            <motor name="right_ankle_roll_joint" joint="right_ankle_roll_joint"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


class RLLocomotionController:
    """RL-based locomotion controller using pre-trained policy."""

    # Joint order matching the RL policy training
    JOINT_NAMES = [
        'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    ]

    def __init__(self, policy_path: str):
        """Initialize with path to pre-trained policy."""
        # Load policy
        print(f"Loading RL policy from: {policy_path}")
        self.policy = torch.jit.load(policy_path)
        self.policy.eval()

        # Config from g1.yaml
        self.kps = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
        self.kds = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
        self.default_angles = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                                         -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], dtype=np.float32)

        # Higher gains for standing (stiffer stance)
        self.standing_kps = np.array([200, 200, 200, 300, 100, 100, 200, 200, 200, 300, 100, 100], dtype=np.float32)
        self.standing_kds = np.array([10, 10, 10, 15, 8, 8, 10, 10, 10, 15, 8, 8], dtype=np.float32)

        # Scaling factors
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.action_scale = 0.25
        self.cmd_scale = np.array([2.0, 2.0, 0.25], dtype=np.float32)

        # State
        self.num_actions = 12
        self.num_obs = 47
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()

        # Command: [forward_vel, lateral_vel, yaw_rate]
        # Start with small forward velocity - the RL policy was trained to walk, not stand
        self.cmd = np.array([0.3, 0.0, 0.0], dtype=np.float32)

        # Standing mode flag - disabled by default since policy handles low-speed balance
        self.standing_mode = False
        self.cmd_threshold = 0.01  # Very low threshold - policy handles most cases

        # Timing
        self.counter = 0
        self.dt = 0.002
        self.control_decimation = 10  # Policy runs at 50Hz

        print(f"RL controller initialized - starting with slow forward walk")

    def set_command(self, forward: float, lateral: float, yaw: float):
        """Set velocity command [m/s, m/s, rad/s]."""
        self.cmd = np.array([forward, lateral, yaw], dtype=np.float32)

        # The RL policy handles all velocities including zero
        # Standing mode is disabled - the policy was trained to walk, not stand statically
        # If you send zero velocity, the robot will walk in place / balance dynamically

    def step(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        """
        Compute control torques given robot state.

        Args:
            qpos: Joint positions [7 (floating base) + 12 (joints)]
            qvel: Joint velocities [6 (floating base) + 12 (joints)]

        Returns:
            Torques for 12 joints
        """
        self.counter += 1

        # Get joint states (skip floating base)
        q = qpos[7:]  # 12 joint positions
        dq = qvel[6:]  # 12 joint velocities

        # Always use RL policy - it was trained to handle all velocity commands
        # PD control to track target positions
        tau = pd_control(self.target_dof_pos, q, self.kps,
                        np.zeros_like(self.kds), dq, self.kds)

        # Run policy at lower rate
        if self.counter % self.control_decimation == 0:
            # Build observation
            quat = qpos[3:7]  # Quaternion [w, x, y, z]
            omega = qvel[3:6]  # Angular velocity

            # Normalize joint positions/velocities
            qj_norm = (q - self.default_angles) * self.dof_pos_scale
            dqj_norm = dq * self.dof_vel_scale
            gravity_orientation = get_gravity_orientation(quat)
            omega_norm = omega * self.ang_vel_scale

            # Phase for gait timing
            period = 0.8
            count = self.counter * self.dt
            phase = (count % period) / period
            sin_phase = np.sin(2 * np.pi * phase)
            cos_phase = np.cos(2 * np.pi * phase)

            # Build observation vector
            obs = np.zeros(self.num_obs, dtype=np.float32)
            obs[0:3] = omega_norm
            obs[3:6] = gravity_orientation
            obs[6:9] = self.cmd * self.cmd_scale
            obs[9:9+self.num_actions] = qj_norm
            obs[9+self.num_actions:9+2*self.num_actions] = dqj_norm
            obs[9+2*self.num_actions:9+3*self.num_actions] = self.action
            obs[9+3*self.num_actions:9+3*self.num_actions+2] = np.array([sin_phase, cos_phase])

            # Policy inference
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                self.action = self.policy(obs_tensor).numpy().squeeze()

            # Convert action to target positions
            self.target_dof_pos = self.action * self.action_scale + self.default_angles

        return tau


class RLLocomotionNode(Node):
    """ROS 2 Node for RL locomotion with MuJoCo simulation."""

    # All joint names matching the URDF exactly (30 joints total)
    URDF_JOINT_NAMES = [
        # Waist (3)
        'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
        # Head (1)
        'head_joint',
        # Left arm (7)
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
        'left_elbow_joint', 'left_wrist_yaw_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint',
        # Right arm (7)
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
        'right_elbow_joint', 'right_wrist_yaw_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint',
        # Left leg (6)
        'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        # Right leg (6)
        'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    ]

    # Mapping from RL policy joint order to URDF joint indices
    # RL policy order: [L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_ankle_pitch, L_ankle_roll,
    #                   R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_ankle_pitch, R_ankle_roll]
    # URDF leg order: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
    RL_TO_URDF_LEG_MAP = {
        0: 20,   # left_hip_pitch -> index 20 in URDF
        1: 19,   # left_hip_roll -> index 19
        2: 18,   # left_hip_yaw -> index 18
        3: 21,   # left_knee -> index 21
        4: 22,   # left_ankle_pitch -> index 22
        5: 23,   # left_ankle_roll -> index 23
        6: 26,   # right_hip_pitch -> index 26
        7: 25,   # right_hip_roll -> index 25
        8: 24,   # right_hip_yaw -> index 24
        9: 27,   # right_knee -> index 27
        10: 28,  # right_ankle_pitch -> index 28
        11: 29,  # right_ankle_roll -> index 29
    }

    def __init__(self, controller, model, data):
        super().__init__('rl_locomotion')
        self.controller = controller
        self.model = model
        self.data = data

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_rl', self.cmd_callback, 10)

        # Timer for publishing (100 Hz)
        self.create_timer(0.01, self.publish_state)

        self.get_logger().info('RL Locomotion node initialized')
        self.get_logger().info('Subscribing to /cmd_vel_rl for velocity commands')

    def cmd_callback(self, msg: Twist):
        """Handle velocity command from teleop."""
        self.controller.set_command(
            msg.linear.x,   # forward
            msg.linear.y,   # lateral
            msg.angular.z   # yaw rate
        )

    def publish_state(self):
        """Publish joint states and TF for RViz."""
        now = self.get_clock().now().to_msg()

        # Publish joint states
        js = JointState()
        js.header.stamp = now
        js.name = self.URDF_JOINT_NAMES

        # Initialize all 30 joints to zero
        positions = [0.0] * 30

        # Get leg joint positions from simulation (12 joints starting at qpos[7])
        rl_leg_positions = self.data.qpos[7:19]

        # Map RL policy joints to URDF joint positions
        for rl_idx, urdf_idx in self.RL_TO_URDF_LEG_MAP.items():
            positions[urdf_idx] = float(rl_leg_positions[rl_idx])

        js.position = positions
        js.velocity = [0.0] * len(js.name)
        js.effort = [0.0] * len(js.name)

        self.joint_pub.publish(js)

        # Publish base TF (world -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Position from floating base
        t.transform.translation.x = float(self.data.qpos[0])
        t.transform.translation.y = float(self.data.qpos[1])
        t.transform.translation.z = float(self.data.qpos[2])

        # Quaternion (MuJoCo uses w,x,y,z - ROS uses x,y,z,w)
        t.transform.rotation.w = float(self.data.qpos[3])
        t.transform.rotation.x = float(self.data.qpos[4])
        t.transform.rotation.y = float(self.data.qpos[5])
        t.transform.rotation.z = float(self.data.qpos[6])

        self.tf_broadcaster.sendTransform(t)


def main():
    """Run the RL locomotion controller with ROS 2 integration."""
    import argparse

    parser = argparse.ArgumentParser(description='G1 RL Locomotion Controller')
    parser.add_argument('--official-model', action='store_true',
                        help='Use official G1 model with meshes (slower but prettier)')
    args, _ = parser.parse_known_args()

    # Initialize ROS 2
    rclpy.init()

    # Find policy file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    policy_path = os.path.join(script_dir, '..', 'models', 'g1_locomotion.pt')

    if not os.path.exists(policy_path):
        # Try installed location
        import ament_index_python
        try:
            pkg_dir = ament_index_python.get_package_share_directory('g1_sim_bridge')
            policy_path = os.path.join(pkg_dir, 'models', 'g1_locomotion.pt')
        except:
            pass

    if not os.path.exists(policy_path):
        print(f"ERROR: Policy file not found at {policy_path}")
        print("Please ensure g1_locomotion.pt is in the models directory")
        rclpy.shutdown()
        return

    # Create model and controller
    print("Loading G1 model...")
    model = load_g1_model(use_official_model=args.official_model)
    data = mujoco.MjData(model)

    print("Initializing RL controller...")
    controller = RLLocomotionController(policy_path)

    # Start with slow forward walk - the RL policy needs some velocity to balance
    # (it was trained to walk, not stand still)
    controller.set_command(0.3, 0.0, 0.0)

    # Create ROS 2 node
    node = RLLocomotionNode(controller, model, data)

    # Set initial pose - set joint angles to default standing pose
    for i, angle in enumerate(controller.default_angles):
        data.qpos[7 + i] = angle

    # Run a few physics steps to settle the robot
    print("Settling initial pose...")
    for _ in range(100):
        tau = controller.step(data.qpos, data.qvel)
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)

    print("\nStarting simulation with ROS 2 integration...")
    print("Robot starting with slow forward walk (0.3 m/s).")
    print("")
    print("To control the robot, run in another terminal:")
    print("  ros2 run g1_sim_bridge rl_keyboard_teleop")
    print("")
    print("Or use: ./scripts/run_sim.sh walk")
    print("        (starts both simulation and keyboard control)")
    print("")
    print("RViz: Joint states published to /joint_states")
    print("Velocity commands: Subscribe to /cmd_vel_rl")
    print("")

    # Run simulation with viewer
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and rclpy.ok():
                step_start = time.time()

                # Get control torques from RL policy
                tau = controller.step(data.qpos, data.qvel)
                data.ctrl[:] = tau

                # Step physics
                mujoco.mj_step(model, data)

                # Spin ROS 2 (non-blocking)
                rclpy.spin_once(node, timeout_sec=0)

                # Update viewer
                viewer.sync()

                # Timing
                elapsed = time.time() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nShutdown complete.")


if __name__ == '__main__':
    main()
