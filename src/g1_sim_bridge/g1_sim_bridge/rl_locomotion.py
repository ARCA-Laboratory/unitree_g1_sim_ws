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


def create_g1_12dof_model():
    """Create a simplified G1 model with only 12 leg DOF for the RL policy."""
    xml = """
    <mujoco model="g1_12dof">
        <compiler angle="radian" autolimits="true"/>

        <option timestep="0.002" iterations="50" solver="Newton" tolerance="1e-10" gravity="0 0 -9.81"/>

        <default>
            <joint damping="2" armature="0.01"/>
            <geom friction="1.5 0.5 0.5" margin="0.001"/>
        </default>

        <worldbody>
            <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
            <geom name="floor" type="plane" size="10 10 0.1" rgba="0.9 0.9 0.9 1"/>

            <!-- Base/Pelvis - floating -->
            <body name="pelvis" pos="0 0 0.75">
                <freejoint name="root"/>
                <inertial pos="0 0 -0.05" mass="10" diaginertia="0.1 0.1 0.05"/>
                <geom type="box" size="0.1 0.08 0.08" rgba="0.3 0.3 0.3 1"/>

                <!-- Torso (fixed to pelvis for 12dof) -->
                <geom type="box" size="0.08 0.1 0.2" pos="0 0 0.15" rgba="0.4 0.4 0.4 1"/>
                <geom type="sphere" size="0.08" pos="0 0 0.4" rgba="0.5 0.5 0.5 1"/>

                <!-- IMU site -->
                <site name="imu_site" pos="0 0 0" size="0.01"/>

                <!-- LEFT LEG -->
                <body name="left_hip_pitch_link" pos="0 0.0645 -0.1027">
                    <joint name="left_hip_pitch_joint" type="hinge" axis="0 1 0" range="-2.53 2.88"/>
                    <inertial pos="0 0.05 -0.03" mass="1.35" diaginertia="0.002 0.002 0.001"/>
                    <geom type="cylinder" size="0.04 0.03" pos="0 0.05 -0.03" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                    <body name="left_hip_roll_link" pos="0 0.052 -0.0305">
                        <joint name="left_hip_roll_joint" type="hinge" axis="1 0 0" range="-0.52 2.97"/>
                        <inertial pos="0.03 0 -0.09" mass="1.52" diaginertia="0.003 0.003 0.001"/>
                        <geom type="cylinder" size="0.035 0.04" pos="0.03 0 -0.05" rgba="0.4 0.4 0.4 1"/>

                        <body name="left_hip_yaw_link" pos="0.025 0 -0.124">
                            <joint name="left_hip_yaw_joint" type="hinge" axis="0 0 1" range="-2.76 2.76"/>
                            <inertial pos="-0.06 -0.01 -0.15" mass="1.7" diaginertia="0.008 0.007 0.002"/>
                            <geom type="box" size="0.035 0.035 0.15" pos="-0.04 0 -0.15" rgba="0.3 0.3 0.3 1"/>

                            <body name="left_knee_link" pos="-0.078 0.002 -0.177">
                                <joint name="left_knee_joint" type="hinge" axis="0 1 0" range="-0.09 2.53"/>
                                <inertial pos="0.01 0 -0.12" mass="1.58" diaginertia="0.007 0.007 0.001"/>
                                <geom type="box" size="0.025 0.025 0.15" pos="0.01 0 -0.12" rgba="0.4 0.4 0.4 1"/>

                                <body name="left_ankle_pitch_link" pos="0.012 -0.002 -0.3">
                                    <joint name="left_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87 0.52"/>
                                    <inertial pos="0 0 0" mass="0.46" diaginertia="0.0003 0.0003 0.0002"/>
                                    <geom type="sphere" size="0.025" rgba="0.3 0.3 0.3 1"/>

                                    <body name="left_ankle_roll_link" pos="0 0 0">
                                        <joint name="left_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.26 0.26"/>
                                        <inertial pos="0.03 0 -0.02" mass="0.65" diaginertia="0.001 0.002 0.002"/>
                                        <geom type="box" size="0.08 0.04 0.015" pos="0.03 0 -0.015" rgba="0.2 0.2 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>

                <!-- RIGHT LEG -->
                <body name="right_hip_pitch_link" pos="0 -0.0645 -0.1027">
                    <joint name="right_hip_pitch_joint" type="hinge" axis="0 1 0" range="-2.53 2.88"/>
                    <inertial pos="0 -0.05 -0.03" mass="1.35" diaginertia="0.002 0.002 0.001"/>
                    <geom type="cylinder" size="0.04 0.03" pos="0 -0.05 -0.03" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                    <body name="right_hip_roll_link" pos="0 -0.052 -0.0305">
                        <joint name="right_hip_roll_joint" type="hinge" axis="1 0 0" range="-2.97 0.52"/>
                        <inertial pos="0.03 0 -0.09" mass="1.52" diaginertia="0.003 0.003 0.001"/>
                        <geom type="cylinder" size="0.035 0.04" pos="0.03 0 -0.05" rgba="0.4 0.4 0.4 1"/>

                        <body name="right_hip_yaw_link" pos="0.025 0 -0.124">
                            <joint name="right_hip_yaw_joint" type="hinge" axis="0 0 1" range="-2.76 2.76"/>
                            <inertial pos="-0.06 0.01 -0.15" mass="1.7" diaginertia="0.008 0.007 0.002"/>
                            <geom type="box" size="0.035 0.035 0.15" pos="-0.04 0 -0.15" rgba="0.3 0.3 0.3 1"/>

                            <body name="right_knee_link" pos="-0.078 -0.002 -0.177">
                                <joint name="right_knee_joint" type="hinge" axis="0 1 0" range="-0.09 2.53"/>
                                <inertial pos="0.01 0 -0.12" mass="1.58" diaginertia="0.007 0.007 0.001"/>
                                <geom type="box" size="0.025 0.025 0.15" pos="0.01 0 -0.12" rgba="0.4 0.4 0.4 1"/>

                                <body name="right_ankle_pitch_link" pos="0.012 0.002 -0.3">
                                    <joint name="right_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87 0.52"/>
                                    <inertial pos="0 0 0" mass="0.46" diaginertia="0.0003 0.0003 0.0002"/>
                                    <geom type="sphere" size="0.025" rgba="0.3 0.3 0.3 1"/>

                                    <body name="right_ankle_roll_link" pos="0 0 0">
                                        <joint name="right_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.26 0.26"/>
                                        <inertial pos="0.03 0 -0.02" mass="0.65" diaginertia="0.001 0.002 0.002"/>
                                        <geom type="box" size="0.08 0.04 0.015" pos="0.03 0 -0.015" rgba="0.2 0.2 0.2 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>

        <sensor>
            <accelerometer name="imu_accel" site="imu_site"/>
            <gyro name="imu_gyro" site="imu_site"/>
        </sensor>
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
        self.cmd = np.array([0.5, 0.0, 0.0], dtype=np.float32)  # Walk forward

        # Timing
        self.counter = 0
        self.dt = 0.002
        self.control_decimation = 10  # Policy runs at 50Hz

        print(f"RL controller initialized with command: {self.cmd}")

    def set_command(self, forward: float, lateral: float, yaw: float):
        """Set velocity command [m/s, m/s, rad/s]."""
        self.cmd = np.array([forward, lateral, yaw], dtype=np.float32)

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


def main():
    """Run the RL locomotion controller standalone."""
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
        return

    # Create model and controller
    print("Creating G1 12-DOF model...")
    model = create_g1_12dof_model()
    data = mujoco.MjData(model)

    print("Initializing RL controller...")
    controller = RLLocomotionController(policy_path)

    # Set initial pose
    data.qpos[2] = 0.75  # Height
    data.qpos[3] = 1.0   # Quaternion w
    for i, angle in enumerate(controller.default_angles):
        data.qpos[7 + i] = angle

    print("\nStarting simulation...")
    print("Commands: forward=0.5 m/s, lateral=0, yaw=0")
    print("Press Ctrl+C to exit\n")

    # Run simulation with viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            step_start = time.time()

            # Get control torques from RL policy
            tau = controller.step(data.qpos, data.qvel)
            data.ctrl[:] = tau

            # Step physics
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Timing
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == '__main__':
    main()
