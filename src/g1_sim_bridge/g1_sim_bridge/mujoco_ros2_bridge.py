#!/usr/bin/env python3
"""
MuJoCo to ROS 2 Bridge for Unitree G1 Humanoid Robot.

This node:
- Runs a MuJoCo simulation of the G1 robot
- Publishes joint states to /joint_states
- Publishes IMU data to /imu/data
- Subscribes to /joint_commands for control
- Publishes TF transforms

Author: Generated for Unitree G1 Simulation
License: BSD-3-Clause
"""

import os
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: MuJoCo not installed. Install with: pip install mujoco")
    sys.exit(1)


class G1MuJoCoSimulator(Node):
    """
    ROS 2 node that runs MuJoCo simulation and bridges to ROS topics.
    """

    # Joint names matching the URDF
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

    def __init__(self):
        super().__init__('g1_mujoco_simulator')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 500.0)  # 500 Hz simulation
        self.declare_parameter('publish_rate', 100.0)  # 100 Hz publishing
        self.declare_parameter('use_viewer', True)
        self.declare_parameter('paused', False)
        self.declare_parameter('fixed_base', False)  # Set True to pin robot for testing

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_viewer = self.get_parameter('use_viewer').get_parameter_value().bool_value
        self.paused = self.get_parameter('paused').get_parameter_value().bool_value
        self.fixed_base = self.get_parameter('fixed_base').get_parameter_value().bool_value

        # Load MuJoCo model
        if model_path and os.path.exists(model_path):
            self.get_logger().info(f'Loading model from: {model_path}')
            self.model = mujoco.MjModel.from_xml_path(model_path)
        else:
            self.get_logger().info(f'No model path provided, using built-in G1 model (fixed_base={self.fixed_base})')
            self.model = self._create_g1_model()

        self.data = mujoco.MjData(self.model)

        # Simulation timestep
        self.model.opt.timestep = 1.0 / self.sim_rate

        # Define standing pose - all zeros for straight standing
        self.standing_pose = np.zeros(len(self.JOINT_NAMES))

        # Joint command storage - initialize to standing pose
        self.joint_commands = self.standing_pose.copy()
        self.command_lock = threading.Lock()

        # Initialize robot to standing pose
        self._set_initial_pose()

        # Create publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', qos)
        self.imu_pub = self.create_publisher(
            Imu, '/imu/data', qos)

        # Create subscriber for joint commands
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, '/joint_commands',
            self.joint_command_callback, qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timers - simulation runs in timer callback now
        self.sim_timer = self.create_timer(
            1.0 / self.sim_rate, self.simulation_step)
        self.pub_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state)

        # Viewer (launched separately if needed)
        self.viewer = None

        self.get_logger().info('G1 MuJoCo Simulator initialized')
        self.get_logger().info(f'  Simulation rate: {self.sim_rate} Hz')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Number of joints: {len(self.JOINT_NAMES)}')

    def _set_initial_pose(self):
        """Set the robot to a stable standing pose."""
        # Set joint positions to standing pose
        for i, joint_name in enumerate(self.JOINT_NAMES):
            try:
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id >= 0:
                    qpos_adr = self.model.jnt_qposadr[joint_id]
                    self.data.qpos[qpos_adr] = self.standing_pose[i]
            except Exception:
                pass

        # If floating base, set base position
        if not self.fixed_base:
            self.data.qpos[0] = 0.0  # x
            self.data.qpos[1] = 0.0  # y
            self.data.qpos[2] = 0.85  # z - height for feet to be on ground
            # Quaternion [w, x, y, z] - upright
            self.data.qpos[3] = 1.0
            self.data.qpos[4] = 0.0
            self.data.qpos[5] = 0.0
            self.data.qpos[6] = 0.0

        # Zero velocities
        self.data.qvel[:] = 0.0

        # Forward kinematics to update positions
        mujoco.mj_forward(self.model, self.data)

    def _apply_control(self):
        """Apply position control to all joints."""
        for i, joint_name in enumerate(self.JOINT_NAMES):
            try:
                actuator_name = joint_name.replace('_joint', '_motor')
                actuator_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                if actuator_id >= 0:
                    # Position actuators take target position directly
                    self.data.ctrl[actuator_id] = self.joint_commands[i]
            except Exception:
                pass

    def _create_g1_model(self):
        """Create a simple G1 MuJoCo model programmatically."""
        # Choose joint type based on fixed_base parameter
        # Leg height: hip(0.07) + thigh(0.30) + calf(0.30) + ankle(0.02) + foot(0.02) = ~0.71m
        # Add some clearance = 0.75m
        if self.fixed_base:
            base_joint = ''  # No joint = fixed to world
            base_height = '0.75'  # Standing height above ground
        else:
            base_joint = '<freejoint name="root"/>'
            base_height = '0.75'

        xml = f"""
        <mujoco model="g1_humanoid">
            <compiler angle="radian" autolimits="true"/>

            <option timestep="0.002" iterations="50" solver="Newton" tolerance="1e-10" gravity="0 0 -9.81"/>

            <default>
                <joint damping="5" armature="0.1"/>
                <geom friction="1.0 0.5 0.5" margin="0.001"/>
                <position kp="100" ctrlrange="-3.14 3.14" ctrllimited="true"/>
            </default>

            <worldbody>
                <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
                <geom name="floor" type="plane" size="10 10 0.1" rgba="0.9 0.9 0.9 1"/>

                <!-- Base/Pelvis -->
                <body name="base_link" pos="0 0 {base_height}">
                    {base_joint}
                    <inertial pos="0 0 0" mass="5" diaginertia="0.02 0.03 0.02"/>
                    <geom type="box" size="0.075 0.06 0.05" rgba="0.3 0.3 0.3 1"/>

                    <!-- IMU sensor site -->
                    <site name="imu_site" pos="0 0 0" size="0.01"/>

                    <!-- Waist Yaw -->
                    <body name="waist_yaw_link" pos="0 0 0.05">
                        <joint name="waist_yaw_joint" type="hinge" axis="0 0 1" range="-0.75 0.75"/>
                        <inertial pos="0 0 0.025" mass="1.5" diaginertia="0.005 0.005 0.003"/>
                        <geom type="cylinder" size="0.06 0.025" pos="0 0 0.025" rgba="0.5 0.5 0.5 1"/>

                        <!-- Waist Roll -->
                        <body name="waist_roll_link" pos="0 0 0.05">
                            <joint name="waist_roll_joint" type="hinge" axis="1 0 0" range="-0.35 0.35"/>
                            <inertial pos="0 0 0.025" mass="1.2" diaginertia="0.004 0.004 0.002"/>
                            <geom type="cylinder" size="0.055 0.025" pos="0 0 0.025" rgba="0.5 0.5 0.5 1"/>

                            <!-- Waist Pitch / Torso -->
                            <body name="torso_link" pos="0 0 0.05">
                                <joint name="waist_pitch_joint" type="hinge" axis="0 1 0" range="-0.35 0.35"/>
                                <inertial pos="0 0 0.15" mass="8" diaginertia="0.15 0.12 0.08"/>
                                <geom type="box" size="0.1 0.125 0.15" pos="0 0 0.15" rgba="0.3 0.3 0.3 1"/>

                                <!-- Head (fixed) -->
                                <body name="head_link" pos="0 0 0.35">
                                    <inertial pos="0 0 0.08" mass="1.5" diaginertia="0.005 0.005 0.003"/>
                                    <geom type="box" size="0.06 0.06 0.08" pos="0 0 0.08" rgba="0.1 0.1 0.1 1"/>
                                </body>

                                <!-- LEFT ARM -->
                                <body name="left_shoulder_pitch_link" pos="0 0.15 0.25">
                                    <joint name="left_shoulder_pitch_joint" type="hinge" axis="0 1 0" range="-2.87 2.87"/>
                                    <inertial pos="0 0 0" mass="0.8" diaginertia="0.001 0.001 0.001"/>
                                    <geom type="sphere" size="0.04" rgba="0.5 0.5 0.5 1"/>

                                    <body name="left_shoulder_roll_link" pos="0 0.02 0">
                                        <joint name="left_shoulder_roll_joint" type="hinge" axis="1 0 0" range="-1.5 0.3"/>
                                        <inertial pos="0 0.05 0" mass="0.6" diaginertia="0.0008 0.0008 0.0005"/>
                                        <geom type="cylinder" size="0.03 0.04" pos="0 0.05 0" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                                        <body name="left_upper_arm_link" pos="0 0.08 0">
                                            <joint name="left_shoulder_yaw_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
                                            <inertial pos="0 0.08 -0.08" mass="1.0" diaginertia="0.003 0.003 0.001"/>
                                            <geom type="box" size="0.025 0.025 0.08" pos="0 0.08 -0.08" rgba="0.3 0.3 0.3 1"/>

                                            <body name="left_lower_arm_link" pos="0 0.08 -0.16">
                                                <joint name="left_elbow_joint" type="hinge" axis="0 1 0" range="-2.35 0"/>
                                                <inertial pos="0 0 -0.08" mass="0.8" diaginertia="0.002 0.002 0.0008"/>
                                                <geom type="box" size="0.02 0.02 0.08" pos="0 0 -0.08" rgba="0.5 0.5 0.5 1"/>

                                                <body name="left_wrist_yaw_link" pos="0 0 -0.16">
                                                    <joint name="left_wrist_yaw_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
                                                    <inertial pos="0 0 0" mass="0.2" diaginertia="0.0001 0.0001 0.0001"/>
                                                    <geom type="cylinder" size="0.025 0.015" rgba="0.3 0.3 0.3 1"/>

                                                    <body name="left_wrist_roll_link" pos="0 0 -0.02">
                                                        <joint name="left_wrist_roll_joint" type="hinge" axis="1 0 0" range="-1.57 1.57"/>
                                                        <inertial pos="0 0 0" mass="0.15" diaginertia="0.00008 0.00008 0.00005"/>
                                                        <geom type="cylinder" size="0.02 0.015" euler="1.57 0 0" rgba="0.5 0.5 0.5 1"/>

                                                        <body name="left_hand_link" pos="0 0 -0.02">
                                                            <joint name="left_wrist_pitch_joint" type="hinge" axis="0 1 0" range="-1.0 1.0"/>
                                                            <inertial pos="0 0 -0.04" mass="0.4" diaginertia="0.0005 0.0005 0.0003"/>
                                                            <geom type="box" size="0.03 0.04 0.04" pos="0 0 -0.04" rgba="0.1 0.1 0.1 1"/>
                                                        </body>
                                                    </body>
                                                </body>
                                            </body>
                                        </body>
                                    </body>
                                </body>

                                <!-- RIGHT ARM -->
                                <body name="right_shoulder_pitch_link" pos="0 -0.15 0.25">
                                    <joint name="right_shoulder_pitch_joint" type="hinge" axis="0 1 0" range="-2.87 2.87"/>
                                    <inertial pos="0 0 0" mass="0.8" diaginertia="0.001 0.001 0.001"/>
                                    <geom type="sphere" size="0.04" rgba="0.5 0.5 0.5 1"/>

                                    <body name="right_shoulder_roll_link" pos="0 -0.02 0">
                                        <joint name="right_shoulder_roll_joint" type="hinge" axis="1 0 0" range="-0.3 1.5"/>
                                        <inertial pos="0 -0.05 0" mass="0.6" diaginertia="0.0008 0.0008 0.0005"/>
                                        <geom type="cylinder" size="0.03 0.04" pos="0 -0.05 0" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                                        <body name="right_upper_arm_link" pos="0 -0.08 0">
                                            <joint name="right_shoulder_yaw_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
                                            <inertial pos="0 -0.08 -0.08" mass="1.0" diaginertia="0.003 0.003 0.001"/>
                                            <geom type="box" size="0.025 0.025 0.08" pos="0 -0.08 -0.08" rgba="0.3 0.3 0.3 1"/>

                                            <body name="right_lower_arm_link" pos="0 -0.08 -0.16">
                                                <joint name="right_elbow_joint" type="hinge" axis="0 1 0" range="0 2.35"/>
                                                <inertial pos="0 0 -0.08" mass="0.8" diaginertia="0.002 0.002 0.0008"/>
                                                <geom type="box" size="0.02 0.02 0.08" pos="0 0 -0.08" rgba="0.5 0.5 0.5 1"/>

                                                <body name="right_wrist_yaw_link" pos="0 0 -0.16">
                                                    <joint name="right_wrist_yaw_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
                                                    <inertial pos="0 0 0" mass="0.2" diaginertia="0.0001 0.0001 0.0001"/>
                                                    <geom type="cylinder" size="0.025 0.015" rgba="0.3 0.3 0.3 1"/>

                                                    <body name="right_wrist_roll_link" pos="0 0 -0.02">
                                                        <joint name="right_wrist_roll_joint" type="hinge" axis="1 0 0" range="-1.57 1.57"/>
                                                        <inertial pos="0 0 0" mass="0.15" diaginertia="0.00008 0.00008 0.00005"/>
                                                        <geom type="cylinder" size="0.02 0.015" euler="1.57 0 0" rgba="0.5 0.5 0.5 1"/>

                                                        <body name="right_hand_link" pos="0 0 -0.02">
                                                            <joint name="right_wrist_pitch_joint" type="hinge" axis="0 1 0" range="-1.0 1.0"/>
                                                            <inertial pos="0 0 -0.04" mass="0.4" diaginertia="0.0005 0.0005 0.0003"/>
                                                            <geom type="box" size="0.03 0.04 0.04" pos="0 0 -0.04" rgba="0.1 0.1 0.1 1"/>
                                                        </body>
                                                    </body>
                                                </body>
                                            </body>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>

                    <!-- LEFT LEG -->
                    <body name="left_hip_yaw_link" pos="0 0.05 -0.05">
                        <joint name="left_hip_yaw_joint" type="hinge" axis="0 0 1" range="-0.43 0.43"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.002 0.002 0.001"/>
                        <geom type="cylinder" size="0.04 0.02" rgba="0.5 0.5 0.5 1"/>

                        <body name="left_hip_roll_link" pos="0 0.02 -0.02">
                            <joint name="left_hip_roll_joint" type="hinge" axis="1 0 0" range="-0.43 0.43"/>
                            <inertial pos="0 0.03 0" mass="0.8" diaginertia="0.0015 0.0015 0.0008"/>
                            <geom type="cylinder" size="0.035 0.03" pos="0 0.03 0" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                            <body name="left_thigh_link" pos="0 0.05 0">
                                <joint name="left_hip_pitch_joint" type="hinge" axis="0 1 0" range="-1.57 0.52"/>
                                <inertial pos="0 0 -0.15" mass="3.0" diaginertia="0.03 0.03 0.005"/>
                                <geom type="box" size="0.035 0.035 0.15" pos="0 0 -0.15" rgba="0.3 0.3 0.3 1"/>

                                <body name="left_calf_link" pos="0 0 -0.30">
                                    <joint name="left_knee_joint" type="hinge" axis="0 1 0" range="0 2.53"/>
                                    <inertial pos="0 0 -0.15" mass="2.0" diaginertia="0.02 0.02 0.003"/>
                                    <geom type="box" size="0.025 0.025 0.15" pos="0 0 -0.15" rgba="0.5 0.5 0.5 1"/>

                                    <body name="left_ankle_link" pos="0 0 -0.30">
                                        <joint name="left_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87 0.52"/>
                                        <inertial pos="0 0 0" mass="0.5" diaginertia="0.0003 0.0003 0.0002"/>
                                        <geom type="sphere" size="0.03" rgba="0.3 0.3 0.3 1"/>

                                        <body name="left_foot_link" pos="0 0 0">
                                            <joint name="left_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.26 0.26"/>
                                            <inertial pos="0.03 0 -0.02" mass="0.8" diaginertia="0.001 0.003 0.003"/>
                                            <geom type="box" size="0.09 0.04 0.02" pos="0.03 0 -0.02" rgba="0.1 0.1 0.1 1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>

                    <!-- RIGHT LEG -->
                    <body name="right_hip_yaw_link" pos="0 -0.05 -0.05">
                        <joint name="right_hip_yaw_joint" type="hinge" axis="0 0 1" range="-0.43 0.43"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.002 0.002 0.001"/>
                        <geom type="cylinder" size="0.04 0.02" rgba="0.5 0.5 0.5 1"/>

                        <body name="right_hip_roll_link" pos="0 -0.02 -0.02">
                            <joint name="right_hip_roll_joint" type="hinge" axis="1 0 0" range="-0.43 0.43"/>
                            <inertial pos="0 -0.03 0" mass="0.8" diaginertia="0.0015 0.0015 0.0008"/>
                            <geom type="cylinder" size="0.035 0.03" pos="0 -0.03 0" euler="1.57 0 0" rgba="0.3 0.3 0.3 1"/>

                            <body name="right_thigh_link" pos="0 -0.05 0">
                                <joint name="right_hip_pitch_joint" type="hinge" axis="0 1 0" range="-1.57 0.52"/>
                                <inertial pos="0 0 -0.15" mass="3.0" diaginertia="0.03 0.03 0.005"/>
                                <geom type="box" size="0.035 0.035 0.15" pos="0 0 -0.15" rgba="0.3 0.3 0.3 1"/>

                                <body name="right_calf_link" pos="0 0 -0.30">
                                    <joint name="right_knee_joint" type="hinge" axis="0 1 0" range="0 2.53"/>
                                    <inertial pos="0 0 -0.15" mass="2.0" diaginertia="0.02 0.02 0.003"/>
                                    <geom type="box" size="0.025 0.025 0.15" pos="0 0 -0.15" rgba="0.5 0.5 0.5 1"/>

                                    <body name="right_ankle_link" pos="0 0 -0.30">
                                        <joint name="right_ankle_pitch_joint" type="hinge" axis="0 1 0" range="-0.87 0.52"/>
                                        <inertial pos="0 0 0" mass="0.5" diaginertia="0.0003 0.0003 0.0002"/>
                                        <geom type="sphere" size="0.03" rgba="0.3 0.3 0.3 1"/>

                                        <body name="right_foot_link" pos="0 0 0">
                                            <joint name="right_ankle_roll_joint" type="hinge" axis="1 0 0" range="-0.26 0.26"/>
                                            <inertial pos="0.03 0 -0.02" mass="0.8" diaginertia="0.001 0.003 0.003"/>
                                            <geom type="box" size="0.09 0.04 0.02" pos="0.03 0 -0.02" rgba="0.1 0.1 0.1 1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </worldbody>

            <actuator>
                <!-- Waist - position servos -->
                <position name="waist_yaw_motor" joint="waist_yaw_joint" kp="200"/>
                <position name="waist_roll_motor" joint="waist_roll_joint" kp="200"/>
                <position name="waist_pitch_motor" joint="waist_pitch_joint" kp="200"/>

                <!-- Left Arm -->
                <position name="left_shoulder_pitch_motor" joint="left_shoulder_pitch_joint" kp="100"/>
                <position name="left_shoulder_roll_motor" joint="left_shoulder_roll_joint" kp="100"/>
                <position name="left_shoulder_yaw_motor" joint="left_shoulder_yaw_joint" kp="100"/>
                <position name="left_elbow_motor" joint="left_elbow_joint" kp="100"/>
                <position name="left_wrist_yaw_motor" joint="left_wrist_yaw_joint" kp="50"/>
                <position name="left_wrist_roll_motor" joint="left_wrist_roll_joint" kp="50"/>
                <position name="left_wrist_pitch_motor" joint="left_wrist_pitch_joint" kp="50"/>

                <!-- Right Arm -->
                <position name="right_shoulder_pitch_motor" joint="right_shoulder_pitch_joint" kp="100"/>
                <position name="right_shoulder_roll_motor" joint="right_shoulder_roll_joint" kp="100"/>
                <position name="right_shoulder_yaw_motor" joint="right_shoulder_yaw_joint" kp="100"/>
                <position name="right_elbow_motor" joint="right_elbow_joint" kp="100"/>
                <position name="right_wrist_yaw_motor" joint="right_wrist_yaw_joint" kp="50"/>
                <position name="right_wrist_roll_motor" joint="right_wrist_roll_joint" kp="50"/>
                <position name="right_wrist_pitch_motor" joint="right_wrist_pitch_joint" kp="50"/>

                <!-- Left Leg -->
                <position name="left_hip_yaw_motor" joint="left_hip_yaw_joint" kp="300"/>
                <position name="left_hip_roll_motor" joint="left_hip_roll_joint" kp="300"/>
                <position name="left_hip_pitch_motor" joint="left_hip_pitch_joint" kp="400"/>
                <position name="left_knee_motor" joint="left_knee_joint" kp="400"/>
                <position name="left_ankle_pitch_motor" joint="left_ankle_pitch_joint" kp="200"/>
                <position name="left_ankle_roll_motor" joint="left_ankle_roll_joint" kp="200"/>

                <!-- Right Leg -->
                <position name="right_hip_yaw_motor" joint="right_hip_yaw_joint" kp="300"/>
                <position name="right_hip_roll_motor" joint="right_hip_roll_joint" kp="300"/>
                <position name="right_hip_pitch_motor" joint="right_hip_pitch_joint" kp="400"/>
                <position name="right_knee_motor" joint="right_knee_joint" kp="400"/>
                <position name="right_ankle_pitch_motor" joint="right_ankle_pitch_joint" kp="200"/>
                <position name="right_ankle_roll_motor" joint="right_ankle_roll_joint" kp="200"/>
            </actuator>

            <sensor>
                <accelerometer name="imu_accel" site="imu_site"/>
                <gyro name="imu_gyro" site="imu_site"/>
            </sensor>
        </mujoco>
        """
        return mujoco.MjModel.from_xml_string(xml)

    def joint_command_callback(self, msg: Float64MultiArray):
        """Handle incoming joint commands."""
        with self.command_lock:
            if len(msg.data) == len(self.JOINT_NAMES):
                self.joint_commands = np.array(msg.data)
            else:
                self.get_logger().warn(
                    f'Received {len(msg.data)} commands, expected {len(self.JOINT_NAMES)}')

    def publish_state(self):
        """Publish current robot state to ROS topics."""
        now = self.get_clock().now().to_msg()

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.JOINT_NAMES

        # Get joint positions and velocities from MuJoCo
        # Skip the first 7 DOF (floating base: 3 pos + 4 quat)
        positions = []
        velocities = []
        efforts = []

        for joint_name in self.JOINT_NAMES:
            try:
                joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id >= 0:
                    qpos_adr = self.model.jnt_qposadr[joint_id]
                    qvel_adr = self.model.jnt_dofadr[joint_id]
                    positions.append(float(self.data.qpos[qpos_adr]))
                    velocities.append(float(self.data.qvel[qvel_adr]))
                    efforts.append(0.0)  # Could compute from actuator forces
                else:
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
            except Exception:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts

        self.joint_state_pub.publish(joint_state)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'base_link'

        if self.fixed_base:
            # Fixed base - identity orientation
            imu_msg.orientation.w = 1.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
        else:
            # Get base orientation from floating joint (quaternion)
            # MuJoCo stores quaternion as [w, x, y, z]
            quat = self.data.qpos[3:7]
            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])
            # Get angular velocity from floating joint
            imu_msg.angular_velocity.x = float(self.data.qvel[3])
            imu_msg.angular_velocity.y = float(self.data.qvel[4])
            imu_msg.angular_velocity.z = float(self.data.qvel[5])

        # Get linear acceleration from sensors if available
        try:
            accel_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_accel')
            if accel_id >= 0:
                accel_adr = self.model.sensor_adr[accel_id]
                imu_msg.linear_acceleration.x = float(self.data.sensordata[accel_adr])
                imu_msg.linear_acceleration.y = float(self.data.sensordata[accel_adr + 1])
                imu_msg.linear_acceleration.z = float(self.data.sensordata[accel_adr + 2])
        except Exception:
            pass

        self.imu_pub.publish(imu_msg)

        # Publish TF: world -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        if self.fixed_base:
            # Fixed base at known position
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.75  # Standing height
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
        else:
            quat = self.data.qpos[3:7]
            t.transform.translation.x = float(self.data.qpos[0])
            t.transform.translation.y = float(self.data.qpos[1])
            t.transform.translation.z = float(self.data.qpos[2])
            t.transform.rotation.w = float(quat[0])
            t.transform.rotation.x = float(quat[1])
            t.transform.rotation.y = float(quat[2])
            t.transform.rotation.z = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def simulation_step(self):
        """Single simulation step - called by timer."""
        # Apply joint commands (PD position control)
        with self.command_lock:
            self._apply_control()

        # Step simulation
        mujoco.mj_step(self.model, self.data)

        # Sync viewer if active
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def start_viewer(self):
        """Start the MuJoCo viewer in a separate thread."""
        if self.use_viewer:
            self.get_logger().info('Starting MuJoCo viewer...')
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def stop(self):
        """Stop the simulation."""
        if self.viewer is not None:
            self.viewer.close()


def main(args=None):
    rclpy.init(args=args)

    node = G1MuJoCoSimulator()

    # Start viewer if enabled
    node.start_viewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
