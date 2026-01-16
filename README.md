# Unitree G1 Humanoid Robot Simulation

A complete simulation environment for the **Unitree G1 humanoid robot** with **ROS 2 Humble** integration using **MuJoCo** physics engine.

## Overview

This workspace provides:

- 🤖 **MuJoCo-based physics simulation** of the 29-DOF Unitree G1 humanoid
- 🔗 **ROS 2 bridge** publishing joint states, IMU data, and TF transforms
- 🎮 **Multiple control interfaces**: keyboard teleop, programmatic control, demo motions
- 📊 **RViz2 visualization** for real-time robot state monitoring
- 📝 **Complete URDF/Xacro** robot description

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Ecosystem                          │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐    │
│  │   RViz2      │   │  Your Node   │   │  Keyboard    │    │
│  │ Visualization│   │  (Control)   │   │  Teleop      │    │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘    │
│         │                  │                  │            │
│    /joint_states     /joint_commands    /joint_commands    │
│         │                  │                  │            │
│  ┌──────┴──────────────────┴──────────────────┴───────┐    │
│  │              MuJoCo-ROS2 Bridge                    │    │
│  │  - Publishes: /joint_states, /imu/data, /tf       │    │
│  │  - Subscribes: /joint_commands                     │    │
│  └────────────────────────┬───────────────────────────┘    │
└───────────────────────────┼─────────────────────────────────┘
                            │
┌───────────────────────────┼─────────────────────────────────┐
│                           ▼                                 │
│  ┌────────────────────────────────────────────────────┐    │
│  │              MuJoCo Physics Engine                 │    │
│  │  - 500 Hz simulation                               │    │
│  │  - 29-DOF G1 humanoid model                       │    │
│  │  - Contact dynamics, IMU simulation               │    │
│  └────────────────────────────────────────────────────┘    │
│                    MuJoCo Viewer (optional)                 │
└─────────────────────────────────────────────────────────────┘
```

## Robot Configuration

The G1 humanoid has **29 degrees of freedom**:

| Body Part   | Joints | DOF |
|-------------|--------|-----|
| Waist       | yaw, roll, pitch | 3 |
| Left Arm    | shoulder (3), elbow, wrist (3) | 7 |
| Right Arm   | shoulder (3), elbow, wrist (3) | 7 |
| Left Leg    | hip (3), knee, ankle (2) | 6 |
| Right Leg   | hip (3), knee, ankle (2) | 6 |
| **Total**   | | **29** |

## Prerequisites

- **Ubuntu 22.04** (recommended)
- **ROS 2 Humble**
- **Python 3.10+**
- **MuJoCo 3.x** (installed via pip)

## Quick Start

### 1. Install Dependencies

```bash
cd ~/unitree_g1_sim_ws

# Option A: Run the full setup script (installs ROS 2, MuJoCo, etc.)
./scripts/setup_environment.sh

# Option B: Manual install (if ROS 2 Humble is already installed)
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-rmw-cyclonedds-cpp
pip3 install --user mujoco gymnasium pyyaml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. Build the Workspace

```bash
./scripts/build.sh
# Or manually:
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select g1_description g1_sim_bridge g1_sim_bringup
```

### 3. Run the Simulation

```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Option A: Use the helper script
./scripts/run_sim.sh sim

# Option B: Use ros2 launch directly
ros2 launch g1_sim_bringup g1_sim.launch.py
```

## Usage

### Running Different Modes

```bash
# Basic simulation with MuJoCo viewer + RViz
./scripts/run_sim.sh sim

# Demo with wave motion
./scripts/run_sim.sh wave

# Demo with squat motion
./scripts/run_sim.sh squat

# Headless mode (no GUI)
./scripts/run_sim.sh headless

# RViz only (with joint_state_publisher_gui)
./scripts/run_sim.sh rviz
```

### Keyboard Teleoperation

In one terminal, start the simulation:
```bash
ros2 launch g1_sim_bringup g1_sim.launch.py
```

In another terminal, start the teleop:
```bash
ros2 run g1_sim_bridge keyboard_teleop
```

**Teleop Controls:**
| Key | Action |
|-----|--------|
| q/a | Left shoulder pitch up/down |
| w/s | Left elbow flex/extend |
| e/d | Left wrist up/down |
| u/j | Right shoulder pitch up/down |
| i/k | Right elbow flex/extend |
| o/l | Right wrist up/down |
| z/x | Waist yaw left/right |
| r   | Reset to standing pose |
| +/- | Increase/decrease step size |
| ESC | Quit |

### Programmatic Control

Send joint commands via the `/joint_commands` topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class G1Controller(Node):
    def __init__(self):
        super().__init__('g1_controller')
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)

    def send_command(self, positions):
        """Send 29-element array of joint positions."""
        msg = Float64MultiArray()
        msg.data = positions
        self.cmd_pub.publish(msg)
```

### Launch File Options

```bash
# Full simulation
ros2 launch g1_sim_bringup g1_sim.launch.py

# With arguments
ros2 launch g1_sim_bringup g1_sim.launch.py \
    use_rviz:=true \
    use_viewer:=true \
    use_sim_time:=true

# Demo mode
ros2 launch g1_sim_bringup g1_demo.launch.py mode:=wave
# Available modes: standing, wave, squat, arms_up
```

## ROS 2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions, velocities |
| `/imu/data` | `sensor_msgs/Imu` | IMU orientation, angular velocity, linear acceleration |
| `/tf` | `tf2_msgs/TFMessage` | Transform from world to base_link |
| `/robot_description` | `std_msgs/String` | URDF robot description |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_commands` | `std_msgs/Float64MultiArray` | Target joint positions (29 elements) |

## Package Structure

```
unitree_g1_sim_ws/
├── README.md
├── setup.bash                    # Environment setup
├── scripts/
│   ├── setup_environment.sh      # Full installation script
│   ├── build.sh                  # Build workspace
│   └── run_sim.sh                # Quick-start runner
├── src/
│   ├── g1_description/           # Robot description package
│   │   ├── urdf/g1.urdf.xacro    # Robot URDF
│   │   ├── launch/display.launch.py
│   │   └── rviz/g1_display.rviz
│   ├── g1_sim_bringup/           # Simulation bringup
│   │   ├── launch/
│   │   │   ├── g1_sim.launch.py
│   │   │   ├── g1_demo.launch.py
│   │   │   └── g1_teleop.launch.py
│   │   └── config/g1_sim_params.yaml
│   └── g1_sim_bridge/            # MuJoCo-ROS2 bridge
│       ├── scripts/
│       │   ├── mujoco_ros2_bridge.py
│       │   ├── joint_command_publisher.py
│       │   └── keyboard_teleop.py
│       └── config/joint_names.yaml
├── config/
└── docs/
```

## Using Official Unitree MuJoCo Models

For more accurate simulation, you can use Unitree's official MuJoCo models:

```bash
# Clone the official unitree_mujoco repository
cd ~/unitree_g1_sim_ws/src
git clone https://github.com/unitreerobotics/unitree_mujoco.git

# The G1 model is at: unitree_mujoco/unitree_robots/g1/
```

Then launch with the official model:
```bash
ros2 run g1_sim_bridge mujoco_ros2_bridge \
    --ros-args -p model_path:=/path/to/g1.xml
```

## Troubleshooting

### MuJoCo Viewer Not Showing

```bash
# Ensure GLFW is working
export MUJOCO_GL=glfw

# For headless servers, use EGL
export MUJOCO_GL=egl
```

### ROS 2 Communication Issues

```bash
# Use CycloneDDS (matches Unitree's real robot)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Build Errors

```bash
# Make sure ROS 2 is sourced first
source /opt/ros/humble/setup.bash

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

## References

- [Unitree G1 Official Page](https://www.unitree.com/g1/)
- [unitree_mujoco GitHub](https://github.com/unitreerobotics/unitree_mujoco)
- [unitree_ros2 GitHub](https://github.com/unitreerobotics/unitree_ros2)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

## License

This simulation package is released under the BSD-3-Clause license.

The Unitree G1 robot design and official software are property of Unitree Robotics.
