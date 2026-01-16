#!/bin/bash
# Unitree G1 Simulation Environment Setup

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# Set ROS 2 to use CycloneDDS (same as Unitree robots)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set domain ID (default 0, change if needed)
export ROS_DOMAIN_ID=0

# Add workspace scripts to PATH
export PATH="$SCRIPT_DIR/scripts:$PATH"

# Add Python user packages to PATH
export PATH="$HOME/.local/bin:$PATH"

# Set MUJOCO_GL for rendering
export MUJOCO_GL=glfw

echo "Unitree G1 simulation environment loaded!"
echo "Workspace: $SCRIPT_DIR"
