#!/bin/bash
# ==============================================================================
# Build Script for Unitree G1 Simulation Workspace
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Building G1 Simulation Workspace${NC}"
echo "Workspace: $WORKSPACE_DIR"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found at /opt/ros/humble"
    echo "Please install ROS 2 Humble first."
    exit 1
fi

cd "$WORKSPACE_DIR"

# Build
echo -e "${BLUE}Running colcon build...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo -e "${GREEN}Build complete!${NC}"
echo ""
echo "To use the workspace, run:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo ""
echo "Then start the simulation with:"
echo "  ros2 launch g1_sim_bringup g1_sim.launch.py"
echo ""
echo "Or use the quick-start script:"
echo "  ./scripts/run_sim.sh sim"
