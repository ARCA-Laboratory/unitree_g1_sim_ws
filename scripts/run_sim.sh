#!/bin/bash
# ==============================================================================
# Quick Start Script for Unitree G1 Simulation
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  sim       - Run basic simulation with MuJoCo viewer and RViz"
    echo "  demo      - Run simulation with demo motions (wave by default)"
    echo "  wave      - Run simulation with wave demo"
    echo "  squat     - Run simulation with squat demo"
    echo "  teleop    - Run simulation (then start teleop separately)"
    echo "  rviz      - Launch only RViz for visualization"
    echo "  headless  - Run simulation without any GUI"
    echo ""
    echo "Examples:"
    echo "  $0 sim"
    echo "  $0 demo wave"
    echo ""
}

# Source environment
source_env() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS 2 Humble not found"
        exit 1
    fi

    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    else
        echo -e "${YELLOW}Workspace not built. Building now...${NC}"
        cd "$WORKSPACE_DIR"
        colcon build --symlink-install
        source "$WORKSPACE_DIR/install/setup.bash"
    fi

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

# Main
main() {
    source_env

    case "${1:-sim}" in
        sim)
            echo -e "${GREEN}Starting G1 simulation...${NC}"
            ros2 launch g1_sim_bringup g1_sim.launch.py
            ;;
        demo)
            MODE="${2:-wave}"
            echo -e "${GREEN}Starting G1 demo (mode: $MODE)...${NC}"
            ros2 launch g1_sim_bringup g1_demo.launch.py mode:="$MODE"
            ;;
        wave)
            echo -e "${GREEN}Starting G1 wave demo...${NC}"
            ros2 launch g1_sim_bringup g1_demo.launch.py mode:=wave
            ;;
        squat)
            echo -e "${GREEN}Starting G1 squat demo...${NC}"
            ros2 launch g1_sim_bringup g1_demo.launch.py mode:=squat
            ;;
        teleop)
            echo -e "${GREEN}Starting G1 simulation...${NC}"
            echo -e "${BLUE}After simulation starts, run in another terminal:${NC}"
            echo "  ros2 run g1_sim_bridge keyboard_teleop"
            ros2 launch g1_sim_bringup g1_sim.launch.py
            ;;
        rviz)
            echo -e "${GREEN}Starting RViz only...${NC}"
            ros2 launch g1_description display.launch.py
            ;;
        headless)
            echo -e "${GREEN}Starting headless simulation...${NC}"
            ros2 launch g1_sim_bringup g1_sim.launch.py use_viewer:=false use_rviz:=false
            ;;
        -h|--help|help)
            print_usage
            ;;
        *)
            echo "Unknown command: $1"
            print_usage
            exit 1
            ;;
    esac
}

main "$@"
