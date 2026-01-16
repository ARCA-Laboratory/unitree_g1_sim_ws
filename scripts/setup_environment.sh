#!/bin/bash
# ==============================================================================
# Unitree G1 Humanoid Simulation Environment Setup Script
# ==============================================================================
# This script sets up the complete simulation environment for the Unitree G1
# humanoid robot with ROS 2 integration using MuJoCo physics engine.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$WORKSPACE_DIR/src"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}============================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}============================================================${NC}"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# ==============================================================================
# Check Prerequisites
# ==============================================================================
check_prerequisites() {
    print_header "Checking Prerequisites"

    # Check Ubuntu version
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        print_info "OS: $NAME $VERSION_ID"
    fi

    # Check if running with sudo capabilities
    if ! sudo -v; then
        print_error "This script requires sudo privileges"
        exit 1
    fi

    # Check Python version
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
        print_info "Python version: $PYTHON_VERSION"
    else
        print_error "Python3 is required but not installed"
        exit 1
    fi

    print_success "Prerequisites check passed"
}

# ==============================================================================
# Install System Dependencies
# ==============================================================================
install_system_deps() {
    print_header "Installing System Dependencies"

    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
        libxxf86vm-dev \
        libosmesa6-dev \
        python3-pip \
        python3-venv \
        python3-numpy \
        libboost-all-dev \
        libeigen3-dev \
        libglfw3-dev \
        libyaml-cpp-dev

    print_success "System dependencies installed"
}

# ==============================================================================
# Install ROS 2 Humble
# ==============================================================================
install_ros2() {
    print_header "Installing ROS 2 Humble"

    # Check if ROS 2 is already installed
    if [ -f /opt/ros/humble/setup.bash ]; then
        print_info "ROS 2 Humble already installed"
        return 0
    fi

    # Set locale
    sudo apt-get update && sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Setup sources
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository universe -y

    sudo apt-get update && sudo apt-get install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2
    sudo apt-get update
    sudo apt-get install -y ros-humble-desktop

    # Install additional ROS 2 packages
    sudo apt-get install -y \
        ros-humble-ros-base \
        ros-humble-rviz2 \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-cyclonedds \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-rosdep

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update || true

    print_success "ROS 2 Humble installed"
}

# ==============================================================================
# Install MuJoCo
# ==============================================================================
install_mujoco() {
    print_header "Installing MuJoCo"

    # Install MuJoCo via pip (modern approach)
    pip3 install --user mujoco mujoco-python-viewer

    # Also install gymnasium for RL environments
    pip3 install --user gymnasium

    print_success "MuJoCo installed"
}

# ==============================================================================
# Install Unitree SDK2 and Python bindings
# ==============================================================================
install_unitree_sdk() {
    print_header "Installing Unitree SDK2"

    cd "$SRC_DIR"

    # Clone unitree_sdk2 if not exists
    if [ ! -d "$SRC_DIR/unitree_sdk2" ]; then
        git clone https://github.com/unitreerobotics/unitree_sdk2.git
        print_info "Cloned unitree_sdk2"
    else
        print_info "unitree_sdk2 already exists, pulling latest"
        cd "$SRC_DIR/unitree_sdk2" && git pull || true
    fi

    # Note: unitree_sdk2py is not on PyPI and needs to be built from source
    # It's optional for MuJoCo simulation - only needed for real robot control
    print_info "unitree_sdk2 cloned (Python SDK build is optional for simulation)"

    print_success "Unitree SDK2 installed"
}

# ==============================================================================
# Clone Unitree MuJoCo Simulation
# ==============================================================================
clone_unitree_mujoco() {
    print_header "Setting up Unitree MuJoCo Simulation"

    cd "$SRC_DIR"

    # Clone unitree_mujoco
    if [ ! -d "$SRC_DIR/unitree_mujoco" ]; then
        git clone https://github.com/unitreerobotics/unitree_mujoco.git
        print_info "Cloned unitree_mujoco"
    else
        print_info "unitree_mujoco already exists, pulling latest"
        cd "$SRC_DIR/unitree_mujoco" && git pull || true
    fi

    # Install Python dependencies for unitree_mujoco
    cd "$SRC_DIR/unitree_mujoco"
    if [ -f "requirements.txt" ]; then
        pip3 install --user -r requirements.txt
    fi

    # Install additional dependencies
    pip3 install --user pygame pyyaml

    print_success "Unitree MuJoCo simulation setup complete"
}

# ==============================================================================
# Clone Unitree ROS 2 packages
# ==============================================================================
clone_unitree_ros2() {
    print_header "Setting up Unitree ROS 2 Packages"

    cd "$SRC_DIR"

    # Clone unitree_ros (contains g1_description)
    if [ ! -d "$SRC_DIR/unitree_ros" ]; then
        git clone https://github.com/unitreerobotics/unitree_ros.git
        print_info "Cloned unitree_ros"
    else
        print_info "unitree_ros already exists, pulling latest"
        cd "$SRC_DIR/unitree_ros" && git pull || true
    fi

    # Clone unitree_ros2
    if [ ! -d "$SRC_DIR/unitree_ros2" ]; then
        git clone https://github.com/unitreerobotics/unitree_ros2.git
        print_info "Cloned unitree_ros2"
    else
        print_info "unitree_ros2 already exists, pulling latest"
        cd "$SRC_DIR/unitree_ros2" && git pull || true
    fi

    print_success "Unitree ROS 2 packages setup complete"
}

# ==============================================================================
# Build ROS 2 Workspace
# ==============================================================================
build_ros2_workspace() {
    print_header "Building ROS 2 Workspace"

    cd "$WORKSPACE_DIR"

    # Source ROS 2
    source /opt/ros/humble/setup.bash

    # Build workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    print_success "ROS 2 workspace built"
}

# ==============================================================================
# Setup Environment Variables
# ==============================================================================
setup_environment() {
    print_header "Setting up Environment Variables"

    # Create environment setup script
    cat > "$WORKSPACE_DIR/setup.bash" << 'EOF'
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
EOF

    chmod +x "$WORKSPACE_DIR/setup.bash"

    # Add to bashrc if not already present
    if ! grep -q "unitree_g1_sim_ws/setup.bash" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# Unitree G1 Simulation Environment" >> ~/.bashrc
        echo "# Uncomment to auto-source on terminal open:" >> ~/.bashrc
        echo "# source $WORKSPACE_DIR/setup.bash" >> ~/.bashrc
        print_info "Added workspace setup to ~/.bashrc (commented out)"
    fi

    print_success "Environment setup complete"
}

# ==============================================================================
# Main Installation
# ==============================================================================
main() {
    print_header "Unitree G1 Simulation Environment Setup"
    echo "Workspace: $WORKSPACE_DIR"
    echo ""

    check_prerequisites
    install_system_deps
    install_ros2
    install_mujoco
    install_unitree_sdk
    clone_unitree_mujoco
    clone_unitree_ros2
    setup_environment

    # Don't build ROS 2 workspace here - we'll do it after creating our packages

    print_header "Installation Complete!"
    echo ""
    echo "Next steps:"
    echo "  1. Source the environment: source $WORKSPACE_DIR/setup.bash"
    echo "  2. Build the workspace: cd $WORKSPACE_DIR && colcon build"
    echo "  3. Run the simulation: ros2 launch g1_sim_bringup g1_sim.launch.py"
    echo ""
    print_success "Setup complete!"
}

# Run main function
main "$@"
