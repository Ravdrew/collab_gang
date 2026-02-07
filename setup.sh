#!/bin/bash
# =============================================================================
# TidyBot2 Setup Script
# Collaborative Robotics 2026 - Stanford ARM Lab
# =============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory (repo root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Detect Ubuntu version and set ROS2 distro
ROS_DISTRO="humble"  # default
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" == "24.04" ]; then
        ROS_DISTRO="jazzy"
    elif [ "$VERSION_ID" == "22.04" ]; then
        ROS_DISTRO="humble"
    fi
fi

echo -e "${BLUE}"
echo "============================================================================="
echo "  TidyBot2 Setup Script - Collaborative Robotics 2026"
echo "  Stanford ARM Lab"
echo "============================================================================="
echo -e "${NC}"

# =============================================================================
# Helper Functions
# =============================================================================

print_status() {
    echo -e "${GREEN}[*]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

ask_yes_no() {
    while true; do
        read -p "$1 [y/n]: " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Please answer y or n.";;
        esac
    done
}

# =============================================================================
# Check Ubuntu Version
# =============================================================================

check_ubuntu() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$ID" != "ubuntu" ]; then
            print_warning "This script is designed for Ubuntu"
            print_warning "Detected: $PRETTY_NAME"
            if ! ask_yes_no "Continue anyway?"; then
                exit 1
            fi
        elif [ "$VERSION_ID" != "22.04" ] && [ "$VERSION_ID" != "24.04" ]; then
            print_warning "This script is designed for Ubuntu 22.04 or 24.04"
            print_warning "Detected: $PRETTY_NAME"
            if ! ask_yes_no "Continue anyway?"; then
                exit 1
            fi
        else
            print_status "Detected Ubuntu $VERSION_ID - will use ROS2 $ROS_DISTRO"
        fi
    fi
}

# =============================================================================
# Install System Dependencies
# =============================================================================

install_system_deps() {
    print_status "Installing system dependencies..."
    sudo apt update
    sudo apt install -y \
        curl \
        git \
        python3-pip \
        libgl1-mesa-dev \
        libglfw3-dev \
        libegl1-mesa-dev \
        mesa-utils \
        software-properties-common \
        locales

    # Set locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
}

# =============================================================================
# Install ROS2 Humble
# =============================================================================

check_ros2_installed() {
    if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
        return 0
    else
        return 1
    fi
}

install_ros2() {
    print_status "Installing ROS2 $ROS_DISTRO..."

    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-desktop

    # Update again before installing additional packages
    sudo apt update

    # Install additional ROS2 packages
    sudo apt install -y \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-joint-state-publisher \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rosidl-typesupport-c \
        ros-$ROS_DISTRO-rosidl-typesupport-cpp \
        ros-$ROS_DISTRO-dynamixel-sdk \
        python3-colcon-common-extensions

    print_status "ROS2 $ROS_DISTRO installed successfully!"
}

# =============================================================================
# Install uv (Python Package Manager)
# =============================================================================

install_uv() {
    if command -v uv &> /dev/null; then
        print_status "uv is already installed"
    else
        print_status "Installing uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.cargo/bin:$PATH"
    fi
}

# =============================================================================
# Setup Python Environment
# =============================================================================

setup_python_env() {
    print_status "Setting up Python environment with uv..."
    cd "$SCRIPT_DIR"

    # Remove old venv if exists
    if [ -d ".venv" ]; then
        print_warning "Removing existing .venv..."
        rm -rf .venv
    fi

    # Create new venv and install dependencies
    uv sync

    print_status "Python environment ready!"
}

# =============================================================================
# Build ROS2 Workspace
# =============================================================================

build_ros2_workspace() {
    print_status "Building ROS2 workspace..."

    cd "$SCRIPT_DIR/ros2_ws"

    # Clean previous builds
    rm -rf build install log

    # Ensure we're using system Python (not conda)
    if [ -n "$CONDA_PREFIX" ]; then
        print_warning "Conda detected. Deactivating for ROS2 build..."
        conda deactivate 2>/dev/null || true
    fi

    # Source ROS2
    source /opt/ros/$ROS_DISTRO/setup.bash

    # Ensure dynamixel-sdk is installed (required for interbotix packages)
    sudo apt install -y ros-$ROS_DISTRO-dynamixel-sdk

    # Build
    colcon build

    print_status "ROS2 workspace built successfully!"
}

# =============================================================================
# Run Options
# =============================================================================

run_standalone_sim() {
    print_status "Starting standalone MuJoCo simulation..."
    cd "$SCRIPT_DIR/simulation/scripts"
    uv run python test_move.py
}

run_ros2_sim() {
    print_status "Starting ROS2 simulation..."
    cd "$SCRIPT_DIR/ros2_ws"

    # Export display settings
    export QT_QPA_PLATFORM=xcb
    export DISPLAY=${DISPLAY:-:0}

    source setup_env.bash
    ros2 launch tidybot_bringup sim.launch.py
}

run_ros2_sim_no_rviz() {
    print_status "Starting ROS2 simulation (MuJoCo only, no RViz)..."
    cd "$SCRIPT_DIR/ros2_ws"

    source setup_env.bash
    ros2 launch tidybot_bringup sim.launch.py use_rviz:=false
}

# =============================================================================
# Main Menu
# =============================================================================

show_main_menu() {
    echo ""
    echo -e "${BLUE}=============================================================================${NC}"
    echo -e "${BLUE}  TidyBot2 - Main Menu${NC}"
    echo -e "${BLUE}=============================================================================${NC}"
    echo ""
    echo "  1) Full Installation (System deps + ROS2 + Python env + Build)"
    echo "  2) Install without ROS2 (Standalone simulation only)"
    echo "  3) Build ROS2 workspace only (assumes deps installed)"
    echo "  4) Run standalone MuJoCo simulation"
    echo "  5) Run ROS2 simulation (with RViz)"
    echo "  6) Run ROS2 simulation (MuJoCo only, no RViz)"
    echo "  7) Exit"
    echo ""
    read -p "Select option [1-7]: " choice

    case $choice in
        1)
            check_ubuntu
            install_system_deps
            if check_ros2_installed; then
                print_status "ROS2 $ROS_DISTRO already installed"
                if ask_yes_no "Reinstall ROS2?"; then
                    install_ros2
                fi
            else
                install_ros2
            fi
            install_uv
            setup_python_env
            build_ros2_workspace
            print_status "Full installation complete!"
            echo ""
            if ask_yes_no "Run simulation now?"; then
                show_run_menu
            fi
            ;;
        2)
            check_ubuntu
            install_system_deps
            install_uv
            setup_python_env
            print_status "Standalone installation complete!"
            echo ""
            if ask_yes_no "Run standalone simulation now?"; then
                run_standalone_sim
            fi
            ;;
        3)
            if ! check_ros2_installed; then
                print_error "ROS2 $ROS_DISTRO not found. Please run full installation first."
                exit 1
            fi
            build_ros2_workspace
            ;;
        4)
            run_standalone_sim
            ;;
        5)
            run_ros2_sim
            ;;
        6)
            run_ros2_sim_no_rviz
            ;;
        7)
            echo "Goodbye!"
            exit 0
            ;;
        *)
            print_error "Invalid option"
            show_main_menu
            ;;
    esac
}

show_run_menu() {
    echo ""
    echo "Which simulation would you like to run?"
    echo "  1) Standalone MuJoCo simulation"
    echo "  2) ROS2 simulation (with RViz)"
    echo "  3) ROS2 simulation (MuJoCo only)"
    echo "  4) Cancel"
    echo ""
    read -p "Select option [1-4]: " run_choice

    case $run_choice in
        1) run_standalone_sim ;;
        2) run_ros2_sim ;;
        3) run_ros2_sim_no_rviz ;;
        4) echo "Cancelled" ;;
        *) print_error "Invalid option" ;;
    esac
}

# =============================================================================
# Entry Point
# =============================================================================

# Check if running with arguments
if [ $# -gt 0 ]; then
    case $1 in
        --install)
            check_ubuntu
            install_system_deps
            if ! check_ros2_installed; then
                install_ros2
            fi
            install_uv
            setup_python_env
            build_ros2_workspace
            ;;
        --install-no-ros)
            check_ubuntu
            install_system_deps
            install_uv
            setup_python_env
            ;;
        --build)
            build_ros2_workspace
            ;;
        --run-standalone)
            run_standalone_sim
            ;;
        --run-ros2)
            run_ros2_sim
            ;;
        --run-ros2-no-rviz)
            run_ros2_sim_no_rviz
            ;;
        --help|-h)
            echo "Usage: $0 [OPTION]"
            echo ""
            echo "Options:"
            echo "  --install          Full installation (deps + ROS2 + build)"
            echo "  --install-no-ros   Install without ROS2 (standalone only)"
            echo "  --build            Build ROS2 workspace only"
            echo "  --run-standalone   Run standalone MuJoCo simulation"
            echo "  --run-ros2         Run ROS2 simulation with RViz"
            echo "  --run-ros2-no-rviz Run ROS2 simulation without RViz"
            echo "  --help, -h         Show this help message"
            echo ""
            echo "Without arguments, shows interactive menu."
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
else
    show_main_menu
fi
