#!/bin/bash
# Setup script for TidyBot2 ROS2 workspace
# This combines the ROS2 environment with the uv-managed packages
#
# Usage: source setup_env.bash
#
# NOTE: We DON'T activate the venv (which would change the python binary).
# Instead, we add the venv's site-packages to PYTHONPATH so ROS2's Python
# can import packages like mujoco, mink, numpy, etc.

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Setting up TidyBot2 ROS2 environment..."
echo ""

# 0. Deactivate any active venv and remove .venv from PATH
#    This ensures colcon uses system Python, not venv Python
if [ -n "$VIRTUAL_ENV" ]; then
    deactivate 2>/dev/null
    echo "✓ Deactivated existing virtual environment"
fi
# Remove any .venv/bin from PATH (in case it was added manually)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "\.venv/bin" | tr '\n' ':' | sed 's/:$//')

# 1. Source ROS2 (detect Humble or Jazzy)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    ROS_DISTRO="jazzy"
    PYTHON_VERSION="3.12"
    echo "✓ Sourced ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO="humble"
    PYTHON_VERSION="3.10"
    echo "✓ Sourced ROS2 Humble"
else
    echo "✗ ROS2 not found (checked Humble and Jazzy)"
    echo "  Install with: sudo apt install ros-humble-desktop  (Ubuntu 22.04)"
    echo "            or: sudo apt install ros-jazzy-desktop   (Ubuntu 24.04)"
    return 1
fi

# 2. Add uv venv site-packages to PYTHONPATH (for mujoco, mink, numpy, etc.)
#    This lets ROS2's Python import your uv-managed packages WITHOUT
#    changing which python binary is used (which would break colcon build)
UV_SITE_PACKAGES="$PROJECT_ROOT/.venv/lib/python$PYTHON_VERSION/site-packages"
if [ -d "$UV_SITE_PACKAGES" ]; then
    export PYTHONPATH="$UV_SITE_PACKAGES:$PYTHONPATH"
    echo "✓ Added uv packages to PYTHONPATH"
else
    echo "⚠ uv environment not found, running 'uv sync'..."
    (cd "$PROJECT_ROOT" && uv sync)
    if [ -d "$UV_SITE_PACKAGES" ]; then
        export PYTHONPATH="$UV_SITE_PACKAGES:$PYTHONPATH"
        echo "✓ Created uv environment and added to PYTHONPATH"
    else
        echo "✗ Failed to create uv environment"
        echo "  Try running manually: cd $PROJECT_ROOT && uv sync"
    fi
fi

# 3. Set repo root for finding simulation assets
export TIDYBOT_REPO_ROOT="$PROJECT_ROOT"
echo "✓ Set TIDYBOT_REPO_ROOT=$PROJECT_ROOT"

# 3.5. Set tidybot2 path for Phoenix 6 base controller
export TIDYBOT2_PATH="/home/locobot/tidybot2"
echo "✓ Set TIDYBOT2_PATH=$TIDYBOT2_PATH"

# 4. Build and source the ROS2 workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "✓ Sourced ROS2 workspace (ros2_ws)"
else
    echo "⚠ Workspace not built yet, running 'colcon build'..."
    (cd "$SCRIPT_DIR" && colcon build)
    if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
        source "$SCRIPT_DIR/install/setup.bash"
        echo "✓ Built and sourced ROS2 workspace (ros2_ws)"
    else
        echo "✗ Failed to build workspace"
        echo "  Try running manually: cd $SCRIPT_DIR && colcon build"
        return 1
    fi
fi

export ROS_DOMAIN_ID=42 # domain id for the robot
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # DDS implementation
echo "✓ Set ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "✓ Set RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

echo ""
echo "Environment ready!"
echo "  Python: $(which python3) (ROS2 system Python)"
echo "  uv packages available via PYTHONPATH"
echo ""
