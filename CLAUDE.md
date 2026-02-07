# CLAUDE.md - Development Guide

## Rules for Claude

1. **Always commit and push changes** - After making code changes, Claude should add, commit, and push to the repository (unless the user specifies otherwise).
2. **Use descriptive commit messages** - Summarize what was changed and why.
3. **Test before committing** - Ensure changes work before pushing.
4. **Update documentation** - Keep README.md and CLAUDE.md up to date with any significant changes.

## Project Overview

**TidyBot2** - A mobile robot platform with bimanual WX250s 6-DOF arms for Stanford's Collaborative Robotics class. This repository provides MuJoCo simulation and ROS2 control software.

### Robot Configuration
- **Mobile Base:** 3 DOF (x, y, theta)
- **Arms:** 2x WX250s (6 DOF each: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
- **Grippers:** 2x Interbotix gripper (prismatic fingers)
- **Camera:** Pan-tilt RealSense D435 (RGB + Depth)

## Quick Start (Ubuntu 22.04)

### Prerequisites

#### 1. Install ROS2 Humble
```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional ROS2 packages
sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher ros-humble-rviz2
```

#### 2. Install uv (Python package manager)
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc  # or restart terminal
```

#### 3. Install System Dependencies
```bash
sudo apt install -y python3-colcon-common-extensions python3-pip \
    libgl1-mesa-dev libglfw3-dev libegl1-mesa-dev
```

### Setup

```bash
cd /home/arm-beast/Desktop/collab/collaborative-robotics-2026

# Install Python dependencies
uv sync

# Build ROS2 workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Running the Code

### Option 1: Standalone MuJoCo Simulation (No ROS2)
```bash
cd simulation/scripts

# Bimanual arm demo with camera
uv run python test_move.py

# Object manipulation demo
uv run python pick_up_block.py
```

### Option 2: Full ROS2 Simulation

**Terminal 1 - Launch simulation:**
```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

**Terminal 2 - Run test scripts:**
```bash
cd ros2_ws
source setup_env.bash

# Test base movement
ros2 run tidybot_bringup test_base_sim.py

# Test bimanual arms
ros2 run tidybot_bringup test_arms_sim.py

# Test camera pan-tilt
ros2 run tidybot_bringup test_camera_sim.py

# Advanced state machine example
ros2 run tidybot_bringup example_state_machine.py
```

**Launch Options:**
```bash
# Disable RViz
ros2 launch tidybot_bringup sim.launch.py use_rviz:=false

# Disable MuJoCo viewer
ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=false
```

### Option 3: Real Hardware (Local)

**On the robot mini PC:**
```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup real.launch.py
```

### Option 4: Remote Control (Network)

Control TidyBot2 from your own machine over the network using native ROS2.

**Step 1: Start robot (on mini PC via SSH):**
```bash
ssh locobot@<ROBOT_IP>
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/collaborative-robotics-2026/ros2_ws
source install/setup.bash
ros2 launch tidybot_bringup robot.launch.py
```

**Step 2: Connect from your machine:**
```bash
# Install Cyclone DDS (one time)
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Set environment (must match robot!)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build tidybot_msgs on your machine (one time)
cd ~/ros2_ws/src
git clone https://github.com/Stanford-ARM-Lab/collaborative-robotics-2026.git
cd ~/ros2_ws && colcon build --packages-select tidybot_msgs
source install/setup.bash

# Test connection
ros2 topic list  # Should see /cmd_vel, /joint_states, etc.
```

**Step 3: Control the robot:**
```bash
# Move base forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10

# Run comprehensive test
ros2 run tidybot_bringup test_real_hardware.py

# Launch RViz remotely
ros2 launch tidybot_client client_rviz.launch.py
```

See [docs/remote_setup.md](docs/remote_setup.md) for detailed network configuration.

## Project Structure

```
collaborative-robotics-2026/
├── simulation/              # Standalone MuJoCo simulation
│   ├── scripts/             # Test scripts (test_move.py, pick_up_block.py)
│   └── assets/              # MuJoCo models and meshes
│
├── docs/
│   └── remote_setup.md      # Remote control setup guide
│
└── ros2_ws/                 # ROS2 workspace
    ├── setup_env.bash       # Environment setup (source this!)
    └── src/
        ├── tidybot_bringup/         # Launch files & test scripts
        ├── tidybot_client/          # Remote client utilities & DDS configs
        ├── tidybot_control/         # Arm/base/pan-tilt controllers
        ├── tidybot_description/     # URDF/XACRO robot model
        ├── tidybot_ik/              # Motion planning & IK
        ├── tidybot_msgs/            # Custom messages & services
        ├── tidybot_mujoco_bridge/   # MuJoCo-ROS2 bridge (simulation)
        └── tidybot_network_bridge/  # Image compression for remote
```

## Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Base velocity commands |
| `/left_arm/command` | ArmCommand | Left arm joint positions + duration |
| `/right_arm/command` | ArmCommand | Right arm joint positions + duration |
| `/left_gripper/command` | GripperCommand | Left gripper (0=open, 1=closed) |
| `/right_gripper/command` | GripperCommand | Right gripper |
| `/camera/pan_tilt` | PanTilt | Camera pan/tilt angles |
| `/joint_states` | sensor_msgs/JointState | All joint positions/velocities |
| `/camera/rgb/image_raw` | sensor_msgs/Image | RGB camera feed |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth camera feed |

## Custom Message Types

**ArmCommand.msg** - Joint space control:
```
float64[6] joint_positions  # [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
float64 duration            # Movement time in seconds
```

**GripperCommand.msg**:
```
float64 position  # 0.0 (open) to 1.0 (closed)
float64 effort    # 0.0 (low) to 1.0 (high force)
```

## WX250s Arm Joint Limits

| Joint | Min (rad) | Max (rad) | Range (deg) |
|-------|-----------|-----------|-------------|
| Waist | -3.14 | 3.14 | 360° |
| Shoulder | -1.88 | 1.99 | 222° |
| Elbow | -2.15 | 1.61 | 215° |
| Forearm Roll | -3.14 | 3.14 | 360° |
| Wrist Angle | -1.75 | 2.15 | 223° |
| Wrist Rotate | -3.14 | 3.14 | 360° |

## Common Development Commands

```bash
# Rebuild after code changes
cd ros2_ws && colcon build

# Rebuild specific package
colcon build --packages-select tidybot_control

# Check ROS2 topics
ros2 topic list
ros2 topic echo /joint_states

# Check available services
ros2 service list
```

## Troubleshooting

**MuJoCo rendering issues:**
```bash
# Ensure OpenGL is working
sudo apt install -y mesa-utils
glxinfo | grep "OpenGL version"
```

**Python import errors:**
```bash
# Always source the environment first
source ros2_ws/setup_env.bash
```

**colcon build fails:**
```bash
# Ensure ROS2 is sourced before building
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build
```

## Dependencies

Managed via `uv` (see `pyproject.toml`):
- mujoco >= 3.2.0
- numpy >= 1.26.0
- opencv-python >= 4.0.0
- mink >= 0.0.6 (trajectory optimization)
- pyrealsense2 >= 2.55.0

## Docker

Build and run with VNC desktop access:

```bash
# 1. Clone the repo (if you haven't already)
git clone https://github.com/Stanford-ARM-Lab/collaborative-robotics-2026.git
cd collaborative-robotics-2026

# 2. Build the Docker image
docker build -f Docker/Dockerfile -t tidybot2:humble .

# 3. Run with the repo mounted (changes sync between host and container)
docker run -p 6080:80 --shm-size=2g \
    -v $(pwd):/home/ubuntu/Desktop/collaborative \
    tidybot2:humble

# Access via browser: http://127.0.0.1:6080/
```

**Syncing updates:** With the volume mount, any `git pull` on your host machine will immediately reflect inside the running container. This makes it easy to pull the latest changes without restarting the container.

**Available commands in container:**
| Command | Description |
|---------|-------------|
| `tidybot-sim` | MuJoCo standalone simulation |
| `tidybot-ros` | ROS2 + RViz + MuJoCo |
| `tidybot-ros-no-rviz` | ROS2 + MuJoCo (no RViz) |
| `tidybot-test-base` | Test base movement |
| `tidybot-test-arms` | Test arm control |

## Real Hardware Setup

### Motor ID Configuration (Shared U2D2 Bus)

If using a **single U2D2** for all Dynamixel motors (both arms + pan-tilt), you must remap motor IDs to avoid conflicts:

| Component | Default IDs | Remapped IDs |
|-----------|-------------|--------------|
| Right Arm | 1-6, gripper 7 | 1-6, gripper 7 (keep default) |
| Left Arm | 1-6, gripper 7 | 11-16, gripper 17 |
| Pan-Tilt | 1-2 | 21-22 |

**How to remap motor IDs using Dynamixel Wizard 2.0:**
1. Connect **only ONE motor** at a time to a U2D2
2. Open Dynamixel Wizard 2.0 and scan for the motor
3. Select the motor and go to the Control Table
4. Find `ID` (address 7) and change it to the new value
5. Click "Save" - the motor will now respond to the new ID
6. Repeat for each motor that needs remapping

**If using separate U2D2s** (one per arm + one for pan-tilt), you can keep default IDs.

### Phoenix 6 Mobile Base

The mobile base uses Phoenix 6 (CTRE) motors over CAN bus. Prerequisites:
- Phoenix 6 library: `pip install phoenix6`
- Ruckig trajectory library: `pip install ruckig`
- CAN interface configured on mini PC
- Real-time permissions (edit `/etc/security/limits.d/99-realtime.conf`)

### Environment Variables

Add to `~/.bashrc` on the robot mini PC:
```bash
export TIDYBOT2_PATH=/home/locobot/tidybot2
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Authors

Alex Qiu & Matt Strong - Stanford ARM Lab
For Professor Monroe Kennedy's Collaborative Robotics Class (2026)
