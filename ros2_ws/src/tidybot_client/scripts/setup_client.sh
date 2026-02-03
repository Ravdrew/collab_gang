#!/bin/bash
#
# TidyBot2 Client Setup Script
#
# This script configures the ROS2 environment on a client machine to connect
# to a remote TidyBot2 robot over the network.
#
# Usage:
#   source setup_client.sh [ROBOT_IP] [DOMAIN_ID]
#
# Examples:
#   source setup_client.sh                    # Use defaults (multicast discovery, domain 42)
#   source setup_client.sh 192.168.1.100      # Specify robot IP for discovery server
#   source setup_client.sh 192.168.1.100 99   # Specify IP and domain ID
#
# After running this script:
#   ros2 topic list              # Should show robot topics
#   ros2 topic echo /joint_states  # Should show joint data
#

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== TidyBot2 Client Setup ===${NC}"

# Get arguments
ROBOT_IP=${1:-""}
DOMAIN_ID=${2:-42}

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=$DOMAIN_ID
echo -e "ROS_DOMAIN_ID=${GREEN}$DOMAIN_ID${NC}"

# Check for Cyclone DDS
if [ -f /opt/ros/humble/share/rmw_cyclonedds_cpp/local_setup.bash ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo -e "RMW_IMPLEMENTATION=${GREEN}rmw_cyclonedds_cpp${NC}"

    # Find config file location
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    CONFIG_FILE="$SCRIPT_DIR/../config/cyclone_dds_client.xml"

    if [ -f "$CONFIG_FILE" ]; then
        export CYCLONEDDS_URI="file://$CONFIG_FILE"
        echo -e "CYCLONEDDS_URI=${GREEN}$CONFIG_FILE${NC}"
    else
        echo -e "${YELLOW}Warning: Cyclone DDS config not found at $CONFIG_FILE${NC}"
        echo "Using default Cyclone DDS configuration"
    fi
else
    # Fall back to FastDDS
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    echo -e "RMW_IMPLEMENTATION=${GREEN}rmw_fastrtps_cpp${NC}"

    if [ -n "$ROBOT_IP" ]; then
        echo -e "${YELLOW}Note: For FastDDS with discovery server, configure fastdds_client.xml${NC}"
        echo "Robot IP: $ROBOT_IP"
    fi
fi

echo ""
echo -e "${GREEN}Client environment configured!${NC}"
echo ""
echo "Test connection with:"
echo "  ros2 topic list"
echo ""
echo "If no topics appear, check:"
echo "  1. Robot is running (robot.launch.py)"
echo "  2. Same network/subnet as robot"
echo "  3. Multicast is not blocked (or use discovery server)"
echo "  4. ROS_DOMAIN_ID matches robot"
echo ""

# Test discovery (optional)
if command -v ros2 &> /dev/null; then
    echo -e "Attempting discovery... (timeout 5s)"
    timeout 5 ros2 topic list 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Discovery successful! Topics found.${NC}"
    else
        echo -e "${YELLOW}No topics found. Robot may not be running or network issue.${NC}"
    fi
fi
