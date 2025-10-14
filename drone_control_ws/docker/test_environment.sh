#!/bin/bash

# Test script to verify the Docker environment setup

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Testing ROS2 + Gazebo Docker Environment${NC}"
echo -e "${BLUE}========================================${NC}"

# Test ROS2 installation
echo -e "${YELLOW}Testing ROS2 installation...${NC}"
if ros2 --help > /dev/null 2>&1; then
    echo -e "${GREEN}✓ ROS2 is installed and working${NC}"
    echo -e "${BLUE}ROS2 version:${NC}"
    ros2 --version
else
    echo -e "${RED}✗ ROS2 is not working properly${NC}"
    exit 1
fi

# Test Gazebo installation
echo -e "${YELLOW}Testing Gazebo Ignition installation...${NC}"
if gz --help > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Gazebo Ignition is installed and working${NC}"
    echo -e "${BLUE}Gazebo version:${NC}"
    gz --version
else
    echo -e "${RED}✗ Gazebo Ignition is not working properly${NC}"
    exit 1
fi

# Test Python packages
echo -e "${YELLOW}Testing Python packages...${NC}"
python3 -c "import numpy; print('NumPy version:', numpy.__version__)" && echo -e "${GREEN}✓ NumPy${NC}"
python3 -c "import matplotlib; print('Matplotlib version:', matplotlib.__version__)" && echo -e "${GREEN}✓ Matplotlib${NC}"
python3 -c "import scipy; print('SciPy version:', scipy.__version__)" && echo -e "${GREEN}✓ SciPy${NC}"

# Test workspace build
echo -e "${YELLOW}Testing workspace build...${NC}"
if [ -d "/workspace/install" ]; then
    echo -e "${GREEN}✓ Workspace is built${NC}"
    echo -e "${BLUE}Available packages:${NC}"
    ls /workspace/install/ | grep -v "_local_setup\|COLCON_IGNORE\|local_setup\|setup"
else
    echo -e "${YELLOW}⚠ Workspace not built yet. Building now...${NC}"
    cd /workspace
    colcon build --symlink-install
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Workspace built successfully${NC}"
    else
        echo -e "${RED}✗ Workspace build failed${NC}"
        exit 1
    fi
fi

# Test ROS2 topics
echo -e "${YELLOW}Testing ROS2 communication...${NC}"
timeout 5 ros2 topic list > /dev/null 2>&1 && echo -e "${GREEN}✓ ROS2 communication working${NC}" || echo -e "${YELLOW}⚠ ROS2 daemon might not be running${NC}"

# Display environment info
echo -e "${BLUE}Environment Information:${NC}"
echo -e "ROS_DISTRO: ${ROS_DISTRO:-Not set}"
echo -e "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-Not set}"
echo -e "GAZEBO_VERSION: ${GAZEBO_VERSION:-Not set}"
echo -e "Workspace: $(pwd)"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Environment test completed!${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "${YELLOW}Quick start commands:${NC}"
echo -e "  ${BLUE}ros2 launch drone_simulation drone_simulation.launch.py${NC}  # Full simulation"
echo -e "  ${BLUE}ros2 launch drone_description display.launch.py${NC}            # View in RViz"
echo -e "  ${BLUE}rviz2${NC}                                                       # Open RViz"
echo -e "  ${BLUE}gz sim${NC}                                                      # Open Gazebo"
