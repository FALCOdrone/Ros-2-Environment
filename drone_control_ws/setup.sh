#!/bin/bash

# ROS2 Drone Control Environment Setup Script
# This script sets up the development environment for the drone control system

set -e

echo "=== ROS2 Drone Control Environment Setup ==="

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 is not installed. Please install ROS2 Humble first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

echo "✓ ROS2 environment sourced"

# Check if workspace exists
if [ ! -d "src" ]; then
    echo "ERROR: This script must be run from the workspace root directory"
    echo "Expected structure: drone_control_ws/src/"
    exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy scipy matplotlib

# Build the workspace
echo "Building ROS2 workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "✓ Workspace built successfully"
else
    echo "ERROR: Workspace build failed"
    exit 1
fi

# Source the workspace
source install/setup.bash

echo "✓ Workspace sourced"

# Create setup script for future use
cat > setup_environment.sh << 'EOF'
#!/bin/bash
# Quick setup script for drone control environment
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/drone_description/share/drone_description
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)/install/drone_description/share/drone_description
echo "ROS2 Drone Control Environment Ready!"
echo "Launch complete system: ros2 launch drone_simulation drone_complete.launch.py"
EOF

chmod +x setup_environment.sh

echo "✓ Setup script created: ./setup_environment.sh"

# Test basic functionality
echo "Testing node discovery..."
timeout 5s ros2 pkg list | grep -E "(drone_control|drone_description|drone_sensors|drone_simulation)" > /dev/null

if [ $? -eq 0 ]; then
    echo "✓ All drone packages found"
else
    echo "WARNING: Some packages may not be properly installed"
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Source the environment: source ./setup_environment.sh"
echo "2. Launch the simulation: ros2 launch drone_simulation drone_complete.launch.py"
echo "3. Open another terminal and monitor: ros2 topic list"
echo ""
echo "Troubleshooting:"
echo "- If Gazebo fails to start, try: killall gzserver gzclient"
echo "- Check README.md for detailed usage instructions"
echo "- Monitor logs: ros2 log list"
