#!/bin/bash
# filepath: /home/lorenzo/Ros-2-Environment/drone_control_ws/launch_gazebo_x500_auto.sh

# Enhanced Gazebo launcher with automatic x500 drone spawning
# Usage: ./launch_gazebo_x500_auto.sh [auto_spawn] [x] [y] [z]
# Example: ./launch_gazebo_x500_auto.sh true 5 5 2

set -e

# Configuration
AUTO_SPAWN=${1:-true}
SPAWN_X=${2:-0}
SPAWN_Y=${3:-0}
SPAWN_Z=${4:-1}
PX4_SOURCE_PATH="/opt/px4_source"

echo "=== Enhanced Gazebo X500 Launcher ==="
echo "Auto-spawn: $AUTO_SPAWN"
if [ "$AUTO_SPAWN" = "true" ]; then
    echo "Spawn position: ($SPAWN_X, $SPAWN_Y, $SPAWN_Z)"
fi

# Check if PX4 source exists
if [ ! -d "$PX4_SOURCE_PATH" ]; then
    echo "ERROR: PX4 source not found at $PX4_SOURCE_PATH"
    echo "Please ensure PX4-Autopilot is installed or update PX4_SOURCE_PATH"
    exit 1
fi

# Check if Gazebo is installed
if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo (gz) command not found"
    echo "Please install Gazebo Garden: sudo apt install gz-garden"
    exit 1
fi

# Source ROS 2 environment
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source install/setup.bash
fi

# Set up Gazebo environment variables for PX4 models
echo "Setting up Gazebo environment..."
export GZ_SIM_RESOURCE_PATH="$PX4_SOURCE_PATH/Tools/simulation/gz/models:$PX4_SOURCE_PATH/Tools/simulation/gz/worlds"
export PX4_GZ_MODELS="$PX4_SOURCE_PATH/Tools/simulation/gz/models"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down Gazebo..."
    pkill -f "gz sim" || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "Starting Gazebo Garden with default world..."
echo "Available models in: $PX4_GZ_MODELS"

# Start Gazebo in background
gz sim -v 4 -r "$PX4_SOURCE_PATH/Tools/simulation/gz/worlds/default.sdf" &
GZ_PID=$!

if [ "$AUTO_SPAWN" = "true" ]; then
    echo "Waiting for Gazebo to initialize..."
    sleep 8
    
    echo "Spawning x500 drone at position ($SPAWN_X, $SPAWN_Y, $SPAWN_Z)..."
    
    # Spawn the x500 model
    gz service -s /world/default/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf_filename: \"x500\", pose: { position: { x: $SPAWN_X, y: $SPAWN_Y, z: $SPAWN_Z } }"
    
    if [ $? -eq 0 ]; then
        echo "✅ X500 drone spawned successfully!"
        echo ""
        echo "🚁 Drone Control Information:"
        echo "   - Model: x500 quadcopter"
        echo "   - Position: ($SPAWN_X, $SPAWN_Y, $SPAWN_Z)"
        echo "   - You can now control it via ROS 2 topics (without PX4)"
        echo ""
        echo "📡 Available ROS 2 Topics for Direct Control:"
        echo "   - Motor control: /model/x500/joint/{rotor_0,rotor_1,rotor_2,rotor_3}/cmd_thrust"
        echo "   - Sensor data: /model/x500/odometry, /model/x500/imu"
        echo "   - Camera: /model/x500/camera/image"
        echo ""
        echo "💡 To control without PX4:"
        echo "   1. Publish thrust commands directly to motor topics"
        echo "   2. Implement your own flight controller as ROS 2 nodes"
        echo "   3. Use /gz/msgs/Double for motor commands"
    else
        echo "❌ Failed to spawn x500 drone"
        echo "You can manually spawn it later using Gazebo GUI"
    fi
else
    echo ""
    echo "Manual spawn mode - use Gazebo GUI to add the x500 model"
    echo "Available in: Insert -> x500"
fi

echo ""
echo "🎮 Controls:"
echo "   - Press Ctrl+C to shutdown"
echo "   - Use Gazebo GUI for manual model interaction"
echo "   - Use ROS 2 topics for programmatic control"
echo ""

# Wait for Gazebo process
wait $GZ_PID