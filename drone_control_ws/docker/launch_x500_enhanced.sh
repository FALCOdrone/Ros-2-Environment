#!/bin/bash

# Launch script for enhanced X500 with GPS and stereo cameras
set -e

echo "Starting enhanced X500 with GPS and stereo cameras..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set up Gazebo environment
export GZ_SIM_RESOURCE_PATH=/opt/px4_source/Tools/simulation/gz/models:/opt/px4_source/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=/opt/px4_source/Tools/simulation/gz/models

# Start Gazebo with default world
echo "Starting Gazebo..."
gz sim -v 4 -r /opt/px4_source/Tools/simulation/gz/worlds/default.sdf &
GZ_PID=$!

# Wait for Gazebo to initialize
sleep 8

# Spawn the enhanced x500 model
echo "Spawning enhanced x500 with GPS and stereo cameras..."
gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"x500_enhanced\", pose: { position: { x: 0, y: 0, z: 1 } }"

if [ $? -eq 0 ]; then
    echo "✅ Enhanced X500 spawned successfully!"
    echo ""
    echo "🚁 Enhanced Drone Features:"
    echo "   - GPS/NavSat sensor for position"
    echo "   - Stereo cameras (left/right) for visual odometry"
    echo "   - IMU and pressure sensors (from base x500)"
    echo ""
    echo "📡 Available sensor topics:"
    echo "   - GPS: /world/default/model/x500_enhanced/link/gps_link/sensor/gps_sensor/navsat"
    echo "   - Left camera: /world/default/model/x500_enhanced/link/camera_left_link/sensor/camera_left/image"
    echo "   - Right camera: /world/default/model/x500_enhanced/link/camera_right_link/sensor/camera_right/image"
    echo "   - IMU: /world/default/model/x500_enhanced/link/base_link/sensor/imu_sensor/imu"
    echo "   - Pressure: /world/default/model/x500_enhanced/link/base_link/sensor/air_pressure_sensor/air_pressure"
else
    echo "❌ Failed to spawn enhanced x500"
fi

# Keep Gazebo running
wait $GZ_PID
