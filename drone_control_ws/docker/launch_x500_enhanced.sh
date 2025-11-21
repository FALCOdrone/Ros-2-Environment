#!/bin/bash
# Launch script for enhanced X500 with GPS and stereo cameras
set -e

echo "Starting enhanced X500 with GPS and stereo cameras..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source Gazebo Fortress/GZ
export GZ_VERSION=fortress

# Set up Gazebo environment (add local workspace models)
export GZ_SIM_RESOURCE_PATH="/workspace/models:/opt/px4_source/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PX4_GZ_MODELS=/opt/px4_source/Tools/simulation/gz/models

# Start Gazebo with default world
echo "Starting Gazebo..."
gz sim -v 4 -r /opt/px4_source/Tools/simulation/gz/worlds/default.sdf &
GZ_PID=$!

# Wait for Gazebo to initialize
sleep 8

# Ensure the create service is available
echo "Waiting for /world/default/create service..."
while ! gz service -l | grep -q "/world/default/create"; do
    sleep 0.5
done

# Absolute path to your model SDF
MODEL_PATH="/workspace/models/x500_enhanced/model.sdf"

# Spawn the enhanced x500 model
echo "Spawning enhanced x500 with GPS and stereo cameras..."
gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"$MODEL_PATH\" pose { position { x: 0 y: 0 z: 1 } }"

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
