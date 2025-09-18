#!/bin/bash

echo "Building ROS2 environment with falco_drone..."
cd docker
docker-compose build ros2_env
if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully!"
else
    echo "❌ Failed to build Docker image"
    exit 1
fi
