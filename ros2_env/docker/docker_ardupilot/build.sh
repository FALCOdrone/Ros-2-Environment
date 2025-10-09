#!/bin/bash

# Build ArduPilot Docker image with better network configuration
echo "Building ArduPilot SITL with ROS2 and MAVROS..."

# Use host network for better connectivity during build
DOCKER_BUILDKIT=1 docker build \
    --network=host \
    --add-host=github.com:140.82.113.4 \
    -t ardupilot-sim .
