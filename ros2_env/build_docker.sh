#!/bin/bash

# Build script for ROS2 Docker container with sjtu_drone integration

# Set the build context to the parent directory so we can access ros2_ws
cd /home/lorenzo/Ros-2-Environment/ros2_env

# Build the Docker image
echo "Building ROS2 Docker container with sjtu_drone integration..."
docker build -f docker/Dockerfile -t ros2-env-with-sjtu:humble .

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully!"
    echo "You can now run the container using:"
    echo "  docker run -it --rm ros2-env-with-sjtu:humble"
    echo ""
    echo "Or use the docker-compose setup for full GUI support."
else
    echo "❌ Docker build failed. Please check the error messages above."
    exit 1
fi
