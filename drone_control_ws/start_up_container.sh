#!/bin/bash

# ====================================
# Building the image if it does not exist
# ====================================

if [[ "$(docker images -q drone_control_ros2_gz 2> /dev/null)" == "" ]]; then
     echo "Building the drone_control_ros2_gz docker image..."
     cd ~/Ros-2-Environment/drone_control_ws
     docker build -t drone_control_ros2_gz -f docker/Dockerfile .
else
     echo "drone_control_ros2_gz docker image already exists. Skipping build."
fiadded a sturtup container bash file

# ====================================
# Run the docker container:
# ====================================

# Start the container in detached mode if not already running
if [ "$(docker ps -q -f name=drone_control_container)" ]; then
    echo "drone_control_container is already running. Skipping startup."
else
    docker-compose up -d drone-control
fi

# Attach to the running container
docker exec -it drone_control_container bash