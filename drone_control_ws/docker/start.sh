#!/bin/bash

# Start script for ROS2 Humble + Gazebo Ignition Docker container

set -e

# Configuration
IMAGE_NAME="drone_control_ros2_humble"
TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${TAG}"
CONTAINER_NAME="drone_control_container"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Starting ROS2 Humble + Gazebo Container${NC}"
echo -e "${BLUE}========================================${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker first.${NC}"
    exit 1
fi

# Check if image exists
if ! docker image inspect "$FULL_IMAGE_NAME" > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker image ${FULL_IMAGE_NAME} not found.${NC}"
    echo -e "${YELLOW}Please build the image first using: ./docker/build.sh${NC}"
    exit 1
fi

# Stop and remove existing container if it exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}Stopping and removing existing container: ${CONTAINER_NAME}${NC}"
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Detect if we're on Linux with X11 or macOS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux - Enable X11 forwarding for GUI applications
    DISPLAY_ARGS="--env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw"
    
    # Allow X11 connections
    xhost +local:docker > /dev/null 2>&1 || echo -e "${YELLOW}Warning: Could not configure X11 permissions${NC}"
    
elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS - Use host networking for X11 (requires XQuartz)
    DISPLAY_ARGS="--env DISPLAY=host.docker.internal:0"
    echo -e "${YELLOW}Note: For GUI applications on macOS, make sure XQuartz is running${NC}"
    echo -e "${YELLOW}and 'Allow connections from network clients' is enabled in XQuartz preferences${NC}"
else
    # Other systems
    DISPLAY_ARGS=""
    echo -e "${YELLOW}Warning: GUI applications may not work on this system${NC}"
fi

# Start the container
echo -e "${GREEN}Starting container: ${CONTAINER_NAME}${NC}"
echo -e "${YELLOW}Workspace mounted at: /workspace${NC}"

docker run -it \
    --name "$CONTAINER_NAME" \
    --hostname drone-control \
    --privileged \
    --network host \
    $DISPLAY_ARGS \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=0 \
    --volume "$WORKSPACE_DIR:/workspace" \
    --volume /dev:/dev \
    --workdir /workspace \
    "$FULL_IMAGE_NAME" \
    bash

echo -e "${GREEN}Container session ended.${NC}"

# Clean up X11 permissions on Linux
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xhost -local:docker > /dev/null 2>&1 || true
fi
