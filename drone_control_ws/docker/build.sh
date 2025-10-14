#!/bin/bash

# Build script for ROS2 Humble + Gazebo Ignition Docker container

set -e

# Configuration
IMAGE_NAME="drone_control_ros2_humble"
TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${TAG}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Building ROS2 Humble + Gazebo Container${NC}"
echo -e "${BLUE}========================================${NC}"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker first.${NC}"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${YELLOW}Workspace directory: ${WORKSPACE_DIR}${NC}"
echo -e "${YELLOW}Docker context: ${SCRIPT_DIR}${NC}"

# Change to the workspace directory (parent of docker folder)
cd "$WORKSPACE_DIR"

# Build the Docker image
echo -e "${BLUE}Building Docker image: ${FULL_IMAGE_NAME}${NC}"
echo -e "${YELLOW}This may take several minutes...${NC}"

docker build \
    --file docker/Dockerfile.minimal \
    --tag "$FULL_IMAGE_NAME" \
    --build-arg BUILDKIT_INLINE_CACHE=1 \
    .

# Check if build was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Build completed successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Image name: ${FULL_IMAGE_NAME}${NC}"
    echo -e "${YELLOW}To run the container, execute: ./docker/start.sh${NC}"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Build failed!${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi

# Show image size
echo -e "${BLUE}Image information:${NC}"
docker images "$FULL_IMAGE_NAME" --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}"

echo -e "${GREEN}Build process completed!${NC}"
