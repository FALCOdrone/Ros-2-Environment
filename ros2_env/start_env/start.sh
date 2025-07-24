#!/bin/bash

# Specify the container name and image
CONTAINER_NAME="Falco_container"
IMAGE_NAME="lorenzo195815/ros2_env:latest"

# Check if the image exists
echo "Checking if Docker image $IMAGE_NAME exists..."
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "Image not found. Building image..."
    cd "$(dirname "$0")/../docker"
    chmod +x build.sh
    ./build.sh
    cd - > /dev/null
else
    echo "Image $IMAGE_NAME already exists. Skipping build."
fi

# Check if the container exists
if docker ps -a --format "{{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping and removing..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running. Removing..."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

# Ensure the local 'data' and 'ros2_ws' folders exist
PWD_DIR=$(pwd)
echo "pw: $PWD_DIR"
DATA_FOLDER="$PWD_DIR/../data"
ROS2_WS_FOLDER="$PWD_DIR/../ros2_ws"

mkdir -p "$DATA_FOLDER"
mkdir -p "$ROS2_WS_FOLDER"

# Create the Docker network if it doesn't exist
docker network create ros 2>/dev/null || true

# Run the container
docker run -it \
    --platform linux/amd64 \
    --user robotics \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=ros \
    --rm \
    --volume="$DATA_FOLDER:/home/robotics/data" \
    --volume="$ROS2_WS_FOLDER:/home/robotics/ros2_ws" \
    --name "$CONTAINER_NAME" \
    -w /home/robotics \
    "$IMAGE_NAME"