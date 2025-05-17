#!/bin/bash

# Specify the container name and image
CONTAINER_NAME="Falco_container"
IMAGE_NAME="lorenzo195815/ros2_env:latest"

# Pull the latest image
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME

# Check if the container exists
if docker ps -a --format "{{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f "{{.State.Running}}" $CONTAINER_NAME 2>/dev/null)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

# Ensure the local 'data' and 'ros2_ws' folders exist
PWD_DIR=$(pwd)
echo "pw: $PWD_DIR "
DATA_FOLDER="$PWD_DIR/../data"
ROS2_WS_FOLDER="$PWD_DIR/../ros2_ws"

mkdir -p "$DATA_FOLDER"
mkdir -p "$ROS2_WS_FOLDER"

# Run the container
docker run -it \
    --user robotics \
    --env="DISPLAY=novnc:0.0" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=ros \
    --rm \
    --volume="$DATA_FOLDER:/home/robotics/data" \
    --volume="$ROS2_WS_FOLDER:/home/robotics/ros2_ws" \
    --name "$CONTAINER_NAME" \
    -w /home/robotics \
    "$IMAGE_NAME"