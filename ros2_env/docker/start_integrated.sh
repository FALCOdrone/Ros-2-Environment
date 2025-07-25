#!/bin/bash

# Enable X11 forwarding for GUI applications
xhost +local:docker

# Create Xauth file if it doesn't exist
export XAUTH=/tmp/.docker.xauth
if [ ! -f ${XAUTH} ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f ${XAUTH} nmerge -
    else
        touch ${XAUTH}
    fi
    chmod a+r ${XAUTH}
fi

# Check if Docker image exists, if not build it
if [[ "$(docker images -q ros2-env-with-sjtu:humble 2> /dev/null)" == "" ]]; then
    echo "Docker image not found. Building ros2-env-with-sjtu:humble..."
    cd .. && ./build_docker.sh
    if [ $? -ne 0 ]; then
        echo "❌ Failed to build Docker image. Please check the build output above."
        exit 1
    fi
    cd docker
fi

echo "Starting ROS2 environment with sjtu_drone integration..."
echo "Available packages:"
echo "  - falco_drone (your existing package)"
echo "  - sjtu_drone (integrated quadrotor simulation)"
echo ""
echo "Usage examples:"
echo "  1. Launch sjtu_drone simulation:"
echo "     ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py"
echo ""
echo "  2. Launch falco_drone simulation:"
echo "     ros2 launch falco_drone gazebo_simple_controllable.launch.py"
echo ""
echo "  3. Control sjtu_drone:"
echo "     ros2 topic pub /drone/takeoff std_msgs/msg/Empty {} --once"
echo ""

# Start the container with docker run
docker run -it --rm \
    --name ros2_with_sjtu_drone \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws:/home/robotics/ros2_ws \
    ros2-env-with-sjtu:humble \
    /bin/bash