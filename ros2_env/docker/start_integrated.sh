#!/bin/bash

# Enable X11 forwarding for GUI applications
xhost +local:docker

# Create Xauth file if it doesn't exist
export XAUTH=/tmp/.docker.xauth
if [ ! -f ${XAUTH} ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/' 2>/dev/null)
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f ${XAUTH} nmerge - 2>/dev/null
    else
        touch ${XAUTH} 2>/dev/null
    fi
    chmod a+r ${XAUTH} 2>/dev/null || true
fi

# Check if Docker image exists, if not build it
if [[ "$(docker images -q lorenzo195815/ros2_env:latest 2> /dev/null)" == "" ]]; then
    echo "Docker image not found. Building lorenzo195815/ros2_env:latest..."
    ./build.sh
    if [ $? -ne 0 ]; then
        echo "❌ Failed to build Docker image. Please check the build output above."
        exit 1
    fi
fi

echo "Starting ROS2 environment with falco_drone integration..."
echo "Available packages:"
echo "  - falco_drone_bringup (launch files)"
echo "  - falco_drone_control (control nodes)" 
echo "  - falco_drone_description (URDF/SDF models)"
echo ""
echo "Usage examples:"
echo "  1. Launch falco_drone simulation:"
echo "     ros2 launch falco_drone_bringup falco_drone_bringup.launch.py"
echo ""
echo "  2. Launch falco_drone Gazebo simulation:"
echo "     ros2 launch falco_drone_bringup falco_drone_gazebo.launch.py"
echo ""
echo "  3. Control falco_drone:"
echo "     ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty {} --once"
echo ""

# Start the container with docker run
docker run -it --rm \
    --name ros2_with_falco_drone \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/lorenzo/Ros-2-Environment/ros2_env/ros2_ws:/home/robotics/ros2_ws \
    lorenzo195815/ros2_env:latest \
    /bin/bash