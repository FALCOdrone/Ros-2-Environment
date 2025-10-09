#!/bin/bash

# Enable X11 forwarding for GUI support
xhost +local:docker

# Run ArduPilot container with X11 support
docker run -it --rm \
    -p 5760:5760 \
    -p 5761:5761 \
    -p 5762:5762 \
    -p 5763:5763 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    --network=host \
    ardupilot-sim

# Clean up X11 permissions after container stops
xhost -local:docker
