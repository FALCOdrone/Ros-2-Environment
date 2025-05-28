#!/bin/bash
# Clear old URDF
rm -f qws.urdf

# Generate new URDF with proper xacro processing
xacro --inorder quadrotor.urdf.xacro id:=0 > qws.urdf

# Verify the file
if [ ! -s qws.urdf ]; then
    echo "Error: Generated URDF is empty!"
    exit 1
fi

# Spawn with error handling
ros2 run gazebo_ros spawn_entity.py \
    -file qws.urdf \
    -entity quadrotor_0 \
    -x -2.5 -y 2.5 -z 0.15