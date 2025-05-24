#!/bin/bash

xacro quadrotor.urdf.xacro id:=0 > qws.urdf
ros2 run gazebo_ros spawn_entity.py -file qws.urdf -entity quadrotor_0 -x -2.5 -y 2.5 -z 0.15
