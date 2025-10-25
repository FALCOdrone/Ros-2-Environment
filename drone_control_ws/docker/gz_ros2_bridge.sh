#!/bin/bash

# Gazebo-ROS2 Bridge Script for Drone Control
# This script bridges Gazebo topics to ROS2 topics

echo "Starting Gazebo-ROS2 Bridge..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Start the bridge with topic mappings
ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/default/model/x500_enhanced/model/x500/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure \
    /world/default/model/x500_enhanced/model/x500/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    /world/default/model/x500_enhanced/link/gps_link/sensor/gps_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
    /world/default/model/x500_enhanced/link/camera_left_link/sensor/camera_left/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /world/default/model/x500_enhanced/link/camera_right_link/sensor/camera_right/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /world/default/model/x500_enhanced/link/camera_left_link/sensor/camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    /world/default/model/x500_enhanced/link/camera_right_link/sensor/camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    /world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    /world/default/stats@rosgraph_msgs/msg/Clock[gz.msgs.WorldStatistics \
    /gui/camera/pose@geometry_msgs/msg/Pose[gz.msgs.Pose \
    --ros-args -r __node:=gz_ros2_bridge

echo "Gazebo-ROS2 Bridge started successfully!"