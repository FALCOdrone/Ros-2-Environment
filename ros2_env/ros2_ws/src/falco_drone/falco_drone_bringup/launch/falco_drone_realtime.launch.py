#!/usr/bin/env python3
"""
Launch file for real-time drone operation
Separates real hardware from simulation to avoid conflicts
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Real-time drone configuration
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")  # No sim time for real drone
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for real-time operation",
        ),

        # Hardware Bridge Node - connects to Teensy
        Node(
            package="falco_drone_control",
            executable="hardware_bridge",
            name="teensy_hardware_bridge",
            namespace="real_drone",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Real-time EKF Node - processes real sensor data
        Node(
            package="falco_drone_description", 
            executable="ekf_realtime",
            name="ekf_realtime_node",
            namespace="real_drone",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Real-time Drone Controller - uses real EKF data
        Node(
            package="falco_drone_description",
            executable="drone_real_time_controller",
            name="real_drone_controller",
            namespace="real_drone", 
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Teleop for real drone (separate from simulation)
        Node(
            package="falco_drone_control",
            executable="teleop",
            name="real_drone_teleop",
            namespace="real_drone",
            output="screen",
            prefix="xterm -e",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
