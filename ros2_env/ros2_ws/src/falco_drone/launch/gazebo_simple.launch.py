#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    # Locate package share and its parent share/
    pkg_share = get_package_share_directory("falco_drone")
    share_root = str(Path(pkg_share).parent)

    # Set Ignition resource paths
    ign_gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            share_root,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            share_root,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="empty.sdf",
        description="World file name to load"
    )

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",  # Enable GUI by default
        description="Start Gazebo with GUI (true/false)"
    )

    verbose_arg = DeclareLaunchArgument(
        name="verbose",
        default_value="true",
        description="Enable verbose output"
    )

    # Gazebo with GUI support
    gazebo_server = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', 
            '-v', '4',          # Verbose level 4
            '-r',               # Run simulation immediately
            LaunchConfiguration("world")  # Removed -s flag to enable GUI
        ],
        output='screen',
        shell=False
    )

    # ROS-Gazebo bridge for essential topics
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # Log environment for debugging
    log_env = ExecuteProcess(
        cmd=["bash", "-c", "echo 'Gazebo Simple Launch Started' && echo 'IGN_GAZEBO_RESOURCE_PATH='$IGN_GAZEBO_RESOURCE_PATH"],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            verbose_arg,
            ign_gazebo_resource_path,
            gz_sim_resource_path,
            log_env,
            gazebo_server,
            gz_ros2_bridge,
        ]
    )
