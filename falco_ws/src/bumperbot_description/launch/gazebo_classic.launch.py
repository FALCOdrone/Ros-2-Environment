

#!/usr/bin/env python3
"""
gazebo_classic_bumperbot.launch.py

Launches Gazebo Classic with an empty world, publishes the bumper-bot robot
description, and spawns the robot into the simulator.
"""


from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)

from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command


import os

from os import pathsep
from pathlib import Path
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)

from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
      # ───────────────────────────── Paths ──────────────────────────────
    pkg_share      = get_package_share_directory("bumperbot_description")
    share_root     = str(Path(pkg_share).parent)          # …/install/share
    gazebo_pkg     = get_package_share_directory("gazebo_ros")

    gazebo_default_world = "/usr/share/gazebo-11/worlds/empty.world"
    # locate our package and the gazebo_ros package
    pkg_share  = get_package_share_directory("bumperbot_description")
    pkg_prefix = get_package_prefix("bumperbot_description")
    gz_ros_share = get_package_share_directory("gazebo_ros")

    # tell Gazebo where to find your meshes
    model_path = pkg_share \
               + pathsep + os.path.join(pkg_prefix, "share", "bumperbot_description")
  
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            share_root,
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    # allow overriding the URDF xacro path at launch
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF XACRO"
    )

    # include the “all-in-one” Gazebo launch (server+client+factory plugin)
    world_file = os.path.join(
        gz_ros_share, "worlds", "optimized.world"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "paused": "false",
            "use_sim_time": "true",
            "gui": "true"
        }.items(),
    )

    # publish the URDF
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # spawn into Gazebo once it’s up
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
          "-topic", "robot_description",
          "-entity", "bumperbot",
          "-x", "0", "-y", "0", "-z", "0.1"
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo_model_path,
        model_arg,
        gazebo,
        rsp,
        spawner,
    ])