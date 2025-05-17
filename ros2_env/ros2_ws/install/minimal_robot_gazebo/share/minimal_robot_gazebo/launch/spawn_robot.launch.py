#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('minimal_robot_gazebo')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Allow overriding the XACRO path
    model_arg = DeclareLaunchArgument(
        'model', default_value=urdf_xacro,
        description='Absolute path to robot xacro file'
    )

    # Tell Ignition/Gazebo where to find resources
    gazebo_res    = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share)
    gazebo_models = SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(Path(pkg_share) / 'models'))

    # Expand XACRO -> URDF string
    robot_description = ParameterValue(
        Command(['xacro', ' ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Launch Ignition Gazebo (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ' -v 4 -r empty.sdf'}.items()
    )

    # Publish TF from URDF
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Log spawning
    log_model = LogInfo(msg=['Spawning robot from URDF: ', LaunchConfiguration('model')])

    # Spawn using ros_gz_sim create (bypass spawn_entity.py)
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-name', 'robot',
            '-x', '0', '-y', '0', '-z', '0.05',
        ]
    )

    # Bridge /clock, /joint_states, /tf back to ROS 2
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gazebo_res,
        gazebo_models,
        gazebo,
        rsp,
        log_model,
        spawn,
        bridge,
    ])
