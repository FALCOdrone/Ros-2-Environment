#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_drone_description = FindPackageShare('drone_description')
    pkg_drone_simulation = FindPackageShare('drone_simulation')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    
    # Paths
    urdf_file = PathJoinSubstitution([pkg_drone_description, 'urdf', 'quadrotor.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg_drone_simulation, 'worlds', 'drone_world.sdf'])
    
    # Launch Arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    # Process XACRO file
    import xacro
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Gazebo Ignition Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'quadrotor_drone',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/drone/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/drone/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/drone/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/drone/sonar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/drone/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '--ros-args', '-r', '__node:=ros_gz_bridge'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time_arg,
        declare_headless_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
