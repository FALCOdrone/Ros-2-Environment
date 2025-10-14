#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directories 
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_drone_simulation = FindPackageShare(package='drone_simulation').find('drone_simulation')
    pkg_drone_description = FindPackageShare(package='drone_description').find('drone_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='drone_world.sdf',
        description='SDF world file name'
    )
    
    # World file path
    world_file = PathJoinSubstitution([pkg_drone_simulation, 'worlds', world_name])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Robot description
    urdf_file = PathJoinSubstitution([pkg_drone_description, 'urdf', 'quadrotor.urdf.xacro'])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_file
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drone',
        arguments=[
            '-entity', 'quadrotor_drone',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_name_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot
    ])
