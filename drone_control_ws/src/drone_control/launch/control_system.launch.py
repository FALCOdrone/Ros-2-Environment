#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Drone controller node
    drone_controller = Node(
        package='drone_control',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # State estimator node
    state_estimator = Node(
        package='drone_control',
        executable='state_estimator',
        name='state_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Trajectory planner node
    trajectory_planner = Node(
        package='drone_control',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Sensor simulation nodes
    barometer_sim = Node(
        package='drone_sensors',
        executable='barometer_sim',
        name='barometer_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    magnetometer_sim = Node(
        package='drone_sensors',
        executable='magnetometer_sim',
        name='magnetometer_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        drone_controller,
        state_estimator,
        trajectory_planner,
        barometer_sim,
        magnetometer_sim
    ])
