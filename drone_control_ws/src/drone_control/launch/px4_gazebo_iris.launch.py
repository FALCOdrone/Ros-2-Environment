#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo in headless mode'
    )
    
    use_px4_arg = DeclareLaunchArgument(
        'use_px4',
        default_value='false',
        description='Start PX4 SITL automatically'
    )
    
    # PX4 SITL process (conditional)
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd /opt/px4_source && HEADLESS=1 make px4_sitl gazebo-classic_iris'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_px4'))
    )
    
    # MAVROS node for PX4 communication (delay start to let PX4 initialize)
    mavros_node = TimerAction(
        period=5.0,  # Wait 5 seconds for PX4 to start
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[{
                    'fcu_url': 'udp://:14540@127.0.0.1:14557',
                    'gcs_url': '',
                    'target_system_id': 1,
                    'target_component_id': 1,
                    'fcu_protocol': 'v2.0',
                    'system_id': 255,
                    'component_id': 240,
                }]
            )
        ]
    )
    
    # High-level mission/setpoint publisher node (uses PX4's controllers)
    mission_commander = TimerAction(
        period=8.0,  # Wait 8 seconds for MAVROS to connect
        actions=[
            Node(
                package='drone_control',
                executable='mission_commander',
                name='mission_commander',
                output='screen',
                parameters=[{
                    'vehicle_name': 'iris',
                    'auto_arm': True,
                    'auto_takeoff': True,
                    'takeoff_altitude': 2.0
                }]
            )
        ]
    )
    
    return LaunchDescription([
        headless_arg,
        use_px4_arg,
        px4_sitl,
        mavros_node,
        mission_commander
    ])
