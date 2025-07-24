#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0'),
        
        Node(
            package='falco_drone',
            executable='drone_controller.py',
            name='drone_controller',
            output='screen',
            arguments=[LaunchConfiguration('drone_id')]
        )
    ])
