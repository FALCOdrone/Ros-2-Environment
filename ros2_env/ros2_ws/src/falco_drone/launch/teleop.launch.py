from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Teleop node
        Node(
            package='falco_drone',
            executable='teleop_quadrotor.py',
            name='teleop_quadrotor',
            output='screen',
            prefix='xterm -e',  # Run in a separate terminal
        ),
    ])
