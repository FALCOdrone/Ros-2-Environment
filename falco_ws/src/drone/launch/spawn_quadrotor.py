from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('drone'),
        'urdf',
        'quadrotor.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='x', default_value='0'),
        DeclareLaunchArgument(name='y', default_value='0'),
        DeclareLaunchArgument(name='z', default_value='0.5'),
        DeclareLaunchArgument(name='R', default_value='0'),
        DeclareLaunchArgument(name='P', default_value='0'),
        DeclareLaunchArgument(name='Y', default_value='0'),

        # Robot state publisher with xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', model_path])
            }]
        ),

        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', 'robot_description',
                '-entity', 'quadrotor',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
            ],
            output='screen'
        )
    ])
