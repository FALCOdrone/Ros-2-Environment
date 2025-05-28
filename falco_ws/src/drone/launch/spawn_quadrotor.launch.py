from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('drone'),
        'urdf',
        'quadrotor_base.urdf.xacro'
    )

    control_config = PathJoinSubstitution([
        FindPackageShare('drone'),
        'config',
        'controllers.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(name='x', default_value='0'),
        DeclareLaunchArgument(name='y', default_value='0'),
        DeclareLaunchArgument(name='z', default_value='0.5'),
        DeclareLaunchArgument(name='R', default_value='0'),
        DeclareLaunchArgument(name='P', default_value='0'),
        DeclareLaunchArgument(name='Y', default_value='0'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
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
        ),

        # Start ros2_control
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller'],
            output='screen'
        ),

        # Spawner: Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Spawner: Main Controller (rename 'controller' to your actual controller name)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['controller'],
            output='screen'
        )
    ])