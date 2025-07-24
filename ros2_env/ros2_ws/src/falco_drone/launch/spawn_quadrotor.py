from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('falco_drone'),
        'urdf',
        'quadrotor_controllable.urdf.xacro'  # Use the controllable version with Ignition plugins
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='x', default_value='0'),
        DeclareLaunchArgument(name='y', default_value='0'),
        DeclareLaunchArgument(name='z', default_value='0.5'),
        DeclareLaunchArgument(name='R', default_value='0'),
        DeclareLaunchArgument(name='P', default_value='0'),
        DeclareLaunchArgument(name='Y', default_value='0'),
        DeclareLaunchArgument(name='drone_id', default_value='0'),

        # Robot state publisher with xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', model_path, ' id:=', LaunchConfiguration('drone_id')]),
                    value_type=str
                )
            }]
        ),

        # Spawn robot in Ignition Gazebo (compatible with gz_sim)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-topic', 'robot_description',
                '-name', 'quadrotor',
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
