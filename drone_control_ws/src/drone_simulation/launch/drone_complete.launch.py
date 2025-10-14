#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Package directories
    pkg_drone_simulation = FindPackageShare(package='drone_simulation').find('drone_simulation')
    pkg_drone_control = FindPackageShare(package='drone_control').find('drone_control')
    
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
    
    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drone_simulation, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': world_name
        }.items()
    )
    
    # Launch control system
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drone_control, 'launch', 'control_system.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # TF static transforms for sensor frames
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    tf_base_to_sonar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_sonar',
        arguments=['0', '0', '-0.08', '0', '1.5708', '0', 'base_link', 'sonar_link']
    )
    
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0.15', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    tf_base_to_barometer = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_barometer',
        arguments=['0.05', '0.05', '0.02', '0', '0', '0', 'base_link', 'barometer_link']
    )
    
    tf_base_to_magnetometer = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_magnetometer',
        arguments=['-0.05', '0.05', '0.02', '0', '0', '0', 'base_link', 'magnetometer_link']
    )
    
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_name_arg,
        gazebo_launch,
        control_launch,
        rviz_node,
        tf_base_to_imu,
        tf_base_to_sonar,
        tf_base_to_camera,
        tf_base_to_barometer,
        tf_base_to_magnetometer
    ])
