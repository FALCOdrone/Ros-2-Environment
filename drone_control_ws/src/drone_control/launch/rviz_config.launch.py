from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to SDF file
    sdf_file = PathJoinSubstitution([
        FindPackageShare("drone_control"),
        "models",
        "drone_model.sdf"
    ])

    # Read the SDF file
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare("your_package_name"),
            "rviz",
            "view_robot.rviz"
        ])]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])   