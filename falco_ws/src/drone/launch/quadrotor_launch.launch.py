from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = FindPackageShare("drone").find("drone")
    
    xacro_file = os.path.join(pkg_share, "urdf", "quadrotor.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "controllers.yaml")

    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([

        # Upload robot description
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers_file],
            output="screen"
        ),

        # Spawner: joint_state_broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),

        # Spawner: quadrotor_velocity_controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["quadrotor_velocity_controller"],
            output="screen"
        ),
    ])