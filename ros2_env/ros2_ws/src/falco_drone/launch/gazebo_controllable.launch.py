import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Locate package share and its parent share/
    pkg_share = get_package_share_directory("falco_drone")
    share_root = str(Path(pkg_share).parent)

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Declare the URDF/model argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", "quadrotor_controllable.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    # World file argument
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(pkg_share, "worlds", "empty.world"),
        description="World file for simulation",
    )

    # Set Gazebo resource paths
    ign_gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            share_root,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            share_root,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # Generate the robot_description parameter
    robot_description = ParameterValue(
        Command(["xacro ",
                LaunchConfiguration("model"),
                " is_ignition:=",
                is_ignition]),
                value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    
    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )
    
    # Spawn drone entity
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description", 
            "-name", "quadrotor_0",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.5"
        ],
    )
    
    # ROS-Gazebo bridge for essential topics
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/quadrotor_0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/quadrotor_0/imu@sensor_msgs/msg/Imu[gz.msgs.IMU]",
            "/quadrotor_0/ground_truth/state@nav_msgs/msg/Odometry[gz.msgs.Odometry]",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]",
            "/quadrotor_0/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image]",
        ],
        remappings=[
            ("/quadrotor_0/cmd_vel", "/cmd_vel"),
        ]
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            ign_gazebo_resource_path,
            gz_sim_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
