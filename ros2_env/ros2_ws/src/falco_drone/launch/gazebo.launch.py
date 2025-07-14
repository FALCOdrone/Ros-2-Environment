import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
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
        default_value=os.path.join(pkg_share, "urdf", "quadrotor.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    # Set Ignition resource paths (chaining existing with new)
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

    # Log to screen so we can verify inside the launch
    log_env = ExecuteProcess(
        cmd=["bash", "-lc", "echo IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH"],
        output="screen",
    )

    # Generate URDF from xacro with the specified parameters
    robot_description = ParameterValue(
        Command([
            "xacro ", 
            os.path.join(pkg_share, "urdf", "quadrotor.urdf.xacro"),
            " id:=0"
        ]),
        value_type=str
    )

    # Nodes and includes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description, 
            "use_sim_time": True,
            "publish_frequency": 30.0
        }],
        output="screen",
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )
    # Spawn entity using ros_gz_sim (compatible with Ignition Gazebo)
    spawn_entity = TimerAction(
        period=5.0,  # Wait 5 seconds for robot_state_publisher to start publishing
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "quadrotor_0",
                    "-x", "-2.5",
                    "-y", "2.5", 
                    "-z", "0.15"
                ],
            )
        ]
    )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
    )

    return LaunchDescription(
        [
            log_env,
            model_arg,
            ign_gazebo_resource_path,
            gz_sim_resource_path,
            robot_state_publisher_node,
            gazebo,
            spawn_entity,
            gz_ros2_bridge,
        ]
    )