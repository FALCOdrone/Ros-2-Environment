from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the PX4 MAVROS SITL launch file
    px4_launch_dir = os.path.join(
        get_package_share_directory('px4'),
        'launch'
    )

    return LaunchDescription([
        # Include PX4's MAVROS SITL launch file
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(px4_launch_dir, 'mavros_posix_sitl.launch.py')
            )
        ),

        # Offboard control node
        Node(
            package='drone',
            executable='offb_node',
            name='offb_node',
            output='screen',
            parameters=[]
        )
    ])