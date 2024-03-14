from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

pkg_dir = get_package_share_directory("hdl_global_localization")


def generate_launch_description():
    list = [
        Node(
            package="hdl_global_localization",
            executable="hdl_global_localization",
            output="screen",
            parameters=[
                os.path.join(pkg_dir, "config", "hdl_global_localization.launch.yaml")
            ],
        )
    ]
    
    return LaunchDescription(list)