from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params_file = os.path.join(
        get_package_share_directory("auv_2d_control"),
        "config",
        "control_2d_default.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML file with controller and allocator parameters",
            ),
            Node(
                package="auv_2d_control",
                executable="controller_2d",
                name="controller_2d",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="auv_2d_control",
                executable="thruster_allocator_2d",
                name="thruster_allocator_2d",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

