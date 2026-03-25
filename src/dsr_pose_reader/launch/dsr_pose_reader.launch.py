import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("dsr_pose_reader"), "config", "default.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to parameter YAML file",
            ),
            Node(
                package="dsr_pose_reader",
                executable="pose_reader_node",
                name="dsr_pose_reader",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
