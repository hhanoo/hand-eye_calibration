"""
Launch file for hand-eye calibration.
Starts realsense2_camera and the calibration GUI node.
When GUI is closed, the entire launch shuts down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Config file
    config_file = os.path.join(
        get_package_share_directory("hand_eye_calibration"), "config", "default.yaml"
    )

    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                )
            ]
        ),
        launch_arguments={
            "depth_module.depth_profile": "1280x720x30",
            "rgb_camera.color_profile": "1280x720x30",
            "align_depth.enable": "true",
        }.items(),
    )

    # Calibration GUI node — on_exit: shut down entire launch
    calibration_node = Node(
        package="hand_eye_calibration",
        executable="gui_node",
        name="hand_eye_calibration",
        parameters=[config_file],
        output="screen",
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            realsense_launch,
            calibration_node,
        ]
    )
