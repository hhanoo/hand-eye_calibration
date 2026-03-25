"""
Robot pose reader using ROS2 TF2.
Reads end-effector pose from /tf topic published by robot drivers
(e.g. ur_robot_driver, doosan-robot2).
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from .pose_reader_interface import PoseReader


def _transform_to_matrix(transform):
    """Convert geometry_msgs/TransformStamped to 4x4 numpy matrix."""
    t = transform.transform.translation
    r = transform.transform.rotation

    # Quaternion to rotation matrix (xyzw convention from geometry_msgs)
    x, y, z, w = r.x, r.y, r.z, r.w
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z

    return T


class ROS2TFPoseReader(PoseReader):
    """Acquire robot poses via ROS2 TF2 lookups."""

    def __init__(self, node, base_frame: str = "base_link", ee_frame: str = "tool0"):
        """
        Args:
            node: rclpy Node instance (needed for TF listener)
            base_frame: Robot base TF frame name
            ee_frame: End-effector TF frame name
        """
        self.node = node
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def connect(self) -> bool:
        """Check if TF frames are available (5 second timeout)."""
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, Time(), timeout=Duration(seconds=5.0)
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def get_pose(self) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame, self.ee_frame, Time(), timeout=Duration(seconds=1.0)
        )
        return _transform_to_matrix(transform)

    def disconnect(self):
        pass
