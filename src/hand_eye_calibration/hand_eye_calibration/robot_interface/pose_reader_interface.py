"""
Abstract base class for robot pose readers.
All robot pose acquisition methods must implement this interface.
"""
from abc import ABC, abstractmethod

import numpy as np


class PoseReader(ABC):
    """Interface for acquiring robot end-effector poses."""

    @abstractmethod
    def connect(self) -> bool:
        """
        Connect to the robot.

        Returns:
            True if connection successful, False otherwise.
        """
        ...

    @abstractmethod
    def get_pose(self) -> np.ndarray:
        """
        Get current end-effector pose.

        Returns:
            4x4 homogeneous transformation matrix (base to end-effector).
        """
        ...

    @abstractmethod
    def disconnect(self):
        """Disconnect from the robot."""
        ...
