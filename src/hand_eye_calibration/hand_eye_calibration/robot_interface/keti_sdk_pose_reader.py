"""
Robot pose reader using KETIRobotSDK.
Supports: UR10 (type=2), M1013/Doosan (type=3), RB10 (type=1), Indy7 (type=4), TestDummy (type=0)
"""
import os
import numpy as np

from .pose_reader_interface import PoseReader


# Robot type constants (from sdk.py)
ROBOT_TYPES = {
    "TestDummy": 0,
    "RB10": 1,
    "UR10": 2,
    "M1013": 3,
    "Indy7": 4,
}


class KETISDKPoseReader(PoseReader):
    """Acquire robot poses via KETIRobotSDK (librobotsdk.so)."""

    def __init__(self, robot_type: int, ip: str, port: int):
        """
        Args:
            robot_type: Robot type constant (0=TestDummy, 1=RB10, 2=UR10, 3=M1013, 4=Indy7)
            ip: Robot IP address
            port: Robot port
        """
        # Set library path before importing SDK
        sdk_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'module', 'KETIRobotSDK')
        lib_path = os.path.join(sdk_dir, 'librobotsdk.so')

        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'module'))
        from KETIRobotSDK import sdk
        if os.path.isfile(lib_path):
            sdk.setLibPath(lib_path)

        self.sdk = sdk
        self.robot = sdk.Robot()
        self.robot_type = robot_type
        self.ip = ip
        self.port = port

    def connect(self) -> bool:
        self.robot.SetRobotConf(self.robot_type, self.ip, self.port)
        return self.robot.RobotConnect()

    def get_pose(self) -> np.ndarray:
        info = self.robot.RobotInfo()
        return np.array(list(info.Mat)).reshape(4, 4)

    def disconnect(self):
        self.robot.RobotDisconnect()
