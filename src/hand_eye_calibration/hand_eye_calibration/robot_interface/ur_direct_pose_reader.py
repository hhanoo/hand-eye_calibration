"""
Robot pose reader using direct TCP socket to UR controller (port 30003).
Single read-only socket — does NOT send commands, so pendant control is preserved.
"""

import socket
import struct
import threading
import time

import numpy as np

from .pose_reader_interface import PoseReader


class URDirectPoseReader(PoseReader):
    """Acquire UR robot poses via direct socket read on port 30003."""

    def __init__(self, ip: str, port: int = 30003):
        self.ip = ip
        self.port = port
        self._socket = None
        self._lock = threading.Lock()
        self._latest_pose = None
        self._running = False
        self._thread = None

    def connect(self) -> bool:
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(5.0)
            self._socket.connect((self.ip, self.port))
            self._socket.settimeout(None)

            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

            # Wait up to 2 seconds for first valid data
            for _ in range(20):
                time.sleep(0.1)
                with self._lock:
                    if self._latest_pose is not None:
                        return True

            # No data received — clean up
            self.disconnect()
            return False
        except Exception:
            self.disconnect()
            return False

    def _read_loop(self):
        while self._running:
            try:
                data = self._socket.recv(5120)
                if len(data) == 0:
                    break
                if len(data) > 1108 and data[0:3] == b"\x00\x00\x04":
                    # Actual TCP pose: bytes 444-492 (6 x big-endian double)
                    x, y, z, rx, ry, rz = struct.unpack(">6d", data[444:492])
                    T = self._build_matrix(x, y, z, rx, ry, rz)
                    with self._lock:
                        self._latest_pose = T
            except Exception:
                break
        self._running = False

    @staticmethod
    def _build_matrix(x, y, z, rx, ry, rz):
        """Build 4x4 homogeneous matrix from position + rotation vector."""
        R = URDirectPoseReader._rotation_vector_to_matrix(rx, ry, rz)
        T = np.eye(4, dtype=np.float64)
        T[0:3, 0:3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    @staticmethod
    def _rotation_vector_to_matrix(rx, ry, rz):
        """Convert rotation vector to 3x3 rotation matrix (Rodrigues)."""
        theta = np.sqrt(rx * rx + ry * ry + rz * rz)
        if theta < 1e-10:
            return np.eye(3, dtype=np.float64)

        k = np.array([rx, ry, rz]) / theta
        K = np.array(
            [
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0],
            ],
            dtype=np.float64,
        )

        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        return R

    def get_pose(self) -> np.ndarray:
        with self._lock:
            if self._latest_pose is None:
                raise RuntimeError("No pose data available from UR controller")
            return self._latest_pose.copy()

    def disconnect(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        self._latest_pose = None
