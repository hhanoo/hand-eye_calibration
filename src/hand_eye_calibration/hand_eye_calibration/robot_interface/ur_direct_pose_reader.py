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

    # The real-time interface streams one fixed-size packet per control cycle.
    # Its length varies with controller software (1044/1060/1108/1116/1220),
    # all of which fall in this range; the leading int32 carries the length.
    _MIN_PACKET = 1024
    _MAX_PACKET = 1279
    _POSE_OFFSET = 444  # "Tool vector actual" — 6 big-endian doubles

    def __init__(self, ip: str, port: int = 30003):
        self.ip = ip
        self.port = port
        self._socket = None
        self._lock = threading.Lock()
        self._latest_pose = None
        self._buffer = b""
        self._seq = 0
        self._running = False
        self._thread = None

    @property
    def pose_seq(self) -> int:
        """Counter incremented once per packet parsed (frozen = stale data)."""
        with self._lock:
            return self._seq

    def _feed(self, data: bytes) -> int:
        """Reassemble the TCP stream, keep the newest pose, return packet count."""
        self._buffer += data
        count = 0
        latest = None

        while True:
            if len(self._buffer) < 4:
                break
            size = struct.unpack(">i", self._buffer[0:4])[0]
            if not (self._MIN_PACKET <= size <= self._MAX_PACKET):
                # Stream is misaligned (joined mid-packet, or bytes lost).
                # Scan for the next plausible length header.
                offset = self._resync(self._buffer)
                if offset is None:
                    self._buffer = self._buffer[-3:]
                    break
                self._buffer = self._buffer[offset:]
                continue
            if len(self._buffer) < size:
                break
            # Keep overwriting: when several packets arrive in one recv(),
            # only the last one is current.
            latest = self._buffer[: self._POSE_OFFSET + 48]
            self._buffer = self._buffer[size:]
            count += 1

        if latest is not None:
            x, y, z, rx, ry, rz = struct.unpack(
                ">6d", latest[self._POSE_OFFSET : self._POSE_OFFSET + 48]
            )
            T = self._build_matrix(x, y, z, rx, ry, rz)
            with self._lock:
                self._latest_pose = T
                self._seq += 1
        return count

    @classmethod
    def _resync(cls, buf: bytes):
        """Index of the next plausible packet header, or None."""
        for i in range(1, len(buf) - 3):
            size = struct.unpack(">i", buf[i : i + 4])[0]
            if cls._MIN_PACKET <= size <= cls._MAX_PACKET:
                return i
        return None

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
                data = self._socket.recv(8192)
                if len(data) == 0:
                    break
                self._feed(data)
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
