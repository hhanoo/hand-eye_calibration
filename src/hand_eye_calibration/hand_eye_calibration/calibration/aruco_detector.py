"""
ArUco marker detection module.
Extracted from: hand_eye_cal_UR/f1_calibration_UR_2_using_pendant.py
"""

import cv2
import cv2.aruco as aruco
import numpy as np


class ArUcoDetector:
    """Robot-agnostic ArUco grid board detector."""

    def __init__(
        self,
        marker_size=6,
        total_markers=250,
        grid_shape=(5, 7),
        marker_length=0.0375,
        marker_separation=0.00375,
    ):
        """
        Args:
            marker_size: ArUco dictionary size (e.g. 6 for 6x6)
            total_markers: Number of markers in dictionary (e.g. 250)
            grid_shape: (cols, rows) of the grid board
            marker_length: Physical marker length in meters
            marker_separation: Physical separation between markers in meters
        """
        key = getattr(aruco, f"DICT_{marker_size}X{marker_size}_{total_markers}")
        self.aruco_dict = aruco.getPredefinedDictionary(key)
        self.aruco_param = aruco.DetectorParameters()
        self.board = aruco.GridBoard(
            grid_shape, marker_length, marker_separation, self.aruco_dict
        )
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_param)

        # Board corner offsets in board coordinate frame
        cols, rows = grid_shape
        board_w = cols * marker_length + (cols - 1) * marker_separation
        board_h = rows * marker_length + (rows - 1) * marker_separation
        self.corner_offsets = np.array(
            [
                [0.0, 0.0, 0.0],  # 좌하단 (원점)
                [board_w, 0.0, 0.0],  # 우하단
                [0.0, board_h, 0.0],  # 좌상단
                [board_w, board_h, 0.0],  # 우상단
                [board_w / 2, board_h / 2, 0.0],  # 정중앙
            ]
        )

    def detect(self, color_img, camera_matrix, dist_coeffs):
        """
        Detect ArUco board and estimate its pose.

        Args:
            color_img: BGR image (numpy array)
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients (1x5 or similar)

        Returns:
            (success, annotated_img, T_cam_marker)
            - success: bool, whether board was detected
            - annotated_img: image with drawn axes (or original if not detected)
            - T_cam_marker: 4x4 transformation matrix (camera to marker)
        """
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        bboxs, ids, _rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return False, color_img, None

        retval, rvec, tvec = aruco.estimatePoseBoard(
            bboxs, ids, self.board, camera_matrix, dist_coeffs, None, None
        )

        if retval == 0:
            return False, color_img, None

        Rotmat = np.zeros((3, 3))
        cv2.Rodrigues(rvec, Rotmat)

        # Draw axes at all four board corners
        annotated = color_img
        axis_length = 0.03
        for offset in self.corner_offsets:
            corner_tvec = tvec.flatten() + Rotmat @ offset
            annotated = cv2.drawFrameAxes(
                annotated,
                camera_matrix,
                dist_coeffs,
                rvec,
                corner_tvec.reshape(3, 1),
                axis_length,
            )

        T = np.eye(4)
        T[0:3, 0:3] = Rotmat
        T[0:3, 3] = tvec.flatten()

        return True, annotated, T
