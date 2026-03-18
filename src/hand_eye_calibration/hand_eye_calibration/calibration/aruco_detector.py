"""
ArUco marker detection module.
Extracted from: hand_eye_cal_UR/f1_calibration_UR_2_using_pendant.py
"""
import numpy as np
import cv2
import cv2.aruco as aruco


class ArUcoDetector:
    """Robot-agnostic ArUco grid board detector."""

    def __init__(self, marker_size=6, total_markers=250,
                 grid_shape=(5, 7), marker_length=0.037, marker_separation=0.003):
        """
        Args:
            marker_size: ArUco dictionary size (e.g. 6 for 6x6)
            total_markers: Number of markers in dictionary (e.g. 250)
            grid_shape: (cols, rows) of the grid board
            marker_length: Physical marker length in meters
            marker_separation: Physical separation between markers in meters
        """
        key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
        self.aruco_dict = aruco.getPredefinedDictionary(key)
        self.aruco_param = aruco.DetectorParameters()
        self.board = aruco.GridBoard(
            grid_shape, marker_length, marker_separation, self.aruco_dict)
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_param)

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
            bboxs, ids, self.board, camera_matrix, dist_coeffs, None, None)

        if retval == 0:
            return False, color_img, None

        annotated = cv2.drawFrameAxes(
            color_img, camera_matrix, dist_coeffs, rvec, tvec, 0.053)

        Rotmat = np.zeros((3, 3))
        cv2.Rodrigues(rvec, Rotmat)

        T = np.eye(4)
        T[0:3, 0:3] = Rotmat
        T[0:3, 3] = tvec.flatten()

        return True, annotated, T
