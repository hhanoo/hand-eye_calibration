"""
Dual Quaternion RANSAC hand-eye calibration wrapper.
Uses the ethz-asl hand_eye_calibration library (Daniilidis 1999).
Source: hand_eye_cal_UR2/p0_cal.py
"""

import numpy as np

from ..hand_eye_calibration_lib import (
    DualQuaternion,
    HandEyeConfig,
    align_paths_at_index,
    compute_hand_eye_calibration_RANSAC,
)


def solve_dq_ransac(robot_T_list, marker_T_list, iterations=50, sample_size=3):
    """
    Solve AX=XB using Dual Quaternion RANSAC (Daniilidis 1999).

    Args:
        robot_T_list: list of 4x4 robot base-to-EE transforms (one per sample)
        marker_T_list: list of 4x4 camera-to-marker transforms (one per sample)
        iterations: RANSAC max iterations
        sample_size: RANSAC sample size per iteration

    Returns:
        (success, X, rmse, num_inliers)
        - success: bool
        - X: 4x4 ndarray (hand-to-eye transform), or None if failed
        - rmse: float, position RMSE
        - num_inliers: int
    """
    if len(robot_T_list) < sample_size:
        return False, None, float("inf"), 0

    # Convert to DualQuaternion
    dq_B_H_vec = []
    dq_W_E_vec = []
    for robot_T, marker_T in zip(robot_T_list, marker_T_list):
        dq_B_H = DualQuaternion.from_transformation_matrix(robot_T)
        dq_B_H_vec.append(dq_B_H)

        # Invert marker T (camera-to-marker → marker-to-camera)
        marker_T_inv = np.linalg.inv(marker_T)
        dq_W_E = DualQuaternion.from_transformation_matrix(marker_T_inv)
        dq_W_E_vec.append(dq_W_E)

    # Align paths at origin
    dq_B_H_vec = align_paths_at_index(dq_B_H_vec)

    # Configure RANSAC
    config = HandEyeConfig()
    config.visualize = False
    config.ransac_max_number_iterations = iterations
    config.ransac_sample_size = sample_size

    # Run RANSAC
    result = compute_hand_eye_calibration_RANSAC(dq_B_H_vec, dq_W_E_vec, config)

    success = result[0]
    if not success:
        return False, None, float("inf"), 0

    dq_H_E = result[1]
    rmse = result[2]
    num_inliers = result[3]

    # Convert to 4x4 matrix
    X = np.linalg.inv(dq_H_E.to_matrix())

    return success, X, rmse, num_inliers
