"""
Tsai-Lenz hand-eye calibration solver (AX=XB).
Extracted from: hand_eye_cal_UR/f1_calibration_UR_2_using_pendant.py

Solves for X (camera-to-end-effector transform) given:
- A: relative robot motions
- B: relative camera/marker motions

References:
    Tsai, Lenz. "A new technique for fully autonomous and efficient 3D
    robotics hand/eye calibration." IEEE TRA, 1989.
"""
import numpy as np
from numpy import dot, eye, zeros, outer
from numpy.linalg import inv


def _log_rotation(R):
    """Rotation matrix logarithm (axis-angle vector)."""
    theta = np.arccos(np.clip((R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0, -1.0, 1.0))
    if abs(theta) < 1e-10:
        return np.zeros(3)
    return np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) * theta / (2 * np.sin(theta))


def _inv_sqrt(mat):
    """Matrix inverse square root via SVD."""
    u, s, v = np.linalg.svd(mat, full_matrices=True)
    return u.dot(np.diag(1.0 / np.sqrt(s))).dot(v)


def _compute_relative_motions(T_list):
    """Compute pairwise relative motions from consecutive absolute poses."""
    motions = []
    for i in range(len(T_list) - 1):
        motions.append(T_list[i + 1] @ inv(T_list[i]))
    return motions


def solve_tsai_lenz(robot_T_list, marker_T_list):
    """
    Solve AX=XB using Tsai-Lenz method.

    Takes absolute pose lists and computes relative motions internally.

    Args:
        robot_T_list: list of 4x4 robot base-to-EE transforms (one per sample)
        marker_T_list: list of 4x4 camera-to-marker transforms (one per sample)

    Returns:
        X: 4x4 ndarray (camera-to-end-effector transform)

    Raises:
        ValueError: if fewer than 3 pose pairs are provided
    """
    if len(robot_T_list) < 3:
        raise ValueError("At least 3 pose pairs required for Tsai-Lenz calibration")

    # Compute relative motions: A = inv(A2) * A1, B = B2 * inv(B1)
    A_list = []
    B_list = []
    for i in range(len(robot_T_list) - 1):
        A_list.append(inv(robot_T_list[i + 1]) @ robot_T_list[i])
        B_list.append(marker_T_list[i + 1] @ inv(marker_T_list[i]))

    N = len(A_list)

    # Step 1: Solve for rotation Rx
    M = np.zeros((3, 3))
    for i in range(N):
        Ra = A_list[i][0:3, 0:3]
        Rb = B_list[i][0:3, 0:3]
        M += outer(_log_rotation(Rb), _log_rotation(Ra))
    Rx = dot(_inv_sqrt(dot(M.T, M)), M.T)

    # Step 2: Solve for translation tx
    C = zeros((3 * N, 3))
    d = zeros((3 * N, 1))
    for i in range(N):
        Ra, ta = A_list[i][0:3, 0:3], A_list[i][0:3, 3]
        _Rb, tb = B_list[i][0:3, 0:3], B_list[i][0:3, 3]
        C[3 * i:3 * i + 3, :] = eye(3) - Ra
        d[3 * i:3 * i + 3, 0] = ta - dot(Rx, tb)

    tx = dot(inv(dot(C.T, C)), dot(C.T, d)).flatten()

    # Compose result
    X = np.eye(4)
    X[0:3, 0:3] = Rx
    X[0:3, 3] = tx

    return X
