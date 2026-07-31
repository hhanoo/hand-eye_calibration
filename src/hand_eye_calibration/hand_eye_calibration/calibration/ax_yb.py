"""
AX=YB robot-world / hand-eye calibration solver.

Follows Ha, "Probabilistic Framework for Hand-Eye and Robot-World
Calibration AX=YB" (IEEE T-RO 2023): absolute poses are used directly
(no relative-motion composition), and the camera measurement is kept in
its native cam→board direction so its noise stays right-translated at
the board (target) frame — the coordination under which distance
minimization coincides with maximum likelihood estimation.

Loop closure per sample:  A_i · Y · B_i = X
    A_i : base→TCP    robot pose (treated as noiseless)
    Y   : TCP→camera  hand-eye transform (unknown)
    B_i : cam→board   camera measurement (noisy at the board side)
    X   : base→board  robot-world transform (unknown)

Two stages:
 1. Closed-form initialization via cv2.calibrateRobotWorldHandEye (Shah).
 2. Levenberg-Marquardt refinement of the MLE cost for board-side
    isotropic noise (paper eq. 21-22):
        sum_i ||log R_{M_i}||^2 + w^2 ||t_{M_i}||^2,  M_i = X^-1 A_i Y B_i
"""

import cv2
import numpy as np
from numpy.linalg import inv, norm

# Positional weight: rotation in rad vs translation in m.
# Default equates 1 degree of rotation noise to 3 mm of translation noise,
# the ratio used in the paper's hardware experiments.
DEFAULT_TRANS_WEIGHT = np.radians(1.0) / 0.003


def _log_SO3(R):
    w, _ = cv2.Rodrigues(np.asarray(R, dtype=float))
    return w.flatten()


def _exp_SO3(w):
    R, _ = cv2.Rodrigues(np.asarray(w, dtype=float))
    return R


def _to_T(w, t):
    T = np.eye(4)
    T[:3, :3] = _exp_SO3(w)
    T[:3, 3] = t
    return T


def _params_to_YX(params):
    Y = _to_T(params[0:3], params[3:6])
    X = _to_T(params[6:9], params[9:12])
    return Y, X


def _residuals(params, robot_T_list, marker_T_list, trans_weight):
    """Stacked per-sample noise vector [log R_M ; w * t_M], M = X^-1 A Y B."""
    Y, X = _params_to_YX(params)
    X_inv = inv(X)
    res = np.empty(6 * len(robot_T_list))
    for i, (A, B) in enumerate(zip(robot_T_list, marker_T_list)):
        M = X_inv @ A @ Y @ B
        res[6 * i : 6 * i + 3] = _log_SO3(M[:3, :3])
        res[6 * i + 3 : 6 * i + 6] = trans_weight * M[:3, 3]
    return res


def _residual_stats(params, robot_T_list, marker_T_list):
    """RMSE of the per-sample noise M_i in degrees / millimeters."""
    Y, X = _params_to_YX(params)
    X_inv = inv(X)
    rot_sq, trans_sq = 0.0, 0.0
    for A, B in zip(robot_T_list, marker_T_list):
        M = X_inv @ A @ Y @ B
        rot_sq += norm(_log_SO3(M[:3, :3])) ** 2
        trans_sq += norm(M[:3, 3]) ** 2
    n = len(robot_T_list)
    return (
        np.degrees(np.sqrt(rot_sq / n)),
        1000.0 * np.sqrt(trans_sq / n),
    )


def _initial_guess(robot_T_list, marker_T_list):
    """Closed-form init via cv2.calibrateRobotWorldHandEye (Shah).

    OpenCV convention (point-mapping): world2cam = cam-from-world,
    base2gripper = gripper-from-base. With world = board:
      world2cam    = marker_T (as measured, no inversion)
      base2gripper = inv(robot_T)
    Returns base2world = board-from-base = X^-1 and
            gripper2cam = cam-from-TCP = Y^-1.
    """
    R_w2c = [B[:3, :3] for B in marker_T_list]
    t_w2c = [B[:3, 3].reshape(3, 1) for B in marker_T_list]
    R_b2g, t_b2g = [], []
    for A in robot_T_list:
        A_inv = inv(A)
        R_b2g.append(A_inv[:3, :3])
        t_b2g.append(A_inv[:3, 3].reshape(3, 1))

    R_b2w, t_b2w, R_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(
        R_w2c, t_w2c, R_b2g, t_b2g
    )

    X_inv = np.eye(4)
    X_inv[:3, :3] = R_b2w
    X_inv[:3, 3] = t_b2w.flatten()
    Y_inv = np.eye(4)
    Y_inv[:3, :3] = R_g2c
    Y_inv[:3, 3] = t_g2c.flatten()
    return inv(Y_inv), inv(X_inv)


def _refine_lm(
    params, robot_T_list, marker_T_list, trans_weight, max_iter=100, diff_step=1e-7
):
    """Levenberg-Marquardt with numeric Jacobian (12 parameters)."""
    res = _residuals(params, robot_T_list, marker_T_list, trans_weight)
    cost = res @ res
    lam = 1e-4

    for _ in range(max_iter):
        J = np.empty((res.size, params.size))
        for k in range(params.size):
            probe = params.copy()
            probe[k] += diff_step
            J[:, k] = (
                _residuals(probe, robot_T_list, marker_T_list, trans_weight) - res
            ) / diff_step

        JtJ = J.T @ J
        g = J.T @ res
        step_taken = False
        for _ in range(20):
            try:
                delta = np.linalg.solve(JtJ + lam * np.diag(np.diag(JtJ)), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            trial = params + delta
            trial_res = _residuals(trial, robot_T_list, marker_T_list, trans_weight)
            trial_cost = trial_res @ trial_res
            if trial_cost < cost:
                params, res, cost = trial, trial_res, trial_cost
                lam = max(lam / 3.0, 1e-12)
                step_taken = True
                break
            lam *= 3.0
        if not step_taken or norm(delta) < 1e-12:
            break
    return params


def solve_ax_yb(robot_T_list, marker_T_list, trans_weight=None):
    """
    Solve A·Y·B = X for the hand-eye transform Y and robot-world transform X.

    Args:
        robot_T_list: list of 4x4 base→TCP robot poses (absolute, per sample)
        marker_T_list: list of 4x4 cam→board camera measurements (as detected)
        trans_weight: rotation(rad)-vs-translation(m) weight in the MLE cost;
            defaults to 1° ≡ 3 mm (paper's hardware setting)

    Returns:
        (Y, X, info)
        - Y: 4x4 ndarray, TCP→camera (hand-eye result)
        - X: 4x4 ndarray, base→board (robot-world result, for validation)
        - info: dict with rot_rmse_deg / trans_rmse_mm (refined),
          init_rot_rmse_deg / init_trans_rmse_mm (closed-form init), n

    Raises:
        ValueError: if fewer than 3 pose pairs are provided
    """
    if len(robot_T_list) < 3:
        raise ValueError("At least 3 pose pairs required for AX=YB calibration")
    if len(robot_T_list) != len(marker_T_list):
        raise ValueError("robot_T_list and marker_T_list must have equal length")
    if trans_weight is None:
        trans_weight = DEFAULT_TRANS_WEIGHT

    robot_T_list = [np.asarray(T, dtype=float) for T in robot_T_list]
    marker_T_list = [np.asarray(T, dtype=float) for T in marker_T_list]

    Y0, X0 = _initial_guess(robot_T_list, marker_T_list)
    params = np.concatenate(
        [
            _log_SO3(Y0[:3, :3]),
            Y0[:3, 3],
            _log_SO3(X0[:3, :3]),
            X0[:3, 3],
        ]
    )
    init_rot_rmse, init_trans_rmse = _residual_stats(
        params, robot_T_list, marker_T_list
    )
    init_res = _residuals(params, robot_T_list, marker_T_list, trans_weight)

    params = _refine_lm(params, robot_T_list, marker_T_list, trans_weight)

    Y, X = _params_to_YX(params)
    rot_rmse, trans_rmse = _residual_stats(params, robot_T_list, marker_T_list)
    res = _residuals(params, robot_T_list, marker_T_list, trans_weight)
    info = {
        "rot_rmse_deg": rot_rmse,
        "trans_rmse_mm": trans_rmse,
        "cost": float(res @ res),
        "init_rot_rmse_deg": init_rot_rmse,
        "init_trans_rmse_mm": init_trans_rmse,
        "init_cost": float(init_res @ init_res),
        "n": len(robot_T_list),
    }
    return Y, X, info
