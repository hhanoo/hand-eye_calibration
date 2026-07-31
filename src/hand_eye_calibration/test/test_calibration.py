"""
Tests for the hand-eye calibration solvers.

Synthetic data follows the noise model of Ha (IEEE T-RO 2023):
robot poses are noiseless, camera measurements carry right-translated
noise at the board (target) frame: B_i = B̄_i · M_i.
"""

import contextlib
import io

import cv2
import numpy as np
import pytest

from hand_eye_calibration.calibration import (
    solve_ax_yb,
    solve_dq_ransac,
    solve_tsai_lenz,
)


def _transform(axis_angle, t):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(axis_angle, dtype=float))[0]
    T[:3, 3] = t
    return T


def _rot_error_deg(R1, R2):
    cos = (np.trace(R1.T @ R2) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def _make_dataset(n, rng, rot_noise_deg=0.0, trans_noise_m=0.0):
    """Poses satisfying A_i · Y · B_i = X, with noise only on the board side."""
    Y_true = _transform([0.1, -0.2, 0.3], [0.05, -0.03, 0.08])
    X_true = _transform([0.0, 0.0, 1.2], [0.6, 0.1, 0.2])

    robot_T_list, marker_T_list = [], []
    for _ in range(n):
        A = _transform(rng.uniform(-1.2, 1.2, 3), rng.uniform(-0.3, 0.3, 3))
        A[2, 3] += 0.5
        B = np.linalg.inv(Y_true) @ np.linalg.inv(A) @ X_true
        B = B @ _transform(
            rng.normal(0.0, np.radians(rot_noise_deg), 3),
            rng.normal(0.0, trans_noise_m, 3),
        )
        robot_T_list.append(A)
        marker_T_list.append(B)
    return robot_T_list, marker_T_list, Y_true, X_true


def test_tsai_lenz_recovers_the_hand_eye_transform():
    rng = np.random.default_rng(10)
    robot_T, marker_T, Y_true, _ = _make_dataset(10, rng)

    Y = solve_tsai_lenz(robot_T, marker_T)

    assert _rot_error_deg(Y[:3, :3], Y_true[:3, :3]) < 1e-4
    assert np.linalg.norm(Y[:3, 3] - Y_true[:3, 3]) < 1e-6


def test_dq_ransac_recovers_the_hand_eye_transform():
    rng = np.random.default_rng(11)
    robot_T, marker_T, Y_true, _ = _make_dataset(10, rng)

    with contextlib.redirect_stdout(io.StringIO()):
        success, Y, _rmse, inliers = solve_dq_ransac(robot_T, marker_T)

    assert success
    assert inliers == len(robot_T)
    assert _rot_error_deg(Y[:3, :3], Y_true[:3, :3]) < 1e-3
    assert np.linalg.norm(Y[:3, 3] - Y_true[:3, 3]) < 1e-5


def test_all_solvers_agree_under_board_side_noise():
    # Same input, three formulations: the results must stay mutually
    # consistent, so a regression in any one solver shows up here.
    rng = np.random.default_rng(12)
    robot_T, marker_T, _, _ = _make_dataset(
        30, rng, rot_noise_deg=1.0, trans_noise_m=0.003
    )

    tsai = solve_tsai_lenz(robot_T, marker_T)
    with contextlib.redirect_stdout(io.StringIO()):
        _, dq, _, _ = solve_dq_ransac(robot_T, marker_T)
    ax_yb, _, _ = solve_ax_yb(robot_T, marker_T)

    for other in (dq, ax_yb):
        assert _rot_error_deg(tsai[:3, :3], other[:3, :3]) < 2.0
        assert np.linalg.norm(tsai[:3, 3] - other[:3, 3]) < 0.02


def test_ax_yb_recovers_exact_solution_from_noiseless_data():
    rng = np.random.default_rng(0)
    robot_T, marker_T, Y_true, X_true = _make_dataset(10, rng)

    Y, X, info = solve_ax_yb(robot_T, marker_T)

    assert _rot_error_deg(Y[:3, :3], Y_true[:3, :3]) < 1e-4
    assert np.linalg.norm(Y[:3, 3] - Y_true[:3, 3]) < 1e-6
    assert _rot_error_deg(X[:3, :3], X_true[:3, :3]) < 1e-4
    assert np.linalg.norm(X[:3, 3] - X_true[:3, 3]) < 1e-6


def test_ax_yb_recovers_solution_under_board_side_noise():
    rng = np.random.default_rng(1)
    robot_T, marker_T, Y_true, X_true = _make_dataset(
        40, rng, rot_noise_deg=1.0, trans_noise_m=0.003
    )

    Y, X, info = solve_ax_yb(robot_T, marker_T)

    assert _rot_error_deg(Y[:3, :3], Y_true[:3, :3]) < 0.5
    assert np.linalg.norm(Y[:3, 3] - Y_true[:3, 3]) < 0.005
    assert _rot_error_deg(X[:3, :3], X_true[:3, :3]) < 0.5
    assert np.linalg.norm(X[:3, 3] - X_true[:3, 3]) < 0.005


def test_ax_yb_reports_residual_statistics():
    rng = np.random.default_rng(2)
    robot_T, marker_T, _, _ = _make_dataset(
        20, rng, rot_noise_deg=1.0, trans_noise_m=0.003
    )

    _, _, info = solve_ax_yb(robot_T, marker_T)

    # Residual stats should reflect the injected noise level (~1 deg / ~3 mm)
    assert 0.2 < info["rot_rmse_deg"] < 3.0
    assert 0.5 < info["trans_rmse_mm"] < 10.0
    assert info["n"] == 20


def test_ax_yb_refinement_improves_over_initial_guess():
    rng = np.random.default_rng(3)
    robot_T, marker_T, Y_true, _ = _make_dataset(
        40, rng, rot_noise_deg=1.0, trans_noise_m=0.003
    )

    _, _, info = solve_ax_yb(robot_T, marker_T)

    # Refinement minimizes the combined weighted MLE cost; individual
    # rotation/translation RMSE components may trade off against each other.
    assert info["cost"] <= info["init_cost"] + 1e-12


def test_ax_yb_rejects_insufficient_poses():
    rng = np.random.default_rng(4)
    robot_T, marker_T, _, _ = _make_dataset(2, rng)

    with pytest.raises(ValueError):
        solve_ax_yb(robot_T, marker_T)
