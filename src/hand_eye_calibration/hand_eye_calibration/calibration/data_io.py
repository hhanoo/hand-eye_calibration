"""
Pose data I/O for hand-eye calibration.
Saves/loads pose pairs and calibration results in CSV format.
"""

import csv
import os
from datetime import datetime

import numpy as np


def save_pose_pairs(filepath, robot_T_list, marker_T_list, metadata=None):
    """
    Save pose pairs to CSV file.

    Format: header with metadata, then one row per sample.
    Each row = 32 floats (robot 4x4 row-major + marker 4x4 row-major).

    Args:
        filepath: Output CSV path
        robot_T_list: list of 4x4 robot transforms
        marker_T_list: list of 4x4 marker transforms
        metadata: optional dict with extra info (robot_type, date, etc.)
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)

        # Header
        meta = metadata or {}
        meta.setdefault("date", datetime.now().isoformat())
        meta.setdefault("num_samples", len(robot_T_list))
        writer.writerow([f"# {k}={v}" for k, v in meta.items()])
        writer.writerow(
            ["# robot_T (16 floats, row-major)", "marker_T (16 floats, row-major)"]
        )

        # Data
        for robot_T, marker_T in zip(robot_T_list, marker_T_list):
            row = list(robot_T.flatten()) + list(marker_T.flatten())
            writer.writerow(row)


def load_pose_pairs(filepath):
    """
    Load pose pairs from CSV file.

    Returns:
        (robot_T_list, marker_T_list) - lists of 4x4 numpy arrays
    """
    robot_T_list = []
    marker_T_list = []

    with open(filepath, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Skip comment/header lines
            if not row or str(row[0]).startswith("#"):
                continue

            values = [float(v) for v in row]
            if len(values) != 32:
                continue

            robot_T = np.array(values[:16]).reshape(4, 4)
            marker_T = np.array(values[16:]).reshape(4, 4)
            robot_T_list.append(robot_T)
            marker_T_list.append(marker_T)

    return robot_T_list, marker_T_list


def save_calibration_result(filepath, X, algorithm, rmse=None, num_inliers=None):
    """
    Save calibration result (4x4 transform) to text file.

    Args:
        filepath: Output path
        X: 4x4 calibration result matrix
        algorithm: Algorithm name string
        rmse: Optional RMSE value
        num_inliers: Optional inlier count
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    with open(filepath, "w") as f:
        f.write(f"# Hand-Eye Calibration Result\n")
        f.write(f"# Algorithm: {algorithm}\n")
        f.write(f"# Date: {datetime.now().isoformat()}\n")
        if rmse is not None:
            f.write(f"# RMSE: {rmse}\n")
        if num_inliers is not None:
            f.write(f"# Inliers: {num_inliers}\n")
        f.write(f"# 4x4 Transform (end-effector to camera, T_hand_eye):\n")
        for row in X:
            f.write(",".join(f"{v:.10f}" for v in row) + "\n")


def load_calibration_result(filepath):
    """
    Load calibration result (4x4 transform) from text file.

    Returns:
        (X, metadata) - 4x4 numpy array and dict with algorithm, date, etc.
    """
    rows = []
    metadata = {}

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("# Algorithm:"):
                metadata["algorithm"] = line.split(":", 1)[1].strip()
            elif line.startswith("# Date:"):
                metadata["date"] = line.split(":", 1)[1].strip()
            elif line.startswith("# RMSE:"):
                metadata["rmse"] = line.split(":", 1)[1].strip()
            elif line.startswith("# Inliers:"):
                metadata["inliers"] = line.split(":", 1)[1].strip()
            elif line.startswith("#"):
                continue
            elif line:
                rows.append([float(v) for v in line.split(",")])

    X = np.array(rows).reshape(4, 4)
    return X, metadata
