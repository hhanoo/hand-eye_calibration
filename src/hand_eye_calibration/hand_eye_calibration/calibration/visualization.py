"""
Visualization utilities for hand-eye calibration.
3D plots for calibration results and captured pose pairs, embedded in PyQt5 dialogs.
"""

import matplotlib
import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import QtWidgets

matplotlib.use("QtAgg")


def _draw_frame(ax, T, label, axis_length=0.06, linewidth=2):
    """Draw a coordinate frame (RGB = XYZ) at the given 4x4 transform."""
    origin = T[:3, 3]
    R = T[:3, :3]
    colors = ["r", "g", "b"]
    for i, c in enumerate(colors):
        ax.quiver(
            *origin,
            *(R[:, i] * axis_length),
            color=c,
            arrow_length_ratio=0.15,
            linewidth=linewidth,
        )


def _set_equal_aspect(ax, points):
    """Set equal aspect ratio for 3D axes based on data range."""
    all_pts = np.array(points)
    center = all_pts.mean(axis=0)
    max_range = (all_pts.max(axis=0) - all_pts.min(axis=0)).max() / 2
    margin = max(max_range * 1.5, 0.05)
    ax.set_xlim(center[0] - margin, center[0] + margin)
    ax.set_ylim(center[1] - margin, center[1] + margin)
    ax.set_zlim(center[2] - margin, center[2] + margin)


class _PlotDialog(QtWidgets.QDialog):
    """Resizable dialog that hosts a matplotlib figure canvas."""

    def __init__(self, fig, title, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(900, 700)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.canvas = FigureCanvasQTAgg(fig)
        layout.addWidget(self.canvas)
        self.canvas.draw()


def plot_calibration_result(X, title=None, parent=None):
    """
    Plot end-effector frame and camera frame from calibration result.

    Args:
        X: 4x4 transform matrix (end-effector to camera, T_hand_eye)
        title: Optional plot title
        parent: Parent QWidget (keeps dialog alive)
    """
    t = X[:3, 3]

    fig = Figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    # End-effector frame (origin)
    ee = np.eye(4)
    _draw_frame(ax, ee, "End-Effector", axis_length=0.08, linewidth=2.5)
    ax.scatter(0, 0, 0, color="k", s=50, zorder=5)
    ax.text(0, 0, 0, "  End-Effector", fontsize=10, fontweight="bold")

    # Camera frame
    _draw_frame(ax, X, "Camera", axis_length=0.06, linewidth=2.5)
    ax.scatter(t[0], t[1], t[2], color="m", s=50, zorder=5)
    ax.text(t[0], t[1], t[2], "  Camera", fontsize=10, fontweight="bold")

    # Connection line
    ax.plot([0, t[0]], [0, t[1]], [0, t[2]], "k--", alpha=0.4, linewidth=1)

    # Translation info
    dist = np.linalg.norm(t)
    info = (
        f"Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m\n"
        f"Distance: {dist:.4f} m"
    )
    ax.text2D(
        0.02,
        0.95,
        info,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title or "Hand-Eye Calibration Result")

    _set_equal_aspect(ax, [[0, 0, 0], t.tolist()])
    ax.view_init(elev=25, azim=-55)
    fig.tight_layout()

    dlg = _PlotDialog(fig, title or "Calibration Result", parent)
    dlg.show()
    return dlg


class PosePairDialog(QtWidgets.QDialog):
    """Live-updating 3D visualization of captured pose pairs."""

    def __init__(self, robot_T_list, marker_T_list, title=None, parent=None):
        super().__init__(parent)
        self._title = title or "Pose Pairs"
        self.setWindowTitle(self._title)
        self.resize(1200, 600)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._fig = Figure(figsize=(14, 6))
        self._canvas = FigureCanvasQTAgg(self._fig)
        layout.addWidget(self._canvas)

        self.update(robot_T_list, marker_T_list)

    def update(self, robot_T_list, marker_T_list):
        """Redraw plots with current data, preserving camera angles."""
        # Save current view angles and limits before clearing
        views = {}
        for ax in self._fig.axes:
            views[ax.get_title()] = {
                "elev": ax.elev,
                "azim": ax.azim,
                "xlim": ax.get_xlim(),
                "ylim": ax.get_ylim(),
                "zlim": ax.get_zlim(),
            }

        self._fig.clear()

        if not robot_T_list:
            self._canvas.draw()
            return

        n = len(robot_T_list)
        colors = np.arange(n)

        # --- Left: Robot EE poses (base frame) ---
        ax1 = self._fig.add_subplot(121, projection="3d")

        robot_positions = []
        for T in robot_T_list:
            robot_positions.append(T[:3, 3])
            _draw_frame(ax1, T, "", axis_length=0.02, linewidth=1)

        robot_positions = np.array(robot_positions)
        ax1.scatter(
            robot_positions[:, 0],
            robot_positions[:, 1],
            robot_positions[:, 2],
            c=colors,
            cmap="viridis",
            s=30,
            zorder=5,
        )
        ax1.plot(
            robot_positions[:, 0],
            robot_positions[:, 1],
            robot_positions[:, 2],
            "k-",
            alpha=0.3,
            linewidth=0.8,
        )

        _draw_frame(ax1, np.eye(4), "Base", axis_length=0.05, linewidth=2.5)
        ax1.text(0, 0, 0, "  Base", fontsize=9, fontweight="bold")

        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.set_title("End-Effector Poses (Base Frame)")
        _set_equal_aspect(ax1, robot_positions.tolist() + [[0, 0, 0]])
        if ax1.get_title() in views:
            v = views[ax1.get_title()]
            ax1.view_init(elev=v["elev"], azim=v["azim"])
            ax1.set_xlim(v["xlim"])
            ax1.set_ylim(v["ylim"])
            ax1.set_zlim(v["zlim"])

        # --- Right: Camera poses (marker frame, marker fixed at origin) ---
        ax2 = self._fig.add_subplot(122, projection="3d")

        cam_positions = []
        for T_cam2marker in marker_T_list:
            T_marker2cam = np.linalg.inv(T_cam2marker)
            cam_positions.append(T_marker2cam[:3, 3])
            _draw_frame(ax2, T_marker2cam, "", axis_length=0.02, linewidth=1)

        cam_positions = np.array(cam_positions)
        ax2.scatter(
            cam_positions[:, 0],
            cam_positions[:, 1],
            cam_positions[:, 2],
            c=colors,
            cmap="viridis",
            s=30,
            zorder=5,
        )
        ax2.plot(
            cam_positions[:, 0],
            cam_positions[:, 1],
            cam_positions[:, 2],
            "k-",
            alpha=0.3,
            linewidth=0.8,
        )

        _draw_frame(ax2, np.eye(4), "Marker", axis_length=0.05, linewidth=2.5)
        ax2.text(0, 0, 0, "  Marker (fixed)", fontsize=9, fontweight="bold")

        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.set_zlabel("Z (m)")
        ax2.set_title("Camera Poses (Marker Frame)")
        _set_equal_aspect(ax2, cam_positions.tolist() + [[0, 0, 0]])

        if ax2.get_title() in views:
            v = views[ax2.get_title()]
            ax2.view_init(elev=v["elev"], azim=v["azim"])
            ax2.set_xlim(v["xlim"])
            ax2.set_ylim(v["ylim"])
            ax2.set_zlim(v["zlim"])

        self._fig.suptitle(f"{self._title} ({n} samples)", fontsize=13)
        self._fig.tight_layout()
        self._canvas.draw()


def plot_pose_pairs(robot_T_list, marker_T_list, title=None, parent=None):
    """Create and show a live-updating pose pair visualization dialog."""
    dlg = PosePairDialog(robot_T_list, marker_T_list, title, parent)
    dlg.show()
    return dlg
