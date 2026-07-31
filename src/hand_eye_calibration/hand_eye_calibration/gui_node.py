"""
Hand-Eye Calibration GUI Node.
ROS2 node with PyQt5 GUI for multi-robot hand-eye calibration.
"""

import contextlib
import copy
import io
import os
import sys
import threading
import time
from datetime import datetime

# Prevent OpenCV's bundled Qt from conflicting with PyQt5
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)  # noqa: E402

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .calibration import (
    ArUcoDetector,
    load_calibration_result,
    load_pose_pairs,
    plot_calibration_result,
    plot_pose_pairs,
    save_calibration_result,
    save_pose_pairs,
    solve_ax_yb,
    solve_dq_ransac,
    solve_tsai_lenz,
)
from .robot_interface import ROS2TFPoseReader, URDirectPoseReader


class CalibrationNode(Node):
    """ROS2 node for subscribing to camera topics."""

    def __init__(self):
        super().__init__("hand_eye_calibration")

        # Declare parameters
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("marker_size", 6)
        self.declare_parameter("total_markers", 250)
        self.declare_parameter("board_grid_shape", [5, 7])
        self.declare_parameter("marker_length", 0.037)
        self.declare_parameter("marker_separation", 0.003)
        self.declare_parameter("robot_mode", "ur_direct")
        self.declare_parameter("tf_base_frame", "base_link")
        self.declare_parameter("tf_ee_frame", "tool0")
        self.declare_parameter("data_dir", "data/")

        # Camera data
        self.cv_bridge = CvBridge()
        self.current_image = None
        self.on_new_image = None
        self.pending_display = False
        self.camera_matrix = None
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

        # Subscribers
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value

        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 10
        )

    def _image_callback(self, msg):
        self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.on_new_image is not None and not self.pending_display:
            self.pending_display = True
            self.on_new_image()

    def _camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)


class _LineEmitter(io.TextIOBase):
    """stdout 리다이렉터: 한 줄씩 캡처해 pyqtSignal로 전달."""

    def __init__(self, signal):
        self._signal = signal
        self._buf = ""

    def write(self, text):
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self._signal.emit(line)
        return len(text)

    def flush(self):
        pass


class CalibrationWorker(QThread):
    finished = pyqtSignal(dict)
    progress = pyqtSignal(str)

    def __init__(self, algo, robot_T_list, marker_T_list):
        super().__init__()
        self.algo = algo
        self.robot_T_list = robot_T_list
        self.marker_T_list = marker_T_list

    def run(self):
        try:
            with contextlib.redirect_stdout(_LineEmitter(self.progress)):
                if self.algo == "Tsai-Lenz":
                    X = solve_tsai_lenz(self.robot_T_list, self.marker_T_list)
                    self.finished.emit({"success": True, "X": X, "algo": self.algo})
                elif self.algo == "AX=YB":
                    Y, X_base_board, info = solve_ax_yb(
                        self.robot_T_list, self.marker_T_list
                    )
                    self.finished.emit(
                        {
                            "success": True,
                            "X": Y,
                            "X_base_board": X_base_board,
                            "info": info,
                            "algo": self.algo,
                        }
                    )
                else:
                    success, X, rmse, num_inliers = solve_dq_ransac(
                        self.robot_T_list,
                        self.marker_T_list,
                        iterations=200,
                        sample_size=3,
                    )
                    self.finished.emit(
                        {
                            "success": success,
                            "X": X,
                            "rmse": rmse,
                            "num_inliers": num_inliers,
                            "algo": self.algo,
                            "n": len(self.robot_T_list),
                        }
                    )
        except Exception as e:
            self.finished.emit({"error": str(e)})


class HandEyeCalibrationGUI(QtWidgets.QMainWindow):
    """PyQt5 GUI for hand-eye calibration."""

    new_image_signal = pyqtSignal()

    def __init__(self, node: CalibrationNode):
        super().__init__()
        self.node = node
        self.pose_reader = None
        self.robot_T_list = []
        self.marker_T_list = []

        self.new_image_signal.connect(self._update_camera_display)
        self.node.on_new_image = self.new_image_signal.emit

        # ArUco detection throttle: run detect every N frames, reuse last overlay between
        self._frame_idx = 0
        self._last_overlay = None
        self.DETECT_EVERY = 2

        # ArUco detector
        grid_shape = tuple(node.get_parameter("board_grid_shape").value)
        self.aruco = ArUcoDetector(
            marker_size=node.get_parameter("marker_size").value,
            total_markers=node.get_parameter("total_markers").value,
            grid_shape=grid_shape,
            marker_length=node.get_parameter("marker_length").value,
            marker_separation=node.get_parameter("marker_separation").value,
        )

        self._init_ui()
        self._start_timers()

    def _init_ui(self):
        self.setWindowTitle("Hand-Eye Calibration")
        self.setMinimumSize(1200, 800)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # === Top: Robot Connection ===
        robot_group = QtWidgets.QGroupBox("Robot Connection")
        robot_layout = QtWidgets.QHBoxLayout(robot_group)

        # Robot mode
        robot_layout.addWidget(QtWidgets.QLabel("Mode:"))
        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItems(["UR Direct", "ROS2 TF"])
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        robot_layout.addWidget(self.combo_mode)

        sep_r1 = QtWidgets.QFrame()
        sep_r1.setFrameShape(QtWidgets.QFrame.VLine)
        sep_r1.setFrameShadow(QtWidgets.QFrame.Sunken)
        robot_layout.addSpacing(8)
        robot_layout.addWidget(sep_r1)
        robot_layout.addSpacing(8)

        # UR Direct widgets (single read-only socket, pendant-friendly)
        self.ur_direct_widgets = QtWidgets.QWidget()
        ur_direct_layout = QtWidgets.QHBoxLayout(self.ur_direct_widgets)
        ur_direct_layout.setContentsMargins(0, 0, 0, 0)
        ur_direct_layout.setSpacing(4)

        ur_direct_layout.addWidget(QtWidgets.QLabel("IP:"))
        self.edit_ur_ip = QtWidgets.QLineEdit("192.168.1.77")
        self.edit_ur_ip.setFixedWidth(130)
        ur_direct_layout.addWidget(self.edit_ur_ip)

        ur_direct_layout.addSpacing(8)
        ur_direct_layout.addWidget(QtWidgets.QLabel("Port:"))
        self.edit_ur_port = QtWidgets.QLineEdit("30003")
        self.edit_ur_port.setFixedWidth(60)
        ur_direct_layout.addWidget(self.edit_ur_port)

        self.ur_direct_widgets.setSizePolicy(
            QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred
        )
        robot_layout.addWidget(self.ur_direct_widgets)
        self.ur_direct_widgets.hide()

        # ROS2 TF widgets
        self.tf_widgets = QtWidgets.QWidget()
        tf_layout = QtWidgets.QHBoxLayout(self.tf_widgets)
        tf_layout.setContentsMargins(0, 0, 0, 0)
        tf_layout.setSpacing(4)

        tf_layout.addWidget(QtWidgets.QLabel("Base Frame:"))
        self.edit_base_frame = QtWidgets.QLineEdit(
            self.node.get_parameter("tf_base_frame").value
        )
        self.edit_base_frame.setFixedWidth(120)
        tf_layout.addWidget(self.edit_base_frame)

        tf_layout.addSpacing(8)
        tf_layout.addWidget(QtWidgets.QLabel("EE Frame:"))
        self.edit_ee_frame = QtWidgets.QLineEdit(
            self.node.get_parameter("tf_ee_frame").value
        )
        self.edit_ee_frame.setFixedWidth(120)
        tf_layout.addWidget(self.edit_ee_frame)

        self.tf_widgets.setSizePolicy(
            QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred
        )
        robot_layout.addWidget(self.tf_widgets)
        self.tf_widgets.hide()

        # 초기 모드 위젯 가시성 설정 (UR Direct 표시)
        self._on_mode_changed(0)

        sep_r2 = QtWidgets.QFrame()
        sep_r2.setFrameShape(QtWidgets.QFrame.VLine)
        sep_r2.setFrameShadow(QtWidgets.QFrame.Sunken)
        robot_layout.addSpacing(8)
        robot_layout.addWidget(sep_r2)
        robot_layout.addSpacing(8)

        # Connect button
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.setStyleSheet("padding: 4px 14px;")
        self.btn_connect.clicked.connect(self._on_connect)
        robot_layout.addWidget(self.btn_connect)

        robot_layout.addStretch()

        self.label_status = QtWidgets.QLabel("Disconnected")
        self.label_status.setStyleSheet("color: red; font-weight: bold;")
        robot_layout.addWidget(self.label_status)
        layout.addWidget(robot_group, 0)  # stretch=0: fixed height

        # === Middle: Camera + Data Table ===
        middle = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Camera view
        self.camera_label = QtWidgets.QLabel()
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setStyleSheet("background-color: #333;")
        middle.addWidget(self.camera_label)

        # Data table + buttons
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(9, 9, 0, 0)

        self.label_count = QtWidgets.QLabel("Captured: 0 poses")
        right_layout.addWidget(self.label_count)

        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(
            ["Robot Pose (xyz)", "Marker Distance (xyz)"]
        )
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(True)  # row header로 번호 표시
        right_layout.addWidget(self.table)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_capture = QtWidgets.QPushButton("Capture Pose")
        self.btn_capture.setMinimumHeight(48)
        self.btn_capture.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.btn_capture.clicked.connect(self._on_capture)
        self.btn_capture.setEnabled(False)
        btn_row.addWidget(self.btn_capture)

        self.btn_delete = QtWidgets.QPushButton("Delete Selected")
        self.btn_delete.setMinimumHeight(48)
        self.btn_delete.clicked.connect(self._on_delete)
        btn_row.addWidget(self.btn_delete)
        right_layout.addLayout(btn_row)

        middle.addWidget(right_panel)
        middle.setSizes([700, 500])
        layout.addWidget(middle, 1)  # stretch=1: camera area takes all extra space

        # === Bottom: Tools ===
        tools_group = QtWidgets.QGroupBox("Tools")
        tools_group.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed
        )
        tools_layout = QtWidgets.QHBoxLayout(tools_group)

        BTN_H = 36
        BTN_STYLE = "padding: 0 14px;"
        LABEL_W = 70

        # Left: result text
        self.result_text = QtWidgets.QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(130)
        self.result_text.setPlaceholderText("Calibration result will appear here...")
        tools_layout.addWidget(self.result_text, 7)

        # Right: buttons (3 rows with labels)
        btn_panel = QtWidgets.QVBoxLayout()
        btn_panel.setContentsMargins(9, 0, 0, 0)

        # --- CSV ---
        csv_row = QtWidgets.QHBoxLayout()
        csv_row.setSpacing(8)
        csv_label = QtWidgets.QLabel("CSV :")
        csv_label.setFixedWidth(LABEL_W)
        csv_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        csv_row.addWidget(csv_label)
        self.btn_load_data = QtWidgets.QPushButton("Load Data")
        self.btn_load_data.setFixedHeight(BTN_H)
        self.btn_load_data.setStyleSheet(BTN_STYLE)
        self.btn_load_data.clicked.connect(self._on_load_data)
        csv_row.addWidget(self.btn_load_data, 1)
        self.btn_save_data = QtWidgets.QPushButton("Save Data")
        self.btn_save_data.setFixedHeight(BTN_H)
        self.btn_save_data.setStyleSheet(BTN_STYLE)
        self.btn_save_data.clicked.connect(self._on_save_data)
        csv_row.addWidget(self.btn_save_data, 1)
        btn_panel.addLayout(csv_row)

        # --- Algorithm ---
        cal_row = QtWidgets.QHBoxLayout()
        cal_row.setSpacing(8)
        algo_label = QtWidgets.QLabel("Algorithm :")
        algo_label.setFixedWidth(LABEL_W)
        algo_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        cal_row.addWidget(algo_label)
        self.combo_algo = QtWidgets.QComboBox()
        self.combo_algo.addItems(["Tsai-Lenz", "DQ RANSAC", "AX=YB"])
        self.combo_algo.setCurrentIndex(0)
        self.combo_algo.setFixedHeight(BTN_H)
        cal_row.addWidget(self.combo_algo)
        self.btn_calibrate = QtWidgets.QPushButton("Calibrate")
        self.btn_calibrate.setFixedHeight(BTN_H)
        self.btn_calibrate.setStyleSheet(
            "font-size: 14px; font-weight: bold; padding: 0 18px;"
        )
        self.btn_calibrate.clicked.connect(self._on_calibrate)
        cal_row.addWidget(self.btn_calibrate, 1)
        self.btn_save = QtWidgets.QPushButton("Save Result")
        self.btn_save.setFixedHeight(BTN_H)
        self.btn_save.setStyleSheet(BTN_STYLE)
        self.btn_save.clicked.connect(self._on_save_result)
        self.btn_save.setEnabled(False)
        cal_row.addWidget(self.btn_save, 1)
        btn_panel.addLayout(cal_row)

        # --- Visual ---
        plot_row = QtWidgets.QHBoxLayout()
        plot_row.setSpacing(8)
        plot_label = QtWidgets.QLabel("Plot :")
        plot_label.setFixedWidth(LABEL_W)
        plot_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        plot_row.addWidget(plot_label)
        self.btn_plot_result = QtWidgets.QPushButton("Calibration Result")
        self.btn_plot_result.setFixedHeight(BTN_H)
        self.btn_plot_result.setStyleSheet(BTN_STYLE)
        self.btn_plot_result.clicked.connect(self._on_plot_result)
        plot_row.addWidget(self.btn_plot_result, 1)
        self.btn_plot_poses = QtWidgets.QPushButton("Data Poses")
        self.btn_plot_poses.setFixedHeight(BTN_H)
        self.btn_plot_poses.setStyleSheet(BTN_STYLE)
        self.btn_plot_poses.clicked.connect(self._on_plot_poses)
        plot_row.addWidget(self.btn_plot_poses, 1)
        btn_panel.addLayout(plot_row)

        tools_layout.addLayout(btn_panel, 5)

        layout.addWidget(tools_group, 0)  # stretch=0: fixed height

    def _start_timers(self):
        # ROS2 spin on a dedicated thread so callbacks (image, camera_info, TF)
        # are processed immediately instead of being throttled by a Qt timer.
        # Image callback emits new_image_signal across threads -> QueuedConnection
        # posts _update_camera_display onto the Qt main event loop.
        self._ros_spin_thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True
        )
        self._ros_spin_thread.start()

    def _on_mode_changed(self, index):
        self.ur_direct_widgets.setVisible(index == 0)
        self.tf_widgets.setVisible(index == 1)

    def _on_connect(self):
        if self.pose_reader is not None:
            self.pose_reader.disconnect()
            self.pose_reader = None
            self.btn_connect.setText("Connect")
            self.label_status.setText("Disconnected")
            self.label_status.setStyleSheet("color: red; font-weight: bold;")
            self.btn_capture.setEnabled(False)
            return

        try:
            mode_index = self.combo_mode.currentIndex()
            if mode_index == 0:  # UR Direct
                ip = self.edit_ur_ip.text()
                port = int(self.edit_ur_port.text())
                self.pose_reader = URDirectPoseReader(ip, port)
            else:  # ROS2 TF
                base_frame = self.edit_base_frame.text()
                ee_frame = self.edit_ee_frame.text()
                self.pose_reader = ROS2TFPoseReader(self.node, base_frame, ee_frame)

            success = self.pose_reader.connect()
            if success:
                self.btn_connect.setText("Disconnect")
                self.label_status.setText("Connected")
                self.label_status.setStyleSheet("color: green; font-weight: bold;")
                self.btn_capture.setEnabled(True)
            else:
                self.pose_reader = None
                self.label_status.setText("Connection failed")
        except Exception as e:
            self.pose_reader = None
            self.label_status.setText(f"Error: {e}")
            self.node.get_logger().error(f"Connection error: {e}")

    def _update_camera_display(self):
        if self.node.current_image is None:
            self.node.pending_display = False
            return

        try:
            img = self.node.current_image.copy()
            self._frame_idx += 1

            # Run ArUco detection every DETECT_EVERY frames; reuse last overlay in between
            run_detect = (
                self._frame_idx % self.DETECT_EVERY == 0
                and self.node.camera_matrix is not None
            )
            if run_detect:
                success, annotated, _ = self.aruco.detect(
                    img, self.node.camera_matrix, self.node.dist_coeffs
                )
                self._last_overlay = annotated if success else None

            display_img = self._last_overlay if self._last_overlay is not None else img

            # Convert to QPixmap and display
            h, w, c = display_img.shape
            rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
            qimg = QtGui.QImage(rgb.data, w, h, w * c, QtGui.QImage.Format_RGB888)
            scaled = QtGui.QPixmap.fromImage(qimg).scaled(
                self.camera_label.size(),
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.FastTransformation,
            )
            self.camera_label.setPixmap(scaled)
        finally:
            self.node.pending_display = False

    def _check_robot_stable(self, decimals=4):
        """로봇이 정지했는지 확인 (소수점 decimals자리 반올림 비교, 2회)."""
        try:
            prev = np.round(self.pose_reader.get_pose(), decimals)
            time.sleep(0.05)
            curr = np.round(self.pose_reader.get_pose(), decimals)
        except Exception:
            return False
        return np.array_equal(prev, curr)

    def _flash_camera(self, color, duration_ms=500):
        """카메라 영역 테두리를 잠깐 색상으로 바꿔 시각 피드백 제공."""
        self.camera_label.setStyleSheet(
            f"background-color: #333; border: 8px solid {color};"
        )
        QtWidgets.QApplication.processEvents()
        QTimer.singleShot(
            duration_ms,
            lambda: self.camera_label.setStyleSheet("background-color: #333;"),
        )

    def _on_capture(self):
        if self.pose_reader is None or self.node.current_image is None:
            return
        if self.node.camera_matrix is None:
            self.result_text.setText("Camera info not received yet.")
            return

        # 1) Check robot is stable
        self.result_text.setText("Waiting for robot to stop...")
        QtWidgets.QApplication.processEvents()
        if not self._check_robot_stable():
            self._flash_camera("red")
            self.result_text.setText("Robot is still moving! Wait until fully stopped.")
            return

        # 2) Background spin thread keeps current_image fresh; no explicit wait needed.

        # 3) Get robot pose and marker pose at the same moment
        try:
            robot_T = self.pose_reader.get_pose()
        except Exception as e:
            self._flash_camera("red")
            self.result_text.setText(f"Failed to get robot pose: {e}")
            return

        img = self.node.current_image.copy()
        success, _, marker_T = self.aruco.detect(
            img, self.node.camera_matrix, self.node.dist_coeffs
        )
        if not success:
            self._flash_camera("red")
            self.result_text.setText(
                "Marker not detected. Move robot so marker is visible."
            )
            return

        # Store
        self.robot_T_list.append(copy.deepcopy(robot_T))
        self.marker_T_list.append(copy.deepcopy(marker_T))

        # Update table
        idx = len(self.robot_T_list)
        self.table.setRowCount(idx)
        xyz = f"[{robot_T[0,3]:.3f}, {robot_T[1,3]:.3f}, {robot_T[2,3]:.3f}]"
        self.table.setItem(idx - 1, 0, QtWidgets.QTableWidgetItem(xyz))
        m_xyz = f"[{marker_T[0,3]:.3f}, {marker_T[1,3]:.3f}, {marker_T[2,3]:.3f}]"
        self.table.setItem(idx - 1, 1, QtWidgets.QTableWidgetItem(m_xyz))

        self.label_count.setText(f"Captured: {idx} poses")
        self.result_text.setText(
            f"Pose #{idx} captured. Robot [{robot_T[0,3]:.4f}, {robot_T[1,3]:.4f}, {robot_T[2,3]:.4f}]"
        )
        self._flash_camera("lime")
        self._update_pose_plot()

    def _on_delete(self):
        rows = sorted(
            set(idx.row() for idx in self.table.selectedIndexes()), reverse=True
        )
        for row in rows:
            self.robot_T_list.pop(row)
            self.marker_T_list.pop(row)
            self.table.removeRow(row)

        self.label_count.setText(f"Captured: {len(self.robot_T_list)} poses")
        self._update_pose_plot()

    def _on_calibrate(self):
        n = len(self.robot_T_list)
        if n < 3:
            self.result_text.setText(f"Need at least 3 poses. Currently have {n}.")
            return

        self.btn_calibrate.setEnabled(False)
        self.result_text.setText("Calibrating...")

        algo = self.combo_algo.currentText()
        self._cal_worker = CalibrationWorker(
            algo, list(self.robot_T_list), list(self.marker_T_list)
        )
        self._cal_worker.finished.connect(self._on_calibration_done)
        self._cal_worker.progress.connect(self.result_text.append)
        self._cal_worker.start()

    def _on_calibration_done(self, result):
        self.btn_calibrate.setEnabled(True)
        if "error" in result:
            self.result_text.setText(f"Calibration error: {result['error']}")
            self.node.get_logger().error(f"Calibration error: {result['error']}")
            return

        algo = result["algo"]
        if not result["success"]:
            self.result_text.setText(
                "DQ RANSAC failed. Try collecting more diverse poses."
            )
            return

        X = result["X"]
        self.calibration_result = X
        self.btn_save.setEnabled(True)

        mat_str = np.array2string(X, precision=6, suppress_small=True)
        extra_str = ""
        if algo == "Tsai-Lenz":
            rmse_str = ""
        elif algo == "AX=YB":
            info = result["info"]
            rmse_str = (
                f"\nResidual RMSE rot: {info['rot_rmse_deg']:.3f} deg"
                f"  trans: {info['trans_rmse_mm']:.3f} mm  (n={info['n']})"
            )
            bw_str = np.array2string(
                result["X_base_board"], precision=6, suppress_small=True
            )
            extra_str = f"\n\nBase to board (T_base_board, for validation):\n{bw_str}"
        else:
            rmse_pos, rmse_ori = result["rmse"]
            rmse_str = f"\nRMSE pos: {rmse_pos:.6f}  ori: {rmse_ori:.6f}  |  Inliers: {result['num_inliers']}/{result['n']}"

        self.result_text.setText(
            f"Algorithm: {algo}{rmse_str}\n"
            f"X (end-effector to camera, T_hand_eye):\n{mat_str}"
            f"{extra_str}"
        )

    def _on_save_result(self):
        if not hasattr(self, "calibration_result"):
            return
        data_dir = self.node.get_parameter("data_dir").value
        ts = datetime.now().strftime("%y%m%d_%H%M")
        algo = self.combo_algo.currentText()

        filepath = os.path.join(data_dir, f"calibration_result_{ts}_{algo}.txt")
        save_calibration_result(filepath, self.calibration_result, algo)
        self.result_text.append(f"\nResult saved to: {filepath}")

    def _on_save_data(self):
        if not self.robot_T_list:
            self.result_text.setText("No data to save.")
            return
        data_dir = self.node.get_parameter("data_dir").value
        ts = datetime.now().strftime("%y%m%d_%H%M")
        filepath = os.path.join(data_dir, f"pose_pairs_{ts}.csv")
        mode_index = self.combo_mode.currentIndex()
        if mode_index == 0:
            robot_name = "ur_direct"
        else:
            robot_name = "ros2_tf"
        save_pose_pairs(
            filepath,
            self.robot_T_list,
            self.marker_T_list,
            metadata={"robot_type": robot_name},
        )
        self.result_text.setText(
            f"Data saved to: {filepath} ({len(self.robot_T_list)} poses)"
        )

    def _on_load_data(self):
        filepath, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Pose Data", "", "CSV Files (*.csv);;All Files (*)"
        )
        if not filepath:
            return
        robot_T_list, marker_T_list = load_pose_pairs(filepath)
        self.robot_T_list = robot_T_list
        self.marker_T_list = marker_T_list

        # Update table
        self.table.setRowCount(len(robot_T_list))
        for i, (rt, mt) in enumerate(zip(robot_T_list, marker_T_list)):
            xyz = f"[{rt[0,3]:.3f}, {rt[1,3]:.3f}, {rt[2,3]:.3f}]"
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(xyz))
            m_xyz = f"[{mt[0,3]:.3f}, {mt[1,3]:.3f}, {mt[2,3]:.3f}]"
            self.table.setItem(i, 1, QtWidgets.QTableWidgetItem(m_xyz))

        self.label_count.setText(f"Captured: {len(robot_T_list)} poses")
        self.result_text.setText(f"Loaded {len(robot_T_list)} poses from: {filepath}")

    def _on_plot_result(self):
        """Visualize calibration result or load one from file."""
        X = getattr(self, "calibration_result", None)
        if X is None:
            filepath, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Load Calibration Result",
                "",
                "Text Files (*.txt);;All Files (*)",
            )
            if not filepath:
                return
            try:
                X, meta = load_calibration_result(filepath)
                algo = meta.get("algorithm", "Unknown")
            except Exception as e:
                self.result_text.setText(f"Failed to load result: {e}")
                return
        else:
            algo = self.combo_algo.currentText()

        self._plot_dlg = plot_calibration_result(
            X,
            title=f"T_hand_eye ({algo})",
            parent=self,
        )

    def _on_plot_poses(self):
        """Visualize captured pose pairs (current data or load from file)."""
        robot_T = self.robot_T_list
        marker_T = self.marker_T_list

        if not robot_T:
            filepath, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Load Pose Data",
                "",
                "CSV Files (*.csv);;All Files (*)",
            )
            if not filepath:
                return
            try:
                robot_T, marker_T = load_pose_pairs(filepath)
            except Exception as e:
                self.result_text.setText(f"Failed to load poses: {e}")
                return

        if not robot_T:
            self.result_text.setText("No pose data to visualize.")
            return

        self._pose_dlg = plot_pose_pairs(robot_T, marker_T, parent=self)

    def _update_pose_plot(self):
        """Update pose visualization dialog if it is open."""
        dlg = getattr(self, "_pose_dlg", None)
        if dlg is not None and dlg.isVisible():
            dlg.update(self.robot_T_list, self.marker_T_list)

    def closeEvent(self, event):
        if self.pose_reader is not None:
            self.pose_reader.disconnect()
        event.accept()
        QtWidgets.QApplication.quit()


def main(args=None):
    # QApplication must be created before rclpy.init to avoid signal handler conflicts
    app = QtWidgets.QApplication(sys.argv)

    rclpy.init(args=args)
    node = CalibrationNode()

    gui = HandEyeCalibrationGUI(node)
    gui.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
