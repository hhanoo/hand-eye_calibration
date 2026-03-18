import numpy as np
import pyrealsense2 as rs


class RSSensor:

    def __init__(self, sensor_index=0, serial_number=None):
        """
        Initialize RealSense sensor with a specific serial number.

        Args:
            sensor_index (int): Index of the sensor to connect to
            serial_number (str): Serial number of the sensor to connect to
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.intr_params = None
        self.isRunning = False
        self.device_list = self._get_connected_devices()

        if not self.device_list:
            raise RuntimeError("[RSSensor: ERROR] No RealSense devices found!")

        if serial_number is not None:
            if serial_number not in self.device_list:
                raise RuntimeError(f"[RSSensor: ERROR] Device with serial number {serial_number} not found!")
            self.serial_number = serial_number
            self.device_index = self.device_list.index(serial_number)
        else:
            self.device_index = sensor_index
            self.serial_number = self.device_list[self.device_index]

        print(f"[RSSensor: INFO] Attempting to connect to device: {self.serial_number}")
        self.config.enable_device(self.serial_number)

    @staticmethod
    def _get_connected_devices():
        """
        Retrieve the list of connected RealSense devices.

        Returns:
            list: List of connected RealSense device serial numbers
        """
        context = rs.context()
        devices = context.query_devices()
        return [dev.get_info(rs.camera_info.serial_number) for dev in devices]

    def get_device_sn(self):
        """
        Retrieve the device serial number.

        Returns:
            str: Device serial number
        """
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        serial_number = device.get_info(rs.camera_info.serial_number)
        print(f"Device Serial Number: {serial_number}")
        return serial_number

    def init(self):
        """
        Initialize the sensor and retrieve intrinsic parameters.

        Returns:
            bool: True if initialization is successful, False otherwise
        """
        while self.device_index < len(self.device_list):
            try:
                # Stop pipeline if it's already running
                if self.isRunning:
                    self.pipeline.stop()
                    self.isRunning = False

                # Create new config for each attempt
                self.config = rs.config()
                self.config.enable_device(self.serial_number)
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

                profile = self.pipeline.start(self.config)
                color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.intr_params = color_stream.get_intrinsics()
                self.isRunning = True
                print(f"[RSSensor: INFO] Camera {self.serial_number} started.")
                return True

            except Exception as e:
                print(f"[RSSensor: ERROR] Failed to connect to device {self.serial_number}: {e}")

                # Clean up pipeline if it was started
                try:
                    if self.isRunning:
                        self.pipeline.stop()
                        self.isRunning = False
                except:
                    pass

                # Try next device
                self.device_index += 1
                if self.device_index < len(self.device_list):
                    self.serial_number = self.device_list[self.device_index]
                    print(f"[RSSensor: INFO] Retrying with device: {self.serial_number}")
                else:
                    print("[RSSensor: ERROR] No available RealSense devices. Exiting.")
                    return False  # 모든 장치 연결 실패

        return False  # 모든 장치 연결 실패

    def get_data(self):
        """
        Retrieve RGB and depth data from the sensor.

        Returns:
            rgb_data: RGB image
            depth_data: Depth image
        """
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            rgb_data = np.asanyarray(color_frame.get_data())
            depth_data = np.asanyarray(depth_frame.get_data())
            return rgb_data, depth_data

        except Exception as e:
            print(f"[RSSensor: WARNING] Pipeline is not running. Call init() first.: {e}")
            return None, None

    def stop(self):
        """
        Stop the sensor.

        Returns:
            None
        """
        self.pipeline.stop()
        self.isRunning = False
        print(f"[RSSensor: INFO] Camera {self.serial_number} stopped.")

    def get_camera_parameters(self):
        """
        Get camera parameters.

        Returns:
            camera_matrix: Camera intrinsic parameter matrix
            camera_params: Camera intrinsic parameter tuple
        """
        if self.intr_params is None:
            raise RuntimeError("[RSSensor: ERROR] Intrinsic parameters are not available. Start the camera first.")

        ppx, ppy, fx, fy = self.intr_params.ppx, self.intr_params.ppy, self.intr_params.fx, self.intr_params.fy
        camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])
        camera_params = (camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2])
        return camera_matrix, camera_params
