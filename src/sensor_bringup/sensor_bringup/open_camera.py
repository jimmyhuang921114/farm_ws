#!/usr/bin/env python3

import glob
import re
import time
from pathlib import Path
from typing import List, Optional, Tuple
from urllib.parse import urlparse

import cv2
import pyudev
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


class UsbCameraPublisher(Node):

    def __init__(self) -> None:
        super().__init__('decxin_camera')

        # USB device identification
        self.declare_parameter('vendor_id', '1bcf')
        self.declare_parameter('product_id', '2cd1')

        # ROS topic settings
        self.declare_parameter(
            'image_topic',
            '/decxin_camera/image_raw',
        )
        self.declare_parameter(
            'frame_id',
            'decxin_camera_link',
        )

        # Camera settings
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('backend', 'V4L2')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('timing_report_interval', 10.0)

        # Reconnection settings
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('max_read_failures', 10)

        self.vendor_id = str(
            self.get_parameter('vendor_id').value
        ).lower()

        self.product_id = str(
            self.get_parameter('product_id').value
        ).lower()

        self.image_topic = str(
            self.get_parameter('image_topic').value
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.width = int(
            self.get_parameter('width').value
        )

        self.height = int(
            self.get_parameter('height').value
        )

        self.fps = float(
            self.get_parameter('fps').value
        )

        self.fourcc = str(
            self.get_parameter('fourcc').value
        )

        self.backend = str(
            self.get_parameter('backend').value
        ).upper()

        self.camera_info_url = str(
            self.get_parameter('camera_info_url').value
        )

        self.timing_report_interval = float(
            self.get_parameter('timing_report_interval').value
        )

        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )

        self.max_read_failures = int(
            self.get_parameter('max_read_failures').value
        )

        if self.fps <= 0.0:
            self.fps = 30.0

        self.udev_context = pyudev.Context()
        self.bridge = CvBridge()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Image,
            self.image_topic,
            image_qos,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            '/decxin_camera/camera_info',
            image_qos,
        )
        self.camera_info = self.load_camera_info(self.camera_info_url)

        self.capture: Optional[cv2.VideoCapture] = None
        self.active_device: Optional[str] = None

        self.last_search_time = 0.0
        self.last_warning_time = 0.0
        self.read_failure_count = 0
        self.frame_count = 0
        self.captured_count = 0
        self.published_count = 0
        self.dropped_count = 0
        self.timing_window_start = time.monotonic()
        self.read_duration = 0.0
        self.bridge_duration = 0.0
        self.publish_duration = 0.0

        timer_period = max(1.0 / self.fps, 0.01)

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Looking for USB camera '
            f'{self.vendor_id}:{self.product_id}'
        )

        self.get_logger().info(
            f'Image output topic: {self.image_topic}'
        )

        if self.camera_info is None:
            self.get_logger().warn(
                'CameraInfo: UNCALIBRATED; camera_info_url is empty or invalid'
            )

    @staticmethod
    def natural_sort_key(path: str):
        return [
            int(part) if part.isdigit() else part.lower()
            for part in re.split(r'(\d+)', path)
        ]

    def log_warning_throttled(
        self,
        message: str,
        interval: float = 3.0,
    ) -> None:
        current_time = time.monotonic()

        if current_time - self.last_warning_time >= interval:
            self.get_logger().warn(message)
            self.last_warning_time = current_time

    def get_usb_ids(
        self,
        video_path: str,
    ) -> Tuple[Optional[str], Optional[str]]:
        """
        Find USB VID/PID for /dev/videoN or /host/dev/videoN.

        Prefer sysfs because udevadm/pyudev databases may be incomplete
        inside Docker containers.
        """
        video_name = video_path.rsplit('/', 1)[-1]

        sysfs_device = (
            Path('/sys/class/video4linux')
            / video_name
            / 'device'
        )

        # --------------------------------------------------------
        # Primary method: walk up the sysfs hierarchy until the
        # USB device containing idVendor and idProduct is found.
        # --------------------------------------------------------
        try:
            current = sysfs_device.resolve(strict=True)

            while True:
                vendor_file = current / 'idVendor'
                product_file = current / 'idProduct'

                if vendor_file.is_file() and product_file.is_file():
                    vendor_id = (
                        vendor_file.read_text()
                        .strip()
                        .lower()
                    )

                    product_id = (
                        product_file.read_text()
                        .strip()
                        .lower()
                    )

                    return vendor_id, product_id

                parent = current.parent

                if parent == current:
                    break

                current = parent

        except (OSError, RuntimeError):
            pass

        # --------------------------------------------------------
        # Fallback: pyudev. This generally works for paths directly
        # under /dev, but may fail for /host/dev paths in Docker.
        # --------------------------------------------------------
        try:
            if video_path.startswith('/dev/'):
                video_device = pyudev.Device.from_device_file(
                    self.udev_context,
                    video_path,
                )

                devices = [video_device]
                devices.extend(list(video_device.ancestors))

                for device in devices:
                    vendor_id = (
                        device.properties.get('ID_VENDOR_ID')
                        or device.properties.get('ID_USB_VENDOR_ID')
                    )

                    product_id = (
                        device.properties.get('ID_MODEL_ID')
                        or device.properties.get('ID_USB_MODEL_ID')
                    )

                    if vendor_id and product_id:
                        return (
                            str(vendor_id).lower(),
                            str(product_id).lower(),
                        )

                    try:
                        vendor_attribute = (
                            device.attributes.get('idVendor')
                        )
                        product_attribute = (
                            device.attributes.get('idProduct')
                        )

                        if (
                            vendor_attribute is not None
                            and product_attribute is not None
                        ):
                            return (
                                vendor_attribute.decode(
                                    errors='ignore'
                                ).lower(),
                                product_attribute.decode(
                                    errors='ignore'
                                ).lower(),
                            )

                    except (AttributeError, OSError):
                        continue

        except Exception as error:
            self.get_logger().debug(
                f'Cannot inspect {video_path} with pyudev: '
                f'{error}'
            )

        return None, None

    def find_candidate_devices(self) -> List[str]:
        candidates: List[str] = []

        video_devices = sorted(
            glob.glob('/dev/video*'),
            key=self.natural_sort_key,
        )

        for video_path in video_devices:
            vendor_id, product_id = self.get_usb_ids(
                video_path
            )

            if (
                vendor_id == self.vendor_id
                and product_id == self.product_id
            ):
                candidates.append(video_path)

        return candidates

    def configure_capture(
        self,
        capture: cv2.VideoCapture,
        fourcc: Optional[str],
    ) -> None:
        results = {
            'buffersize': capture.set(cv2.CAP_PROP_BUFFERSIZE, 1),
        }

        if fourcc and len(fourcc) == 4:
            results['fourcc'] = capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*fourcc),
            )

        results['width'] = capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            float(self.width),
        )

        results['height'] = capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            float(self.height),
        )

        results['fps'] = capture.set(
            cv2.CAP_PROP_FPS,
            float(self.fps),
        )
        failed = [name for name, success in results.items() if not success]
        if failed:
            self.get_logger().warn(
                'V4L2 rejected capture settings: ' + ', '.join(failed)
            )

    def test_device(
        self,
        device_path: str,
    ) -> Optional[cv2.VideoCapture]:
        formats: List[Optional[str]] = []

        if self.fourcc:
            formats.append(self.fourcc)

        formats.extend([
            None,
            'YUYV',
        ])

        unique_formats: List[Optional[str]] = []

        for image_format in formats:
            if image_format not in unique_formats:
                unique_formats.append(image_format)

        for image_format in unique_formats:
            backend = cv2.CAP_V4L2 if self.backend == 'V4L2' else cv2.CAP_ANY
            capture = cv2.VideoCapture(device_path, backend)

            if not capture.isOpened():
                capture.release()
                continue

            self.configure_capture(
                capture,
                image_format,
            )

            # Some cameras need several frames before becoming ready.
            for _ in range(12):
                success, frame = capture.read()

                if (
                    success
                    and frame is not None
                    and frame.size > 0
                ):
                    actual_width = int(
                        capture.get(
                            cv2.CAP_PROP_FRAME_WIDTH
                        )
                    )

                    actual_height = int(
                        capture.get(
                            cv2.CAP_PROP_FRAME_HEIGHT
                        )
                    )

                    actual_fps = capture.get(
                        cv2.CAP_PROP_FPS
                    )
                    actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
                    actual_fourcc_name = ''.join(
                        chr((actual_fourcc >> (8 * index)) & 0xff)
                        for index in range(4)
                    )

                    format_name = (
                        image_format
                        if image_format
                        else 'driver-default'
                    )

                    self.get_logger().info(
                        f'Connected to {device_path}: '
                        f'{actual_width}x{actual_height} '
                        f'@ {actual_fps:.2f} FPS, '
                        f'requested={format_name}, actual={actual_fourcc_name}, '
                        f'backend={capture.getBackendName()}'
                    )

                    return capture

                time.sleep(0.08)

            capture.release()

        return None

    def connect_camera(self) -> bool:
        candidates = self.find_candidate_devices()

        if not candidates:
            self.log_warning_throttled(
                f'USB camera '
                f'{self.vendor_id}:{self.product_id} '
                f'not found'
            )
            return False

        self.get_logger().info(
            'Matching video devices: '
            + ', '.join(candidates)
        )

        for device_path in candidates:
            capture = self.test_device(device_path)

            if capture is not None:
                self.capture = capture
                self.active_device = device_path
                self.read_failure_count = 0
                return True

            self.get_logger().warn(
                f'{device_path} matches the USB ID, '
                f'but it does not provide image frames'
            )

        return False

    def disconnect_camera(self) -> None:
        old_device = self.active_device

        if self.capture is not None:
            self.capture.release()

        self.capture = None
        self.active_device = None
        self.read_failure_count = 0

        if old_device is not None:
            self.get_logger().warn(
                f'Camera {old_device} disconnected. '
                f'Waiting for reconnection.'
            )

    def timer_callback(self) -> None:
        if self.capture is None:
            current_time = time.monotonic()

            if (
                current_time - self.last_search_time
                >= self.reconnect_interval
            ):
                self.last_search_time = current_time
                self.connect_camera()

            return

        read_start = time.monotonic()
        success, frame = self.capture.read()
        self.read_duration += time.monotonic() - read_start

        if (
            not success
            or frame is None
            or frame.size == 0
        ):
            self.dropped_count += 1
            self.read_failure_count += 1

            self.log_warning_throttled(
                f'Failed to read {self.active_device}: '
                f'{self.read_failure_count}/'
                f'{self.max_read_failures}',
                interval=1.0,
            )

            if (
                self.read_failure_count
                >= self.max_read_failures
            ):
                self.disconnect_camera()

            return

        self.read_failure_count = 0
        self.captured_count += 1

        try:
            bridge_start = time.monotonic()
            image_message = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8',
            )

            image_message.header.stamp = (
                self.get_clock().now().to_msg()
            )

            image_message.header.frame_id = (
                self.frame_id
            )

            self.bridge_duration += time.monotonic() - bridge_start

            publish_start = time.monotonic()
            self.publisher.publish(image_message)
            if self.camera_info is not None:
                self.camera_info.header = image_message.header
                self.camera_info_publisher.publish(self.camera_info)
            self.publish_duration += time.monotonic() - publish_start
            self.published_count += 1
            self.frame_count += 1
            self.report_timing()

        except Exception as error:
            self.dropped_count += 1
            self.get_logger().error(
                f'Image conversion or publication failed: '
                f'{error}'
            )

    def load_camera_info(self, url: str) -> Optional[CameraInfo]:
        """Load a standard camera_calibration YAML without inventing values."""
        if not url:
            return None
        parsed = urlparse(url)
        path = Path(parsed.path if parsed.scheme == 'file' else url).expanduser()
        try:
            calibration = yaml.safe_load(path.read_text(encoding='utf-8'))
            info = CameraInfo()
            info.width = int(calibration['image_width'])
            info.height = int(calibration['image_height'])
            info.distortion_model = str(calibration['distortion_model'])
            info.d = [float(value) for value in calibration['distortion_coefficients']['data']]
            info.k = [float(value) for value in calibration['camera_matrix']['data']]
            info.r = [float(value) for value in calibration['rectification_matrix']['data']]
            info.p = [float(value) for value in calibration['projection_matrix']['data']]
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f'Invalid camera_info_url {url}: {error}')
            return None
        self.get_logger().info(f'Loaded camera calibration: {path}')
        return info

    def report_timing(self) -> None:
        """Periodically report measured camera pipeline throughput and stages."""
        elapsed = time.monotonic() - self.timing_window_start
        if elapsed < self.timing_report_interval or self.frame_count == 0:
            return
        frames = self.frame_count
        self.get_logger().info(
            f'Camera timing: fps={frames / elapsed:.2f}, '
            f'read_ms={1000.0 * self.read_duration / frames:.2f}, '
            f'bridge_ms={1000.0 * self.bridge_duration / frames:.2f}, '
            f'publish_ms={1000.0 * self.publish_duration / frames:.2f}, '
            f'captured={self.captured_count}, published={self.published_count}, '
            f'dropped={self.dropped_count}'
        )
        self.frame_count = 0
        self.read_duration = 0.0
        self.bridge_duration = 0.0
        self.publish_duration = 0.0
        self.timing_window_start = time.monotonic()

    def destroy_node(self) -> bool:
        self.disconnect_camera()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = UsbCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
