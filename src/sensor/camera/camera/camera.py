from pathlib import Path
from array import array
import math
import sys
import copy
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


def find_device():
    for entry in sorted(Path('/sys/class/video4linux').glob('video*')):
        for parent in (entry / 'device').resolve().parents:
            vendor, product = parent / 'idVendor', parent / 'idProduct'
            if vendor.exists() and product.exists():
                if vendor.read_text().strip() == '1bcf' and product.read_text().strip() == '2cd1':
                    return '/dev/' + entry.name
    raise RuntimeError('USB camera 1bcf:2cd1 not found; set video_device explicitly')


def image_message(data, encoding, header):
    data = np.ascontiguousarray(data)
    msg = Image()
    msg.header = header
    msg.height, msg.width = data.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = int(sys.byteorder == 'big')
    msg.step = data.strides[0]
    payload = array('B')
    payload.frombytes(data.tobytes())
    msg.data = payload
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        cv2.setNumThreads(1)
        defaults = {
            'video_device': '', 'width': 640, 'height': 480, 'fps': 30.0,
            'frame_id': 'camera_optical_frame', 'camera_info_file': '',
            'approximate_horizontal_fov_deg': 0.0,
            'run_depth_model': True, 'device': 'cuda',
            'checkpoint': str(Path(get_package_share_directory('camera')) / 'model/best.pt'),
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda name: self.get_parameter(name).value
        fps = float(param('fps'))
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('fps must be positive and finite')
        self.frame_id = param('frame_id')
        self.approximate_horizontal_fov_deg = float(param('approximate_horizontal_fov_deg'))
        if self.approximate_horizontal_fov_deg and not (0 < self.approximate_horizontal_fov_deg < 180):
            raise ValueError('approximate_horizontal_fov_deg must be between 0 and 180 degrees')
        self.calibration = None
        if param('camera_info_file'):
            with open(Path(param('camera_info_file')).expanduser()) as stream:
                self.calibration = yaml.safe_load(stream)
            k = np.array(self.calibration['camera_matrix']['data']).reshape(3, 3)
            if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
                raise ValueError('Invalid camera calibration matrix')
        elif self.approximate_horizontal_fov_deg:
            self.get_logger().warning(
                f'Using approximate CameraInfo with horizontal FOV '
                f'{self.approximate_horizontal_fov_deg:g} degrees; replace with camera calibration')
        else:
            self.get_logger().warning('No calibration: CameraInfo K remains zero (uncalibrated). Calibrate before using nvblox.')
        self.model = None
        if param('run_depth_model'):
            from .depth_estimate import DepthEstimate
            self.model = DepthEstimate(param('checkpoint'), param('device'))
            if self.model.calibrated and self.calibration is None:
                raise ValueError('The selected model requires camera_info_file')
        device = param('video_device') or find_device()

        if isinstance(device, str) and device.startswith('/dev/video'):
            device_index = int(device.replace('/dev/video', ''))
        else:
            device_index = int(device)

        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'Cannot open camera {device}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, param('width'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, param('height'))
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Two V4L2 buffers are the best trade-off for this UVC camera. With
        # one buffer OpenCV/V4L2 drops to about half the capture rate.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera request: {param("width")}x{param("height")} @ {fps:g} FPS; '
            f'driver reports: {actual_width:.0f}x{actual_height:.0f} @ {actual_fps:.1f} FPS')
        # cap.read() can block inside the V4L2 driver. Do not run it from the
        # ROS executor thread, otherwise one blocked read throttles both RGB
        # and all ROS timers.
        self.capture_condition = threading.Condition()
        self.latest_bgr = None
        self.latest_seq = 0
        self.published_seq = 0
        self.capture_stopping = False
        self.capture_reader = threading.Thread(
            target=self.read_camera, name='camera-reader', daemon=True)
        self.capture_reader.start()
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        self.maps = None
        self.shape = None
        self.depth_condition = threading.Condition()
        self.pending_depth = None
        self.depth_stopping = False
        self.depth_worker = None
        self.depth_stats = [time.monotonic(), 0, 0.0]
        if self.model:
            self.depth_worker = threading.Thread(target=self.infer_depth, name='depth-inference')
            self.depth_worker.start()
        self.capture_stats = [time.monotonic(), 0, 0.0, 0.0]
        self.timer = self.create_timer(1/fps, self.capture)
        self.get_logger().info(f'Camera opened: {device}; depth inference={self.model is not None}')

    def read_camera(self):
        stats_start = time.monotonic()
        stats_count = 0
        while True:
            with self.capture_condition:
                if self.capture_stopping:
                    return
            ok, bgr = self.cap.read()
            if not ok or bgr is None:
                self.get_logger().warning(
                    'Camera frame read failed', throttle_duration_sec=2)
                continue
            with self.capture_condition:
                self.latest_bgr = bgr
                self.latest_seq += 1
                self.capture_condition.notify()
            stats_count += 1
            now = time.monotonic()
            if now - stats_start >= 5.0:
                self.get_logger().info(
                    f'V4L2 reader: {stats_count / (now - stats_start):.1f} FPS')
                stats_start, stats_count = now, 0

    def prepare_info(self, width, height):
        info = CameraInfo()
        info.width, info.height = width, height
        info.header.frame_id = self.frame_id
        info.distortion_model = 'plumb_bob'
        self.intrinsics = None
        if self.calibration:
            c = self.calibration
            if c.get('distortion_model', 'plumb_bob') not in ('plumb_bob', 'rational_polynomial'):
                raise ValueError('Only plumb_bob/rational_polynomial calibration supported')
            k = np.array(c['camera_matrix']['data'], dtype=float).reshape(3, 3)
            k[0, :] *= width / c['image_width']
            k[1, :] *= height / c['image_height']
            d = np.array(c['distortion_coefficients']['data'], dtype=float)
            self.maps = cv2.initUndistortRectifyMap(k, d, np.eye(3), k, (width, height), cv2.CV_32FC1)
            info.k = k.ravel().tolist()
            info.r = np.eye(3).ravel().tolist()
            info.p = np.column_stack((k, np.zeros(3))).ravel().tolist()
            info.d = [0.0] * 5  # Published image is rectified before inference.
            self.intrinsics = (k[0, 0], k[1, 1], k[0, 2], k[1, 2])
        elif self.approximate_horizontal_fov_deg:
            focal = width / (2 * math.tan(math.radians(self.approximate_horizontal_fov_deg) / 2))
            cx, cy = width / 2, height / 2
            info.k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
            info.r = np.eye(3).ravel().tolist()
            info.p = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
            info.d = [0.] * 5
            self.intrinsics = (focal, focal, cx, cy)
        return info

    def capture(self):
        capture_start = time.monotonic()
        with self.capture_condition:
            if self.latest_bgr is None or self.latest_seq == self.published_seq:
                return
            bgr = self.latest_bgr.copy()
            self.published_seq = self.latest_seq
        stamp = self.get_clock().now().to_msg()
        if self.shape != bgr.shape[:2]:
            self.shape = bgr.shape[:2]
            self.info = self.prepare_info(bgr.shape[1], bgr.shape[0])
        if self.maps is not None:
            bgr = cv2.remap(bgr, *self.maps, interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.info.header.stamp = stamp
        self.rgb_pub.publish(image_message(rgb, 'rgb8', self.info.header))
        self.rgb_info_pub.publish(self.info)
        if self.model:
            # Keep only the newest frame; preserve its own stamp and intrinsics.
            with self.depth_condition:
                self.pending_depth = (rgb, self.intrinsics, copy.deepcopy(self.info))
                self.depth_condition.notify()
        now = time.monotonic()
        stats = self.capture_stats
        stats[1] += 1
        stats[2] += now - capture_start
        if now - stats[0] >= 5:
            self.get_logger().info(
                f'RGB publish: {stats[1] / (now - stats[0]):.1f} FPS; '
                f'prepare/publish={1000 * stats[2] / stats[1]:.1f} ms')
            self.capture_stats = [now, 0, 0.0, 0.0]

    def infer_depth(self):
        while True:
            with self.depth_condition:
                self.depth_condition.wait_for(
                    lambda: self.depth_stopping or self.pending_depth is not None)
                if self.depth_stopping:
                    return
                rgb, intrinsics, info = self.pending_depth
                self.pending_depth = None
            try:
                depth = self.model.predict(rgb, intrinsics)
                if self.depth_stopping or not rclpy.ok():
                    return
                # ROS 2's conventional depth format is uint16 millimeters.
                # Publishing float32 here costs twice the image bandwidth and
                # forces extra copies in DDS/nvblox. Keep the model output in
                # meters internally, then quantize only at the ROS boundary.
                depth_mm = np.nan_to_num(depth, nan=0.0, posinf=65.535, neginf=0.0)
                depth_mm = np.rint(np.clip(depth_mm, 0.0, 65.535) * 1000.0).astype(np.uint16)
                publish_start = time.monotonic()
                self.depth_pub.publish(image_message(depth_mm, '16UC1', info.header))
                self.depth_info_pub.publish(info)
                now = time.monotonic()
                self.depth_stats[1] += 1
                self.depth_stats[2] += now - publish_start
                if now - self.depth_stats[0] >= 5.0:
                    count = self.depth_stats[1]
                    self.get_logger().info(
                        f'Depth publish: {count / (now - self.depth_stats[0]):.1f} FPS; '
                        f'publish/serialize={1000 * self.depth_stats[2] / count:.1f} ms')
                    self.depth_stats = [now, 0, 0.0]
            except Exception as exc:
                if self.depth_stopping or not rclpy.ok():
                    return
                self.get_logger().error(f'Depth inference failed: {exc}', throttle_duration_sec=5)

    def destroy_node(self):
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if hasattr(self, 'depth_condition'):
            with self.depth_condition:
                self.depth_stopping = True
                self.depth_condition.notify_all()
            if self.depth_worker is not None:
                self.depth_worker.join()
        if hasattr(self, 'capture_condition'):
            with self.capture_condition:
                self.capture_stopping = True
                self.capture_condition.notify_all()
            if hasattr(self, 'capture_reader'):
                self.capture_reader.join(timeout=2)
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
