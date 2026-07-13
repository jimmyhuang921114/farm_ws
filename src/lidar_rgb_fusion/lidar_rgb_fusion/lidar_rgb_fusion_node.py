import math
import struct

import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros


class LidarRgbFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_rgb_fusion_node')

        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_cloud_topic', '/velodyne_colored_points')
        self.declare_parameter('debug_image_topic', '/lidar_rgb_fusion/debug_image')
        self.declare_parameter('target_camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 30.0)
        self.declare_parameter('sync_slop', 0.2)
        self.declare_parameter('max_points', 30000)
        self.declare_parameter('debug_point_radius', 2)

        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_cloud_topic = self.get_parameter('output_cloud_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.target_camera_frame = self.get_parameter('target_camera_frame').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.sync_slop = float(self.get_parameter('sync_slop').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.debug_point_radius = int(self.get_parameter('debug_point_radius').value)

        qos = QoSProfile(depth=10)
        self.cloud_sub = message_filters.Subscriber(
            self, PointCloud2, self.lidar_topic, qos_profile=qos)
        self.image_sub = message_filters.Subscriber(
            self, Image, self.image_topic, qos_profile=qos)
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_sub, self.image_sub, self.info_sub],
            queue_size=10,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_cloud_topic, qos)
        self.debug_image_pub = self.create_publisher(
            Image, self.debug_image_topic, qos)

        self.last_tf_warning_time = None
        self.last_projection_warning_time = None

        self.get_logger().info(
            f'Listening: {self.lidar_topic}, {self.image_topic}, {self.camera_info_topic}')
        self.get_logger().info(
            f'Publishing: {self.output_cloud_topic}, {self.debug_image_topic}')

    def synced_callback(self, cloud_msg, image_msg, camera_info_msg):
        try:
            rgb_image = self.image_to_rgb_array(image_msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        try:
            transform_msg = self.lookup_transform(cloud_msg.header)
        except Exception as exc:
            self.warn_throttled(
                'tf',
                f'No TF {self.target_camera_frame} <- {cloud_msg.header.frame_id}: {exc}',
                5.0,
            )
            return

        points_lidar = self.read_lidar_points(cloud_msg)
        if points_lidar.size == 0:
            return

        points_camera = self.transform_points(points_lidar, transform_msg)
        selected = self.project_points(points_camera, camera_info_msg, image_msg)
        if selected is None:
            self.warn_throttled(
                'projection',
                'No LiDAR points projected into image',
                5.0,
            )
            self.publish_debug_image(image_msg, rgb_image, np.empty((0, 2), dtype=np.int32))
            return

        valid_indices, uv = selected
        colors = rgb_image[uv[:, 1], uv[:, 0], :]
        self.publish_colored_cloud(cloud_msg.header, points_lidar[valid_indices], colors)
        self.publish_debug_image(image_msg, rgb_image, uv)

    def image_to_rgb_array(self, image_msg):
        encoding = image_msg.encoding.lower()
        channels_by_encoding = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f'Unsupported image encoding: {image_msg.encoding}')

        channels = channels_by_encoding[encoding]
        row_bytes = image_msg.width * channels
        if image_msg.step < row_bytes:
            raise ValueError(
                f'Invalid image step {image_msg.step} for width {image_msg.width} and encoding {image_msg.encoding}')

        data = np.frombuffer(image_msg.data, dtype=np.uint8)
        expected = image_msg.height * image_msg.step
        if data.size < expected:
            raise ValueError(
                f'Image data too small: got {data.size} bytes, expected at least {expected}')

        rows = data[:expected].reshape((image_msg.height, image_msg.step))
        pixels = rows[:, :row_bytes].reshape((image_msg.height, image_msg.width, channels))

        if encoding == 'rgb8':
            return pixels[:, :, :3].copy()
        if encoding == 'bgr8':
            return pixels[:, :, [2, 1, 0]].copy()
        if encoding == 'rgba8':
            return pixels[:, :, :3].copy()
        return pixels[:, :, [2, 1, 0]].copy()

    def lookup_transform(self, cloud_header):
        stamp = self.header_stamp_to_time(cloud_header.stamp)
        return self.tf_buffer.lookup_transform(
            self.target_camera_frame,
            cloud_header.frame_id,
            stamp,
            timeout=Duration(seconds=0.05),
        )

    @staticmethod
    def header_stamp_to_time(stamp_msg):
        if isinstance(stamp_msg, TimeMsg):
            return rclpy.time.Time.from_msg(stamp_msg)
        return rclpy.time.Time()

    def read_lidar_points(self, cloud_msg):
        """Read x/y/z from PointCloud2 as a normal Nx3 float32 numpy array.

        ROS2 Humble sensor_msgs_py.point_cloud2.read_points() can return a
        structured numpy array with dtype names such as x/y/z. That cannot be
        directly cast with np.asarray(..., dtype=np.float32), so handle both
        structured-array and tuple/list styles.
        """
        points = point_cloud2.read_points(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        # Humble commonly returns a structured numpy array.
        if isinstance(points, np.ndarray):
            points_array = points
        else:
            points_list = list(points)
            if len(points_list) == 0:
                return np.empty((0, 3), dtype=np.float32)
            points_array = np.asarray(points_list)

        if points_array.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        if points_array.dtype.names is not None:
            xyz = np.column_stack((
                points_array['x'],
                points_array['y'],
                points_array['z'],
            )).astype(np.float32, copy=False)
        else:
            xyz = np.asarray(points_array, dtype=np.float32)
            if xyz.ndim == 1:
                xyz = xyz.reshape((-1, 3))
            else:
                xyz = xyz[:, :3]

        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]

        if self.max_points > 0 and xyz.shape[0] > self.max_points:
            indices = np.linspace(
                0, xyz.shape[0] - 1, self.max_points, dtype=np.int64)
            xyz = xyz[indices]

        return xyz.astype(np.float32, copy=False)

    @staticmethod
    def transform_points(points, transform_msg):
        t = transform_msg.transform.translation
        q = transform_msg.transform.rotation
        rotation = LidarRgbFusionNode.quaternion_to_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return points @ rotation.T + translation

    @staticmethod
    def quaternion_to_matrix(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3, dtype=np.float32)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float32)

    def project_points(self, points_camera, camera_info_msg, image_msg):
        fx = float(camera_info_msg.k[0])
        fy = float(camera_info_msg.k[4])
        cx = float(camera_info_msg.k[2])
        cy = float(camera_info_msg.k[5])
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warning('Invalid CameraInfo K matrix: fx or fy is zero')
            return None

        x = points_camera[:, 0]
        y = points_camera[:, 1]
        z = points_camera[:, 2]

        valid_depth = (z > self.min_depth) & (z < self.max_depth)
        valid_indices = np.nonzero(valid_depth)[0]
        if valid_indices.size == 0:
            return None

        x = x[valid_indices]
        y = y[valid_indices]
        z = z[valid_indices]

        u_float = fx * x / z + cx
        v_float = fy * y / z + cy
        u = np.rint(u_float).astype(np.int32)
        v = np.rint(v_float).astype(np.int32)

        inside = (
            (u >= 0) & (u < int(image_msg.width)) &
            (v >= 0) & (v < int(image_msg.height))
        )
        if not np.any(inside):
            return None

        return valid_indices[inside], np.column_stack((u[inside], v[inside]))

    def publish_colored_cloud(self, header, points, colors):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_points = []
        for point, color in zip(points, colors):
            r = int(color[0])
            g = int(color[1])
            b = int(color[2])
            rgb_uint32 = (r << 16) | (g << 8) | b
            rgb_float = struct.unpack('f', struct.pack('I', rgb_uint32))[0]
            cloud_points.append((float(point[0]), float(point[1]), float(point[2]), rgb_float))

        cloud_out = point_cloud2.create_cloud(header, fields, cloud_points)
        self.cloud_pub.publish(cloud_out)

    def publish_debug_image(self, image_msg, rgb_image, uv):
        debug = rgb_image.copy()
        radius = max(0, self.debug_point_radius)
        for u, v in uv:
            u_min = max(0, int(u) - radius)
            u_max = min(debug.shape[1] - 1, int(u) + radius)
            v_min = max(0, int(v) - radius)
            v_max = min(debug.shape[0] - 1, int(v) + radius)
            debug[v_min:v_max + 1, u_min:u_max + 1] = np.array([255, 0, 0], dtype=np.uint8)

        out = Image()
        out.header = image_msg.header
        out.height = image_msg.height
        out.width = image_msg.width
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = image_msg.width * 3
        out.data = debug.tobytes()
        self.debug_image_pub.publish(out)

    def warn_throttled(self, key, message, period_sec):
        now = self.get_clock().now()
        attr = f'last_{key}_warning_time'
        last = getattr(self, attr, None)
        if last is None or (now - last).nanoseconds > period_sec * 1e9:
            self.get_logger().warning(message)
            setattr(self, attr, now)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRgbFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
