"""Publish mapping trajectory and approximate 3D point-density coverage."""

from collections import Counter
import json
import math
import struct
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .coverage_model import CoverageModel

FIELD_DTYPES = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}


class CoverageAnalyzer(Node):
    """Convert GLIM mapping odometry into bounded trajectory/coverage previews."""

    def __init__(self):
        super().__init__('coverage_analyzer')
        self.declare_parameter('odom_topic', '/glim_ros/odom')
        self.declare_parameter('points_topic', '/glim_ros/points')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('trajectory_topic', '/collection/trajectory')
        self.declare_parameter('markers_topic', '/coverage/markers')
        self.declare_parameter(
            'density_markers_topic', '/coverage/density_markers')
        self.declare_parameter('density_cloud_topic', '/coverage/density_cloud')
        self.declare_parameter('diagnostics_topic', '/coverage/diagnostics')
        self.declare_parameter('status_json_topic', '/coverage/status_json')
        self.declare_parameter('cell_size_m', 0.5)
        self.declare_parameter('density_cell_size_m', 0.25)
        self.declare_parameter('sparse_points_per_cell', 15)
        self.declare_parameter('dense_points_per_cell', 120)
        self.declare_parameter('minimum_translation_m', 0.05)
        self.declare_parameter('maximum_path_poses', 20000)
        self.declare_parameter('maximum_marker_cells', 50000)
        self.declare_parameter('maximum_density_cells', 100000)
        self.declare_parameter('stale_after_sec', 2.0)

        self.cell_size = float(self.get_parameter('cell_size_m').value)
        self.density_cell_size = float(
            self.get_parameter('density_cell_size_m').value)
        self.sparse_points_per_cell = int(
            self.get_parameter('sparse_points_per_cell').value)
        self.dense_points_per_cell = int(
            self.get_parameter('dense_points_per_cell').value)
        self.minimum_translation = float(
            self.get_parameter('minimum_translation_m').value)
        self.maximum_path_poses = int(
            self.get_parameter('maximum_path_poses').value)
        self.maximum_marker_cells = int(
            self.get_parameter('maximum_marker_cells').value)
        self.maximum_density_cells = int(
            self.get_parameter('maximum_density_cells').value)
        self.stale_after = float(self.get_parameter('stale_after_sec').value)
        if (
            self.maximum_path_poses < 1
            or self.maximum_marker_cells < 1
            or self.maximum_density_cells < 1
        ):
            raise ValueError('path and marker limits must be positive')
        if self.density_cell_size <= 0.0:
            raise ValueError('density_cell_size_m must be positive')
        if self.dense_points_per_cell <= self.sparse_points_per_cell:
            raise ValueError(
                'dense_points_per_cell must exceed sparse_points_per_cell')

        self.model = CoverageModel(self.cell_size)
        self.density_cells = Counter()
        self.path = Path()
        self.frame_id = ''
        self.points_frame_id = ''
        self.last_receive_time = None
        self.last_points_time = None
        self.last_accepted_position = None
        self.last_stamp_ns = None
        self.rejected_nonfinite = 0
        self.rejected_stamp = 0
        self.rejected_pointcloud_layout = 0
        self.rejected_missing_transform = 0
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.path_publisher = self.create_publisher(
            Path, self.get_parameter('trajectory_topic').value, transient_qos)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value,
            transient_qos)
        self.density_marker_publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('density_markers_topic').value,
            transient_qos)
        self.density_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('density_cloud_topic').value,
            transient_qos)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10)
        self.json_publisher = self.create_publisher(
            String, self.get_parameter('status_json_topic').value,
            transient_qos)
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('points_topic').value,
            self.on_points,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, '/coverage/reset', self.on_reset)
        self.create_timer(1.0, self.publish_status)
        odom_topic = self.get_parameter('odom_topic').value
        points_topic = self.get_parameter('points_topic').value
        self.get_logger().info(
            f'Coverage analyzer uses mapping odometry {odom_topic}; '
            f'3D point density uses {points_topic}; '
            'visited-cell area is an approximation, not pure localization'
        )

    def on_odometry(self, message):
        """Accept finite, increasing mapping poses at a bounded spacing."""
        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y)):
            self.rejected_nonfinite += 1
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.rejected_stamp += 1
            return
        self.last_stamp_ns = stamp_ns
        self.last_receive_time = time.monotonic()
        self.frame_id = message.header.frame_id or self.frame_id or 'map'

        current = (float(position.x), float(position.y))
        if self.last_accepted_position is not None:
            separation = math.hypot(
                current[0] - self.last_accepted_position[0],
                current[1] - self.last_accepted_position[1],
            )
            if separation < self.minimum_translation:
                return
        self.last_accepted_position = current
        self.model.add(*current)

        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.path.header = message.header
        self.path.poses.append(pose)
        if len(self.path.poses) > self.maximum_path_poses:
            self.path.poses = self.path.poses[-self.maximum_path_poses:]
        self.path_publisher.publish(self.path)
        self.publish_markers(message.header.stamp)

    def publish_markers(self, stamp):
        """Publish one bounded CUBE_LIST representing visited grid cells."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'visited_cells'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = 0.03
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.3
        marker.color.a = 0.55
        cells = sorted(self.model.cells)[-self.maximum_marker_cells:]
        for cell_x, cell_y in cells:
            point = Point()
            point.x = (cell_x + 0.5) * self.cell_size
            point.y = (cell_y + 0.5) * self.cell_size
            point.z = 0.0
            marker.points.append(point)
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def on_points(self, message):
        """Accumulate approximate 3D point density from registered clouds."""
        try:
            x_values, y_values, z_values = self.xyz_arrays(message)
        except ValueError:
            self.rejected_pointcloud_layout += 1
            return
        source_frame = message.header.frame_id or self.target_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                message.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time().to_msg(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                self.rejected_missing_transform += 1
                return
        finite = np.isfinite(x_values) & np.isfinite(y_values) & np.isfinite(z_values)
        if not np.any(finite):
            return
        try:
            x_map, y_map, z_map = self.transform_xyz(
                transform,
                x_values[finite].astype(np.float64, copy=False),
                y_values[finite].astype(np.float64, copy=False),
                z_values[finite].astype(np.float64, copy=False),
            )
        except ValueError:
            self.rejected_missing_transform += 1
            return
        x_cells = np.floor(x_map / self.density_cell_size).astype(np.int64)
        y_cells = np.floor(y_map / self.density_cell_size).astype(np.int64)
        z_cells = np.floor(z_map / self.density_cell_size).astype(np.int64)
        for cell in zip(x_cells.tolist(), y_cells.tolist(), z_cells.tolist()):
            self.density_cells[cell] += 1
        while len(self.density_cells) > self.maximum_density_cells:
            self.density_cells.pop(next(iter(self.density_cells)))
        self.points_frame_id = self.target_frame
        self.last_points_time = time.monotonic()
        self.publish_density_markers(message.header.stamp)
        self.publish_density_cloud(message.header.stamp)

    @staticmethod
    def transform_xyz(transform, x_values, y_values, z_values):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise ValueError('zero-length quaternion')
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        map_x = (
            (1.0 - 2.0 * (yy + zz)) * x_values
            + 2.0 * (xy - wz) * y_values
            + 2.0 * (xz + wy) * z_values
            + translation.x
        )
        map_y = (
            2.0 * (xy + wz) * x_values
            + (1.0 - 2.0 * (xx + zz)) * y_values
            + 2.0 * (yz - wx) * z_values
            + translation.y
        )
        map_z = (
            2.0 * (xz - wy) * x_values
            + 2.0 * (yz + wx) * y_values
            + (1.0 - 2.0 * (xx + yy)) * z_values
            + translation.z
        )
        return map_x, map_y, map_z

    @staticmethod
    def point_count(message):
        declared = int(message.width) * int(message.height)
        available = len(message.data) // int(message.point_step or 1)
        return min(declared, available)

    @staticmethod
    def field_array(message, name, count):
        field = next((item for item in message.fields if item.name == name), None)
        if field is None or field.datatype not in FIELD_DTYPES or field.count != 1:
            return None
        endian = '>' if message.is_bigendian else '<'
        dtype = np.dtype(endian + FIELD_DTYPES[field.datatype])
        return np.ndarray(
            shape=(count,),
            dtype=dtype,
            buffer=message.data,
            offset=field.offset,
            strides=(message.point_step,),
        )

    @classmethod
    def xyz_arrays(cls, message):
        count = cls.point_count(message)
        arrays = [cls.field_array(message, name, count) for name in ('x', 'y', 'z')]
        if count <= 0 or any(array is None for array in arrays):
            raise ValueError('PointCloud2 must contain scalar x, y, and z fields')
        return arrays

    def publish_density_markers(self, stamp):
        """Publish accumulated 3D point density; red points need more coverage."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.points_frame_id or self.frame_id or 'map'
        marker.ns = 'point_density'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        cells = list(self.density_cells.items())[-self.maximum_density_cells:]
        span = float(self.dense_points_per_cell - self.sparse_points_per_cell)
        for (cell_x, cell_y, cell_z), count in cells:
            point = Point()
            point.x = (cell_x + 0.5) * self.density_cell_size
            point.y = (cell_y + 0.5) * self.density_cell_size
            point.z = (cell_z + 0.5) * self.density_cell_size
            marker.points.append(point)

            ratio = min(
                1.0,
                max(0.0, (count - self.sparse_points_per_cell) / span),
            )
            red, green, blue = self.density_color(ratio)
            color = ColorRGBA()
            color.r = red / 255.0
            color.g = green / 255.0
            color.b = blue / 255.0
            color.a = 0.65
            marker.colors.append(color)
        self.density_marker_publisher.publish(MarkerArray(markers=[marker]))

    @staticmethod
    def density_color(ratio):
        """Map sparse-to-dense ratio into red-yellow-green RGB."""
        clamped = min(1.0, max(0.0, ratio))
        if clamped < 0.5:
            local = clamped * 2.0
            red = 255
            green = int(50 + 205 * local)
            blue = 20
        else:
            local = (clamped - 0.5) * 2.0
            red = int(255 * (1.0 - local))
            green = 255
            blue = 20
        return red, green, blue

    @staticmethod
    def packed_rgb_float(red, green, blue):
        """Pack RGB bytes in the float32 layout expected by RViz RGB8."""
        packed = (int(red) << 16) | (int(green) << 8) | int(blue)
        return struct.unpack('<f', struct.pack('<I', packed))[0]

    def publish_density_cloud(self, stamp):
        """Publish density as a colored PointCloud2 fixed in the map frame."""
        cells = list(self.density_cells.items())[-self.maximum_density_cells:]
        span = float(self.dense_points_per_cell - self.sparse_points_per_cell)
        output = PointCloud2()
        output.header.stamp = stamp
        output.header.frame_id = self.points_frame_id or self.target_frame
        output.height = 1
        output.width = len(cells)
        output.is_bigendian = False
        output.is_dense = True
        output.point_step = 16
        output.row_step = output.point_step * output.width
        output.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        output.data = bytearray(output.row_step)
        for index, ((cell_x, cell_y, cell_z), count) in enumerate(cells):
            ratio = min(
                1.0,
                max(0.0, (count - self.sparse_points_per_cell) / span),
            )
            red, green, blue = self.density_color(ratio)
            rgb = self.packed_rgb_float(red, green, blue)
            struct.pack_into(
                '<ffff',
                output.data,
                index * output.point_step,
                (cell_x + 0.5) * self.density_cell_size,
                (cell_y + 0.5) * self.density_cell_size,
                (cell_z + 0.5) * self.density_cell_size,
                rgb,
            )
        self.density_cloud_publisher.publish(output)

    def status_values(self):
        """Return a JSON-compatible status shared by RQT and Web adapters."""
        age = (
            None if self.last_receive_time is None
            else max(0.0, time.monotonic() - self.last_receive_time)
        )
        if age is None:
            state = 'WAITING'
        elif age >= self.stale_after:
            state = 'STALE'
        else:
            state = 'MAPPING'
        return {
            'schema_version': 1,
            'mode': 'mapping',
            'pure_localization': False,
            'state': state,
            'odom_topic': self.get_parameter('odom_topic').value,
            'frame_id': self.frame_id,
            'density_frame_id': self.points_frame_id,
            'last_message_age_sec': age,
            'pose_count': self.model.pose_count,
            'path_pose_count': len(self.path.poses),
            'visited_cell_count': len(self.model.cells),
            'cell_size_m': self.cell_size,
            'visited_area_m2': self.model.area,
            'density_cell_count': len(self.density_cells),
            'density_cell_size_m': self.density_cell_size,
            'sparse_points_per_cell': self.sparse_points_per_cell,
            'dense_points_per_cell': self.dense_points_per_cell,
            'travel_distance_m': self.model.distance,
            'rejected_nonfinite': self.rejected_nonfinite,
            'rejected_non_increasing_stamp': self.rejected_stamp,
            'rejected_pointcloud_layout': self.rejected_pointcloud_layout,
            'rejected_missing_transform': self.rejected_missing_transform,
        }

    def publish_status(self):
        """Publish machine-readable JSON and ROS diagnostics."""
        values = self.status_values()
        json_message = String()
        json_message.data = json.dumps(
            values, separators=(',', ':'), sort_keys=True)
        self.json_publisher.publish(json_message)

        status = DiagnosticStatus()
        status.name = 'mapping_coverage'
        status.hardware_id = 'glim_mapping'
        status.message = values['state']
        status.level = (
            DiagnosticStatus.OK
            if values['state'] == 'MAPPING'
            else DiagnosticStatus.WARN
        )
        for key, value in values.items():
            item = KeyValue()
            item.key = str(key)
            item.value = 'null' if value is None else str(value)
            status.values.append(item)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diagnostics_publisher.publish(array)

    def on_reset(self, request, response):
        """Reset only in-memory preview state; mapping output is untouched."""
        del request
        self.model.reset()
        self.density_cells.clear()
        self.path = Path()
        self.frame_id = ''
        self.points_frame_id = ''
        self.last_receive_time = None
        self.last_points_time = None
        self.last_accepted_position = None
        self.last_stamp_ns = None
        response.success = True
        response.message = 'Coverage preview reset; GLIM mapping was not changed'
        self.publish_status()
        return response


def main(args=None):
    """Run the coverage analyzer."""
    rclpy.init(args=args)
    node = CoverageAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
