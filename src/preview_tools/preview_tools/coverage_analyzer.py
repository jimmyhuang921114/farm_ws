"""Publish mapping trajectory and an approximate visited-cell coverage view."""

import json
import math
import time

import rclpy
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
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .coverage_model import CoverageModel


class CoverageAnalyzer(Node):
    """Convert GLIM mapping odometry into bounded trajectory/coverage previews."""

    def __init__(self):
        super().__init__('coverage_analyzer')
        self.declare_parameter('odom_topic', '/glim_ros/odom')
        self.declare_parameter('trajectory_topic', '/collection/trajectory')
        self.declare_parameter('markers_topic', '/coverage/markers')
        self.declare_parameter('diagnostics_topic', '/coverage/diagnostics')
        self.declare_parameter('status_json_topic', '/coverage/status_json')
        self.declare_parameter('cell_size_m', 0.5)
        self.declare_parameter('minimum_translation_m', 0.05)
        self.declare_parameter('maximum_path_poses', 20000)
        self.declare_parameter('maximum_marker_cells', 50000)
        self.declare_parameter('stale_after_sec', 2.0)

        self.cell_size = float(self.get_parameter('cell_size_m').value)
        self.minimum_translation = float(
            self.get_parameter('minimum_translation_m').value)
        self.maximum_path_poses = int(
            self.get_parameter('maximum_path_poses').value)
        self.maximum_marker_cells = int(
            self.get_parameter('maximum_marker_cells').value)
        self.stale_after = float(self.get_parameter('stale_after_sec').value)
        if self.maximum_path_poses < 1 or self.maximum_marker_cells < 1:
            raise ValueError('path and marker limits must be positive')

        self.model = CoverageModel(self.cell_size)
        self.path = Path()
        self.frame_id = ''
        self.last_receive_time = None
        self.last_accepted_position = None
        self.last_stamp_ns = None
        self.rejected_nonfinite = 0
        self.rejected_stamp = 0

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
        self.create_service(
            Trigger, '/coverage/reset', self.on_reset)
        self.create_timer(1.0, self.publish_status)
        odom_topic = self.get_parameter('odom_topic').value
        self.get_logger().info(
            f'Coverage analyzer uses mapping odometry {odom_topic}; '
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
            'last_message_age_sec': age,
            'pose_count': self.model.pose_count,
            'path_pose_count': len(self.path.poses),
            'visited_cell_count': len(self.model.cells),
            'cell_size_m': self.cell_size,
            'visited_area_m2': self.model.area,
            'travel_distance_m': self.model.distance,
            'rejected_nonfinite': self.rejected_nonfinite,
            'rejected_non_increasing_stamp': self.rejected_stamp,
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
        self.path = Path()
        self.frame_id = ''
        self.last_receive_time = None
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
