#!/usr/bin/env python3
"""Measure continuous nearest-neighbor LiDAR/IMU timestamp offsets."""

import json
import statistics
import time
from collections import deque
from pathlib import Path
from typing import Deque, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from sensor_bringup.pointcloud2_utils import timestamp_statistics


def percentile(values: List[float], percentage: float) -> float:
    """Return a linearly interpolated percentile."""
    return float(np.percentile(np.asarray(values), percentage))


class LivoxTimeSyncMonitor(Node):
    """Collect LiDAR stamps and pair each with the nearest buffered IMU stamp."""

    def __init__(self) -> None:
        super().__init__('livox_time_sync_monitor')
        self.declare_parameter('points_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu_base')
        self.declare_parameter('duration_sec', 60.0)
        self.declare_parameter('output_path', '/tmp/livox_time_sync.json')
        self.duration = float(self.get_parameter('duration_sec').value)
        self.output_path = Path(str(self.get_parameter('output_path').value))
        self.start_time = time.monotonic()
        self.imu_stamps: Deque[float] = deque(maxlen=5000)
        self.pending_lidar: Deque[Tuple[float, dict]] = deque()
        self.deltas: List[float] = []
        self.point_ranges: List[float] = []
        self.lidar_receive_offsets: List[float] = []
        self.imu_receive_offsets: List[float] = []
        self.finished = False
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('points_topic').value),
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.2, self.timer_callback)

    def imu_callback(self, message: Imu) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.imu_stamps.append(stamp)
        self.imu_receive_offsets.append(time.time() - stamp)
        self.finish_if_due()

    def lidar_callback(self, message: PointCloud2) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        stats = timestamp_statistics(message)
        point_range = stats['timestamp_range']
        if point_range is not None:
            self.point_ranges.append(float(point_range) / 1e9)
        self.lidar_receive_offsets.append(time.time() - stamp)
        self.pending_lidar.append((stamp, stats))
        self.pair_pending()
        self.finish_if_due()

    def pair_pending(self) -> None:
        if not self.imu_stamps:
            return
        imu_values = np.asarray(self.imu_stamps)
        while self.pending_lidar:
            lidar_stamp, _ = self.pending_lidar.popleft()
            nearest = float(imu_values[np.argmin(np.abs(imu_values - lidar_stamp))])
            self.deltas.append(lidar_stamp - nearest)

    def timer_callback(self) -> None:
        self.finish_if_due()

    def finish_if_due(self) -> None:
        """Finish from any callback so high sensor rates cannot starve a timer."""
        if self.finished or time.monotonic() - self.start_time < self.duration:
            return
        self.finished = True
        self.pair_pending()
        summary = self.make_summary()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        self.get_logger().info(json.dumps(summary, separators=(',', ':')))
        rclpy.shutdown()

    def make_summary(self) -> dict:
        """Build signed LiDAR-minus-nearest-IMU statistics."""
        if not self.deltas:
            return {'duration_sec': self.duration, 'sample_count': 0}
        return {
            'duration_sec': time.monotonic() - self.start_time,
            'sample_count': len(self.deltas),
            'signed_delta_definition': 'lidar_header_stamp-nearest_imu_header_stamp',
            'mean_sec': statistics.fmean(self.deltas),
            'median_sec': statistics.median(self.deltas),
            'std_sec': statistics.pstdev(self.deltas),
            'p5_sec': percentile(self.deltas, 5.0),
            'p95_sec': percentile(self.deltas, 95.0),
            'min_sec': min(self.deltas),
            'max_sec': max(self.deltas),
            'point_timestamp_range_sec': summarize(self.point_ranges),
            'lidar_receive_wall_minus_stamp_sec': summarize(
                self.lidar_receive_offsets
            ),
            'imu_receive_wall_minus_stamp_sec': summarize(self.imu_receive_offsets),
        }


def summarize(values: List[float]) -> dict:
    """Return compact distribution statistics."""
    if not values:
        return {'count': 0}
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'median': statistics.median(values),
        'min': min(values),
        'max': max(values),
    }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxTimeSyncMonitor()
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
