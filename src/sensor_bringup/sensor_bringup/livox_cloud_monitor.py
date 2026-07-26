#!/usr/bin/env python3
"""Record compact, continuous health evidence for Livox PointCloud2 frames."""

import csv
import json
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from sensor_bringup.pointcloud2_utils import (
    point_count,
    timestamp_statistics,
    xyz_masks,
)


class LivoxCloudMonitor(Node):
    """Monitor every cloud while logging only periodic summaries and anomalies."""

    def __init__(self) -> None:
        super().__init__('livox_cloud_monitor')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_directory', '/tmp/farm_ws_cloud_monitor')
        self.declare_parameter('minimum_finite_points', 256)
        self.declare_parameter('minimum_valid_ratio', 0.5)
        self.declare_parameter('maximum_frame_gap_sec', 0.5)
        self.declare_parameter('maximum_timestamp_range_sec', 0.2)
        self.declare_parameter('summary_interval_sec', 30.0)
        self.declare_parameter('save_anomaly_clouds', True)
        self.declare_parameter('anomaly_detail_interval_frames', 1000)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.minimum_finite_points = int(
            self.get_parameter('minimum_finite_points').value
        )
        self.minimum_valid_ratio = float(
            self.get_parameter('minimum_valid_ratio').value
        )
        self.maximum_frame_gap = float(
            self.get_parameter('maximum_frame_gap_sec').value
        )
        self.maximum_timestamp_range = float(
            self.get_parameter('maximum_timestamp_range_sec').value
        )
        self.summary_interval = float(
            self.get_parameter('summary_interval_sec').value
        )
        self.save_anomaly_clouds = bool(
            self.get_parameter('save_anomaly_clouds').value
        )
        self.anomaly_detail_interval = max(
            1,
            int(self.get_parameter('anomaly_detail_interval_frames').value),
        )

        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)
        )
        self.anomaly_directory = self.output_directory / 'anomalies'
        self.anomaly_directory.mkdir(parents=True, exist_ok=True)
        self.jsonl_file = (self.output_directory / 'frames.jsonl').open(
            'a', encoding='utf-8', buffering=1
        )
        self.csv_handle = (self.output_directory / 'frames.csv').open(
            'a', encoding='utf-8', newline='', buffering=1
        )
        self.csv_writer: Optional[csv.DictWriter] = None

        self.previous_stamp: Optional[float] = None
        self.last_summary_time = time.monotonic()
        self.frames = 0
        self.anomalous_frames = 0
        self.consecutive_anomalies = 0
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Monitoring {self.input_topic}; evidence: {self.output_directory}'
        )

    def cloud_callback(self, message: PointCloud2) -> None:
        receive_wall = time.time()
        receive_monotonic = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        stamp_delta = (
            None if self.previous_stamp is None else stamp - self.previous_stamp
        )
        self.previous_stamp = stamp
        count = point_count(message)
        reasons: List[str] = []

        try:
            finite_mask, zero_mask, valid_mask = xyz_masks(message)
            finite_count = int(finite_mask.sum())
            zero_count = int(zero_mask.sum())
            valid_count = int(valid_mask.sum())
        except ValueError as error:
            finite_count = zero_count = valid_count = 0
            reasons.append(str(error))

        valid_ratio = valid_count / count if count else 0.0
        time_stats = timestamp_statistics(message)
        if count == 0:
            reasons.append('empty_cloud')
        if finite_count < self.minimum_finite_points:
            reasons.append('too_few_finite_points')
        if valid_ratio < self.minimum_valid_ratio:
            reasons.append('low_valid_ratio')
        if stamp_delta is not None and stamp_delta <= 0.0:
            reasons.append('non_increasing_header_stamp')
        if stamp_delta is not None and stamp_delta > self.maximum_frame_gap:
            reasons.append('large_frame_gap')
        if time_stats['timestamp_nonfinite_count'] not in (None, 0):
            reasons.append('nonfinite_point_timestamp')

        timestamp_range = time_stats['timestamp_range']
        # Livox FLOAT64 timestamps are nanoseconds; GLIM converts them to sec.
        timestamp_range_sec = (
            timestamp_range / 1e9 if timestamp_range is not None else None
        )
        if (
            timestamp_range_sec is not None
            and timestamp_range_sec > self.maximum_timestamp_range
        ):
            reasons.append('large_point_timestamp_range')

        self.frames += 1
        if reasons:
            self.anomalous_frames += 1
            self.consecutive_anomalies += 1
        else:
            self.consecutive_anomalies = 0

        row: Dict[str, object] = {
            'sequence': self.frames,
            'header_stamp': stamp,
            'receive_wall_time': receive_wall,
            'stamp_delta_sec': stamp_delta,
            'width': message.width,
            'height': message.height,
            'point_step': message.point_step,
            'row_step': message.row_step,
            'data_length': len(message.data),
            'point_count': count,
            'finite_xyz_count': finite_count,
            'nan_inf_xyz_count': count - finite_count,
            'zero_xyz_count': zero_count,
            'valid_xyz_count': valid_count,
            'valid_ratio': valid_ratio,
            'frame_id': message.header.frame_id,
            'processing_latency_sec': time.monotonic() - receive_monotonic,
            'consecutive_anomalies': self.consecutive_anomalies,
            'anomaly_reasons': reasons,
            'timestamp_range_sec': timestamp_range_sec,
            **time_stats,
        }
        self.jsonl_file.write(json.dumps(row, separators=(',', ':')) + '\n')
        csv_row = {
            key: json.dumps(value) if isinstance(value, list) else value
            for key, value in row.items()
        }
        if self.csv_writer is None:
            self.csv_writer = csv.DictWriter(
                self.csv_handle, fieldnames=list(csv_row.keys())
            )
            if self.csv_handle.tell() == 0:
                self.csv_writer.writeheader()
        self.csv_writer.writerow(csv_row)

        if reasons:
            if (
                self.consecutive_anomalies == 1
                or self.consecutive_anomalies % self.anomaly_detail_interval == 0
            ):
                self.save_anomaly(message, row)
            if self.consecutive_anomalies == 1 or (
                self.consecutive_anomalies % 20 == 0
            ):
                self.get_logger().warn(
                    'Anomalous cloud: '
                    f'{reasons}, points={count}, valid={valid_count}, '
                    f'consecutive={self.consecutive_anomalies}'
                )
        if time.monotonic() - self.last_summary_time >= self.summary_interval:
            self.get_logger().info(
                f'Cloud summary: frames={self.frames}, '
                f'anomalous={self.anomalous_frames}, '
                f'last_points={count}, last_valid_ratio={valid_ratio:.4f}'
            )
            self.last_summary_time = time.monotonic()

    def save_anomaly(self, message: PointCloud2, row: Dict[str, object]) -> None:
        """Persist metadata and optionally exact raw point records."""
        stem = f'frame_{self.frames:09d}_{message.header.stamp.sec}'
        metadata = self.anomaly_directory / f'{stem}.json'
        metadata.write_text(json.dumps(row, indent=2), encoding='utf-8')
        if self.save_anomaly_clouds:
            (self.anomaly_directory / f'{stem}.bin').write_bytes(message.data)

    def destroy_node(self) -> bool:
        self.jsonl_file.close()
        self.csv_handle.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCloudMonitor()
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
