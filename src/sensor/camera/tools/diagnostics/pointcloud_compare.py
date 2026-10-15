#!/usr/bin/env python3
"""Compare the unclipped diagnostic cloud with nvblox back-projected depth."""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointcloudCompare(Node):
    def __init__(self):
        super().__init__("pointcloud_compare")
        pointcloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.stats = {"raw": [], "nvblox": []}
        self.subs = [
            self.create_subscription(
                PointCloud2, "/debug/raw_depth_pointcloud",
                lambda msg: self.callback("raw", msg), pointcloud_qos),
            self.create_subscription(
                PointCloud2, "/nvblox_node/back_projected_depth/camera_optical_frame",
                lambda msg: self.callback("nvblox", msg), pointcloud_qos),
        ]
        self.timer = self.create_timer(0.5, self.report)
        self.started = time.monotonic()

    def callback(self, name, msg):
        points = []
        try:
            for p in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
        except Exception as exc:
            self.get_logger().warning(f"{name} decode failed: {exc}")
            return
        if points:
            structured = np.asarray(points)
            if structured.dtype.names:
                a = np.column_stack((
                    structured["x"].astype(np.float32, copy=False),
                    structured["y"].astype(np.float32, copy=False),
                    structured["z"].astype(np.float32, copy=False),
                ))
            else:
                a = np.asarray(points, dtype=np.float32)
            self.stats[name].append({
                "stamp": (msg.header.stamp.sec, msg.header.stamp.nanosec),
                "frame_id": msg.header.frame_id,
                "points": int(a.shape[0]),
                "max_z": float(np.max(a[:, 2])),
                "p95_z": float(np.percentile(a[:, 2], 95)),
                "min_z": float(np.min(a[:, 2])),
            })

    def report(self):
        if time.monotonic() - self.started < 3.0:
            return
        for name in ("raw", "nvblox"):
            values = self.stats[name]
            if not values:
                print(f"{name}: no pointcloud received", flush=True)
                continue
            latest = values[-1]
            print(
                f"{name}: messages={len(values)} latest_points={latest['points']} "
                f"min_z={latest['min_z']:.3f} p95_z={latest['p95_z']:.3f} "
                f"max_z={latest['max_z']:.3f} frame={latest['frame_id']}",
                flush=True)


def main():
    rclpy.init()
    node = PointcloudCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
