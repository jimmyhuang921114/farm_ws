#!/usr/bin/env python3
"""Non-invasive RGB-D depth diagnostics for the camera/nvblox pipeline."""

import argparse
import math
import time
from collections import Counter

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def stamp_tuple(stamp):
    return int(stamp.sec), int(stamp.nanosec)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def decode_depth(msg):
    if msg.encoding == "32FC1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        scale = 1.0
    elif msg.encoding == "16UC1":
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        scale = 0.001
    else:
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    row_bytes = msg.width * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
        raise ValueError("invalid depth message stride or data length")
    raw = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=bytes(msg.data),
        strides=(msg.step, dtype.itemsize),
    ).copy()
    return raw.astype(np.float32) * scale


class DepthDiagnostic(Node):
    def __init__(self, args):
        super().__init__("depth_diagnostic")
        self.start = time.monotonic()
        self.duration = args.duration
        self.depth_topic = args.depth_topic
        self.info_topic = args.info_topic
        self.count = 0
        self.errors = Counter()
        self.hist = Counter()
        self.total_pixels = 0
        self.valid_pixels = 0
        self.nan_pixels = 0
        self.inf_pixels = 0
        self.nonpositive_pixels = 0
        self.all_values = []
        self.roi_values = {name: [] for name in (
            "left", "center", "right", "top", "middle", "bottom", "center_middle")}
        self.last_info = None
        self.last_depth_stamp = None
        self.last_info_stamp = None
        self.info_count = 0
        self.info_exact_matches = 0
        self.info_delta_ms = []
        self.last_cloud_time = 0.0
        self.last_report = self.start
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/debug/raw_depth_pointcloud", 1)
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f"Monitoring {self.depth_topic} and {self.info_topic}; "
            "publishing unclipped /debug/raw_depth_pointcloud")
        self.timer = self.create_timer(0.1, self.check_done)

    def info_callback(self, msg):
        self.info_count += 1
        self.last_info = msg
        self.last_info_stamp = stamp_tuple(msg.header.stamp)

    def depth_callback(self, msg):
        self.count += 1
        self.last_depth_stamp = stamp_tuple(msg.header.stamp)
        if self.last_info is not None:
            if self.last_depth_stamp == self.last_info_stamp:
                self.info_exact_matches += 1
            else:
                delta = abs(
                    stamp_seconds(msg.header.stamp) -
                    stamp_seconds(self.last_info.header.stamp)) * 1000.0
                self.info_delta_ms.append(delta)
        try:
            depth = decode_depth(msg)
            self.analyze(depth)
            if time.monotonic() - self.last_cloud_time >= 1.0:
                self.publish_cloud(depth, msg.header)
                self.last_cloud_time = time.monotonic()
        except Exception as exc:
            self.errors[str(exc)] += 1

        now = time.monotonic()
        if now - self.last_report >= 5.0:
            self.report(intermediate=True)
            self.last_report = now

    def analyze(self, depth):
        finite = np.isfinite(depth)
        valid = finite & (depth > 0.0)
        self.total_pixels += depth.size
        self.valid_pixels += int(valid.sum())
        self.nan_pixels += int(np.isnan(depth).sum())
        self.inf_pixels += int(np.isinf(depth).sum())
        self.nonpositive_pixels += int((finite & (depth <= 0.0)).sum())
        values = depth[valid]
        if values.size == 0:
            return
        self.all_values.append(values)
        h, w = depth.shape
        regions = {
            "left": depth[:, :w // 3],
            "center": depth[:, w // 3:2 * w // 3],
            "right": depth[:, 2 * w // 3:],
            "top": depth[:h // 3, :],
            "middle": depth[h // 3:2 * h // 3, :],
            "bottom": depth[2 * h // 3:, :],
            "center_middle": depth[h // 4:3 * h // 4, w // 4:3 * w // 4],
        }
        for name, region in regions.items():
            rv = region[np.isfinite(region) & (region > 0.0)]
            if rv.size:
                self.roi_values[name].append(rv)
        bins = ((0, 1), (1, 2), (2, 3), (3, 5), (5, 7),
                (7, 10), (10, 15), (15, 20), (20, math.inf))
        for lo, hi in bins:
            self.hist[f"{lo:g}-{hi:g}m"] += int(((values >= lo) & (values < hi)).sum())

    def publish_cloud(self, depth, header):
        if self.last_info is None:
            return
        h, w = depth.shape
        fx, fy = float(self.last_info.k[0]), float(self.last_info.k[4])
        cx, cy = float(self.last_info.k[2]), float(self.last_info.k[5])
        if min(fx, fy) <= 0:
            return
        valid = np.isfinite(depth) & (depth > 0.0)
        yy, xx = np.nonzero(valid)
        z = depth[yy, xx].astype(np.float32)
        points = np.column_stack(((xx - cx) * z / fx, (yy - cy) * z / fy, z))
        cloud_header = Header()
        cloud_header.stamp = header.stamp
        cloud_header.frame_id = header.frame_id
        cloud = point_cloud2.create_cloud_xyz32(cloud_header, points.tolist())
        self.cloud_pub.publish(cloud)

    @staticmethod
    def summarize(values):
        if not values:
            return None
        a = np.concatenate(values)
        return {
            "count": int(a.size), "min": float(np.min(a)),
            "median": float(np.median(a)), "mean": float(np.mean(a)),
            "p90": float(np.percentile(a, 90)),
            "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "max": float(np.max(a)),
        }

    def report(self, intermediate=False):
        label = "INTERMEDIATE" if intermediate else "FINAL"
        print(f"\n===== DEPTH DIAGNOSTIC {label} =====", flush=True)
        print(f"depth_messages={self.count} camera_info_messages={self.info_count}", flush=True)
        if self.last_info is not None:
            print(
                f"camera_info={self.last_info.width}x{self.last_info.height} "
                f"frame_id={self.last_info.header.frame_id} "
                f"K=({self.last_info.k[0]:.9g},{self.last_info.k[4]:.9g},"
                f"{self.last_info.k[2]:.9g},{self.last_info.k[5]:.9g})", flush=True)
        print(f"exact_image_info_matches={self.info_exact_matches}", flush=True)
        if self.info_delta_ms:
            print(f"last_info_delta_ms={self.info_delta_ms[-1]:.3f}", flush=True)
        print(f"overall={self.summarize(self.all_values)}", flush=True)
        if self.total_pixels:
            print(
                f"valid_ratio={self.valid_pixels/self.total_pixels:.6f} "
                f"nan_ratio={self.nan_pixels/self.total_pixels:.6f} "
                f"inf_ratio={self.inf_pixels/self.total_pixels:.6f} "
                f"nonpositive_ratio={self.nonpositive_pixels/self.total_pixels:.6f}", flush=True)
        for name, values in self.roi_values.items():
            print(f"roi_{name}={self.summarize(values)}", flush=True)
        total = sum(self.hist.values())
        print("histogram:", flush=True)
        for key in ("0-1m", "1-2m", "2-3m", "3-5m", "5-7m", "7-10m",
                    "10-15m", "15-20m", "20-infm"):
            count = self.hist[key]
            print(f"  {key}: count={count} percentage={(100*count/total if total else 0):.3f}%", flush=True)
        print(f"errors={dict(self.errors)}", flush=True)

    def check_done(self):
        if time.monotonic() - self.start >= self.duration:
            self.report()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--info-topic", default="/camera/depth/camera_info")
    args = parser.parse_args()
    rclpy.init()
    node = DepthDiagnostic(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
