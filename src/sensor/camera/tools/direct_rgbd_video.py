#!/usr/bin/env python3
"""Direct V4L2 RGB + metric-depth viewer, without ROS image transport."""

from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from camera.depth_estimate import DepthEstimate


DEVICE = "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
CHECKPOINT = str(PACKAGE / "model/version6/last.pt")
CALIBRATION = str(PACKAGE / "config/decxin_camera_640x480.yaml")
WIDTH, HEIGHT, FPS = 640, 480, 30
DISPLAY_MAX_DEPTH = 10.0


class DirectRgbd:
    def __init__(self):
        # A one-slot mailbox keeps capture and inference real-time: old frames
        # are replaced instead of creating a latency queue.
        self.condition = threading.Condition()
        self.pending = None
        self.depth = None
        self.depth_rgb = None
        self.depth_frame_time = 0.0
        self.depth_done_time = 0.0
        self.rgb_count = 0
        self.depth_count = 0
        self.stop = False
        self.mouse = None

        with open(CALIBRATION, encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
        k = np.asarray(calibration["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        d = np.asarray(calibration["distortion_coefficients"]["data"], dtype=np.float64)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            k, d, np.eye(3), k, (WIDTH, HEIGHT), cv2.CV_32FC1)
        self.intrinsics = (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]))

        print(f"Loading {CHECKPOINT}", flush=True)
        self.model = DepthEstimate(CHECKPOINT, "cuda")
        self.cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open {DEVICE}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        # Two buffers preserve the UVC camera's capture rate; one buffer makes
        # this camera fall to roughly half speed in OpenCV/V4L2.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(
            f"Camera: {actual_width:.0f}x{actual_height:.0f} "
            f"requested={FPS} FPS, driver reports={actual_fps:.1f} FPS",
            flush=True,
        )

        self.worker = threading.Thread(target=self.infer, daemon=True)
        self.worker.start()

    def infer(self):
        while not self.stop:
            with self.condition:
                self.condition.wait_for(lambda: self.stop or self.pending is not None)
                if self.stop:
                    return
                rgb, frame_time = self.pending
                self.pending = None
            try:
                depth = self.model.predict(rgb, self.intrinsics)
                with self.condition:
                    self.depth = cv2.rotate(depth, cv2.ROTATE_180)
                    self.depth_rgb = cv2.rotate(
                        cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), cv2.ROTATE_180)
                    self.depth_frame_time = frame_time
                    self.depth_done_time = time.monotonic()
                    self.depth_count += 1
            except Exception as exc:
                print(f"depth inference failed: {exc}", flush=True)

    def on_mouse(self, event, x, y, flags, userdata):
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_MOUSEMOVE):
            self.mouse = (x, y)

    def render_depth(self, depth):
        valid = np.isfinite(depth) & (depth > 0)
        scaled = np.zeros(depth.shape, np.uint8)
        scaled[valid] = (np.clip(depth[valid] / DISPLAY_MAX_DEPTH, 0, 1) * 255).astype(np.uint8)
        image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        image[~valid] = 0
        if self.mouse is not None:
            x, y = self.mouse
            if 0 <= x < WIDTH and 0 <= y < HEIGHT:
                z = float(depth[y, x])
                text = f"({x}, {y})  {z:.3f} m ({z * 100:.1f} cm)" if valid[y, x] else "INVALID"
                cv2.putText(image, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .65,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.drawMarker(image, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 15, 1)
        cv2.putText(image, f"Depth 0-{DISPLAY_MAX_DEPTH:g} m | left-click to inspect | Q/Esc quit",
                    (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return image

    @staticmethod
    def rate(count, start):
        return count / max(time.monotonic() - start, 1e-3)

    def run(self):
        cv2.setNumThreads(1)
        cv2.namedWindow("RGB", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Depth", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("Depth", self.on_mouse)
        cv2.setMouseCallback("RGB", self.on_mouse)
        frames = 0
        started = time.monotonic()
        try:
            while True:
                ok, bgr = self.cap.read()
                if not ok:
                    continue
                frame_time = time.monotonic()
                bgr = cv2.remap(bgr, self.map1, self.map2, cv2.INTER_LINEAR)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                with self.condition:
                    # Replace an unfinished frame; never queue stale frames.
                    self.pending = (rgb, frame_time)
                    self.condition.notify()
                    depth = None if self.depth is None else self.depth.copy()
                    paired_rgb = None if self.depth_rgb is None else self.depth_rgb.copy()
                    depth_age = (time.monotonic() - self.depth_frame_time
                                 if self.depth is not None else 0)
                frames += 1
                if paired_rgb is not None:
                    bgr = paired_rgb
                else:
                    bgr = cv2.rotate(bgr, cv2.ROTATE_180)
                self.rgb_count = frames
                rgb_fps = self.rate(self.rgb_count, started)
                depth_fps = self.rate(self.depth_count, started)
                cv2.putText(bgr, f"RGB {rgb_fps:.1f}/{FPS} FPS",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("RGB", bgr)
                if depth is None:
                    canvas = np.zeros_like(bgr)
                    cv2.putText(canvas, "Waiting for depth model...", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
                else:
                    canvas = self.render_depth(depth)
                    cv2.putText(canvas, f"Depth {depth_fps:.1f}/{FPS} FPS | age {depth_age * 1000:.0f} ms", (10, 52),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                cv2.imshow("Depth", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or any(
                    cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1
                    for name in ("RGB", "Depth")
                ):
                    break
        finally:
            self.stop = True
            self.worker.join(timeout=2)
            self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    DirectRgbd().run()
