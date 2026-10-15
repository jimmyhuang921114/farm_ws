"""Read-only ROS RGB/depth viewer with raw metric depth inspection."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def decode_image(msg):
    """Decode supported ROS encodings, including padded rows and byte order."""
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3),
               '32FC1': ('f4', 1), '16UC1': ('u2', 1)}
    if msg.encoding not in formats:
        raise ValueError(f'Unsupported encoding: {msg.encoding}')
    kind, channels = formats[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
        raise ValueError('Invalid image row stride or data length')
    shape = (msg.height, msg.width, channels) if channels == 3 else (msg.height, msg.width)
    strides = (msg.step, channels * dtype.itemsize, dtype.itemsize) if channels == 3 else (msg.step, dtype.itemsize)
    a = np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if msg.encoding == '16UC1':
        return a.astype(np.float32) * 0.001
    return a


class RgbdViewer(Node):
    def __init__(self):
        super().__init__('rgbd_pixel_viewer')
        rgb_topic = self.declare_parameter('rgb_topic', '/camera/color/image_raw').value
        depth_topic = self.declare_parameter('depth_topic', '/camera/depth/image_raw').value
        self.max_depth = float(self.declare_parameter('display_max_depth_m', 10.0).value)
        if not np.isfinite(self.max_depth) or self.max_depth <= 0:
            raise ValueError('display_max_depth_m must be finite and positive')
        self.frames = {}
        self.received = {}
        self.mouse = None
        self.paused = False
        self.last_error = 0.0
        self.subs = [
            self.create_subscription(Image, rgb_topic, lambda m: self.receive('RGB', m), qos_profile_sensor_data),
            self.create_subscription(Image, depth_topic, lambda m: self.receive('Depth', m), qos_profile_sensor_data),
        ]
        for name in ('RGB', 'Depth'):
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow('RGB', 20, 50)
        cv2.moveWindow('Depth', 680, 50)
        cv2.setMouseCallback('Depth', self.on_mouse)
        self.get_logger().info(f'Viewing {rgb_topic} and {depth_topic}; hover Depth, Space pauses, Q exits')

    def receive(self, name, msg):
        if self.paused:
            return
        try:
            if name == 'Depth' and msg.encoding not in ('32FC1', '16UC1'):
                raise ValueError(f'Depth encoding must be 32FC1 or 16UC1, got {msg.encoding}')
            if name == 'RGB' and msg.encoding not in ('rgb8', 'bgr8'):
                raise ValueError(f'RGB encoding must be rgb8 or bgr8, got {msg.encoding}')
            self.frames[name] = decode_image(msg)
            self.received[name] = time.monotonic()
        except (ValueError, TypeError) as exc:
            if time.monotonic() - self.last_error > 5:
                self.get_logger().warning(str(exc))
                self.last_error = time.monotonic()

    def on_mouse(self, event, x, y, flags, userdata):
        self.mouse = (x, y)

    def render(self, name):
        a = self.frames.get(name)
        if a is None:
            canvas = np.zeros((480, 640, 3), np.uint8)
            lines = [f'Waiting for {name}...', 'Space: pause/resume | Q / Esc: close viewer']
        else:
            h, w = a.shape[:2]
            dh = max(1, round(h * 640 / w))
            lines = []
            age = time.monotonic() - self.received[name]
            state = 'PAUSED' if self.paused else ('STALE' if age > 2 else 'LIVE')
            if name == 'Depth':
                valid = np.isfinite(a) & (a > 0)
                scaled = np.zeros(a.shape, np.uint8)
                scaled[valid] = (np.clip(a[valid] / self.max_depth, 0, 1) * 255).astype(np.uint8)
                color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
                color[~valid] = 0
                canvas = cv2.resize(color, (640, dh), interpolation=cv2.INTER_NEAREST)
                lines.append('Hover image to inspect raw depth (meters)')
                if self.mouse is not None:
                    mx, my = self.mouse
                    if 0 <= mx < 640 and 0 <= my < dh:
                        x, y = min(w - 1, int(mx * w / 640)), min(h - 1, int(my * h / dh))
                        z = float(a[y, x])
                        value = f'{z:.4f} m ({z * 100:.1f} cm)' if valid[y, x] else 'INVALID (zero / NaN / Inf)'
                        lines[0] = f'Pixel ({x}, {y})  Depth: {value}'
                        cv2.drawMarker(canvas, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 15, 1)
                lines.append(f'Color: 0 .. {self.max_depth:g} m (clipped); readout is NOT clipped')
            else:
                canvas = cv2.resize(a, (640, dh))
                lines.append('RGB - latest received frame (not synchronized pair)')
                lines.append('Depth is predicted camera-axis Z, not radial range')
            lines.append(f'{state} | age {age:.1f}s | {w}x{h} | Space: pause | Q: quit')
        footer = np.zeros((90, 640, 3), np.uint8)
        for i, line in enumerate(lines):
            cv2.putText(footer, line, (10, 23 + i * 27), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(name, np.vstack((canvas, footer)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RgbdViewer()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.render('RGB')
            node.render('Depth')
            key = cv2.waitKey(15) & 0xff
            if key in (27, ord('q')):
                break
            if key == ord(' '):
                node.paused = not node.paused
            if any(cv2.getWindowProperty(n, cv2.WND_PROP_AUTOSIZE) < 0 for n in ('RGB', 'Depth')):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
