import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from collection_interfaces.msg import SensorStatus

try:
    from velodyne_msgs.msg import VelodyneScan
except ImportError:
    VelodyneScan = None


class Monitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')
        defaults = {
            'lidar_packets_topic': '/velodyne_packets',
            'lidar_points_topic': '/velodyne_points',
            'imu_topic': '/imu',
            'camera_image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'glim_tf_topic': '/tf',
            'stale_after_sec': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.stale_after = float(self.get_parameter('stale_after_sec').value)
        topics = {name: self.get_parameter(param).value for name, param in (
            ('LiDAR Packets', 'lidar_packets_topic'),
            ('LiDAR Points', 'lidar_points_topic'),
            ('IMU', 'imu_topic'),
            ('Camera', 'camera_image_topic'),
            ('CameraInfo', 'camera_info_topic'),
            ('GLIM TF', 'glim_tf_topic'),
        )}
        self.data = {
            name: {'topic': topic, 'last': None, 'count': 0, 'prev': 0,
                   'valid': False, 'detail': 'Waiting for valid message'}
            for name, topic in topics.items()
        }
        self.pub = self.create_publisher(
            SensorStatus, '/collection/sensor_status', 20)
        if VelodyneScan is not None:
            self.create_subscription(
                VelodyneScan, topics['LiDAR Packets'], self.packet_cb,
                qos_profile_sensor_data)
        else:
            self.data['LiDAR Packets']['detail'] = 'velodyne_msgs is not installed'
        self.create_subscription(
            PointCloud2, topics['LiDAR Points'], self.points_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, topics['IMU'], self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, topics['Camera'], self.image_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, topics['CameraInfo'], self.camera_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, topics['GLIM TF'], self.tf_cb, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def accept(self, name, valid, detail):
        state = self.data[name]
        state['last'] = time.monotonic()
        state['count'] += 1
        state['valid'] = bool(valid)
        state['detail'] = detail

    def packet_cb(self, msg):
        self.accept('LiDAR Packets', bool(msg.packets),
                    f'{len(msg.packets)} packets' if msg.packets else 'Empty packet scan')

    def points_cb(self, msg):
        fields = {field.name for field in msg.fields}
        valid = msg.width * msg.height > 0 and bool(msg.data) and {'x', 'y', 'z'} <= fields
        self.accept('LiDAR Points', valid,
                    f'{msg.width * msg.height} points; frame={msg.header.frame_id}'
                    if valid else 'Empty/invalid PointCloud2')

    def imu_cb(self, msg):
        values = [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                  msg.orientation.w, msg.angular_velocity.x,
                  msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y,
                  msg.linear_acceleration.z]
        valid = all(math.isfinite(value) for value in values)
        self.accept('IMU', valid,
                    f'Finite IMU; frame={msg.header.frame_id}' if valid
                    else 'IMU contains non-finite values')

    def image_cb(self, msg):
        valid = msg.width > 0 and msg.height > 0 and bool(msg.data)
        detail = f'{msg.width}x{msg.height} {msg.encoding}; frame={msg.header.frame_id}'
        if valid and (msg.width, msg.height) != (1280, 720):
            detail += ' (expected 1280x720)'
        self.accept('Camera', valid, detail if valid else 'Empty image')

    def camera_info_cb(self, msg):
        valid = msg.width > 0 and msg.height > 0 and any(msg.k)
        self.accept('CameraInfo', valid,
                    f'{msg.width}x{msg.height}; frame={msg.header.frame_id}'
                    if valid else 'CameraInfo has zero dimensions/intrinsics')

    def tf_cb(self, msg):
        # Sensor static TF must not make GLIM look healthy. Require a map transform.
        transforms = [t for t in msg.transforms
                      if t.header.frame_id == 'map' or t.child_frame_id == 'map']
        if transforms:
            transform = transforms[-1]
            self.accept('GLIM TF', True,
                        f'{transform.header.frame_id}->{transform.child_frame_id}')

    def send(self, name, state, status, rate, age):
        msg = SensorStatus()
        msg.name = name
        msg.topic = state['topic']
        msg.status = status
        msg.measured_rate_hz = float(rate)
        msg.last_message_age_sec = float(age)
        msg.detail = state['detail']
        self.pub.publish(msg)

    def report(self):
        now = time.monotonic()
        for name, state in self.data.items():
            rate = state['count'] - state['prev']
            state['prev'] = state['count']
            if state['last'] is None:
                self.send(name, state, SensorStatus.WAITING, 0.0, -1.0)
                continue
            age = now - state['last']
            if not state['valid']:
                status = SensorStatus.ERROR
            elif age >= self.stale_after:
                status = SensorStatus.WARNING
                state['detail'] = 'Topic stale'
            else:
                status = SensorStatus.OK
            self.send(name, state, status, rate, age)
        try:
            fs = os.statvfs('/workspace/farm_ws')
            free = fs.f_bavail * fs.f_frsize / 1e9
            status = SensorStatus.OK if free >= 10 else SensorStatus.WARNING
            detail = f'{free:.1f} GB free'
        except OSError:
            free, status, detail = 0.0, SensorStatus.ERROR, 'Workspace unavailable'
        disk = {'topic': '', 'detail': detail}
        self.send('Disk', disk, status, 0.0, 0.0)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
