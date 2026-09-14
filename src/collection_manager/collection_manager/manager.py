import os
import shutil
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from collection_interfaces.msg import CollectionEvent, CollectionState
from collection_interfaces.srv import AddMarker, PreflightCollection, StartCollection, StopCollection
from rclpy.node import Node

from .session import SessionStore


class CollectionManager(Node):
    """Own session metadata and optionally a ros2 bag record subprocess."""

    def __init__(self):
        super().__init__('collection_manager')
        self.declare_parameter('session_root', '/workspace/farm_ws/mapping_sessions')
        self.declare_parameter('recording_enabled', False)
        self.declare_parameter(
            'record_topics',
            '/livox/lidar /livox/lidar_valid /livox/imu /livox/imu_base '
            '/tf /tf_static /decxin_camera/image_compressed '
            '/decxin_camera/camera_info /glim_ros/odom /glim_ros/points /glim_ros/map',
        )
        self.declare_parameter('storage_id', 'sqlite3')
        self.declare_parameter('compression_mode', 'file')
        self.declare_parameter('compression_format', 'zstd')
        self.declare_parameter('record_polling_interval', 100)
        self.store = SessionStore(self.get_parameter('session_root').value)
        self.state = CollectionState.IDLE
        self.duration = 0.0
        self.start_at = None
        self.countdown_at = None
        self.recording_enabled = bool(self.get_parameter('recording_enabled').value)
        self.record_topics = self._split_topics(self.get_parameter('record_topics').value)
        self.storage_id = str(self.get_parameter('storage_id').value)
        self.compression_mode = str(self.get_parameter('compression_mode').value)
        self.compression_format = str(self.get_parameter('compression_format').value)
        self.record_polling_interval = int(self.get_parameter('record_polling_interval').value)
        self.recorder = None
        self.recording_log = None
        self.bag_path = None

        self.pub = self.create_publisher(CollectionState, '/collection/state', 10)
        self.events = self.create_publisher(CollectionEvent, '/collection/events', 10)
        self.create_service(PreflightCollection, '/collection/preflight', self.preflight)
        self.create_service(StartCollection, '/collection/start', self.start)
        self.create_service(StopCollection, '/collection/stop', self.stop)
        self.create_service(AddMarker, '/collection/add_marker', self.marker)
        self.create_timer(0.1, self.tick)

    @staticmethod
    def _split_topics(value):
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value if str(item)]
        return [item for item in str(value).replace(',', ' ').split() if item]

    def emit(self, message, level='INFO', marker=''):
        self.store.write_event(level, message, marker)
        event = CollectionEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.level = level
        event.message = message
        event.marker = marker
        self.events.publish(event)

    def preflight(self, request, response):
        if self.state in (CollectionState.COUNTDOWN, CollectionState.RECORDING):
            response.success = False
            response.message = 'Collection active'
            return response
        self.state = CollectionState.PREFLIGHT
        self.duration = max(0.1, float(request.duration_sec))
        cfg = dict(
            session_name=request.session_name,
            location=request.location,
            duration_sec=self.duration,
            profile=request.profile,
            note=request.note,
            ui_only=bool(request.ui_only),
        )
        if not request.ui_only and not self.recording_enabled:
            self.state = CollectionState.FAILED
            response.success = False
            response.message = 'Real rosbag2 recording is disabled for this launch'
            return response
        path = self.store.create(cfg)
        self.state = CollectionState.IDLE
        response.success = True
        response.message = 'Ready'
        response.session_path = str(path)
        return response

    def start(self, _request, response):
        if not self.store.path:
            response.success = False
            response.message = 'Run preflight first'
            return response
        if self.state not in (CollectionState.IDLE, CollectionState.COMPLETED):
            response.success = False
            response.message = 'Collection active'
            return response
        self.state = CollectionState.COUNTDOWN
        self.countdown_at = time.monotonic()
        self.start_at = None
        self.emit('Countdown started')
        response.success = True
        response.message = '3 second countdown'
        return response

    def start_recording(self):
        if not self.recording_enabled or not self.store.path:
            return True
        if self.recorder is not None:
            return self.recorder.poll() is None
        ros2 = shutil.which('ros2')
        if not ros2:
            self.emit('ros2 executable not found; recording failed', 'ERROR')
            return False
        self.bag_path = self.store.path / 'rosbag'
        log_path = self.store.path / 'recording.log'
        command = [
            ros2, 'bag', 'record', '-o', str(self.bag_path), '-s', self.storage_id,
            '--polling-interval', str(self.record_polling_interval), *self.record_topics,
        ]
        if self.compression_mode != 'none':
            command[command.index('--polling-interval'):command.index('--polling-interval')] = [
                '--compression-mode', self.compression_mode,
                '--compression-format', self.compression_format,
            ]
        try:
            self.recording_log = log_path.open('w', encoding='utf-8')
            self.recorder = subprocess.Popen(
                command,
                stdout=self.recording_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.store.update_config(
                bag_path=str(self.bag_path),
                lidar_topic=next((x for x in self.record_topics if 'velodyne' in x or 'lidar' in x), ''),
                imu_topic=next((x for x in self.record_topics if x.endswith('/imu')), ''),
                rgb_topic=next((x for x in self.record_topics if 'image' in x and 'camera_info' not in x), ''),
                camera_info_topic=next((x for x in self.record_topics if 'camera_info' in x), ''),
                frames=['base_link', 'velodyne', 'imu_link', 'camera_link'],
                start_time=datetime.now().astimezone().isoformat(),
                contains_real_sensor_data=True,
                recording_command=' '.join(command),
                storage_id=self.storage_id,
                compression_mode=self.compression_mode,
                compression_format=self.compression_format,
            )
            self.emit('rosbag2 recording started')
            return True
        except (OSError, ValueError) as exc:
            self.emit(f'rosbag2 start failed: {exc}', 'ERROR')
            if self.recording_log:
                self.recording_log.close()
                self.recording_log = None
            return False

    def stop_recording(self):
        process = self.recorder
        self.recorder = None
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        process.kill()
                    except ProcessLookupError:
                        pass
        if self.recording_log:
            self.recording_log.close()
            self.recording_log = None
        if self.bag_path and self.bag_path.exists():
            self.store.update_config(contains_rosbag=True, bag_path=str(self.bag_path))
        if process is not None:
            self.emit('rosbag2 recording stopped')

    def finish(self, reason='Duration completed'):
        self.state = CollectionState.FINALIZING
        self.emit(reason)
        self.stop_recording()
        self.state = CollectionState.VERIFYING
        if self.store.path:
            self.store.update_config(
                end_time=datetime.now().astimezone().isoformat(),
                contains_real_sensor_data=bool(self.bag_path and self.bag_path.exists()),
                contains_rosbag=bool(self.bag_path and self.bag_path.exists()),
            )
        self.state = CollectionState.COMPLETED
        self.emit('Session completed')

    def stop(self, _request, response):
        if self.state not in (CollectionState.COUNTDOWN, CollectionState.RECORDING):
            response.success = False
            response.message = 'Not recording'
            return response
        self.finish('Stopped by user')
        response.success = True
        response.message = 'Stopped'
        return response

    def marker(self, request, response):
        if not self.store.path:
            response.success = False
            response.message = 'No session'
            return response
        self.emit('Marker added', 'INFO', request.label or 'marker')
        response.success = True
        response.message = 'Marker saved'
        return response

    def tick(self):
        now = time.monotonic()
        if self.state == CollectionState.COUNTDOWN and now - self.countdown_at >= 3.0:
            if not self.start_recording():
                self.state = CollectionState.FAILED
                self.emit('Recording failed', 'ERROR')
            else:
                self.state = CollectionState.RECORDING
                self.start_at = now
                self.emit('Recording timer started')
        if self.state == CollectionState.RECORDING and now - self.start_at >= self.duration:
            self.finish()
        message = CollectionState()
        message.state = self.state
        message.session_path = str(self.store.path or '')
        if self.start_at:
            message.elapsed_sec = float(max(0.0, now - self.start_at))
            message.remaining_sec = float(max(0.0, self.duration - message.elapsed_sec))
        try:
            stat = os.statvfs(self.store.root)
            message.disk_free_gb = stat.f_bavail * stat.f_frsize / 1e9
        except OSError:
            message.disk_free_gb = 0.0
        message.bag_size_bytes = self._bag_size()
        message.detail = (
            'rosbag2 recording enabled' if self.recording_enabled
            else 'UI-only; no rosbag2 is created'
        )
        self.pub.publish(message)

    def _bag_size(self):
        if not self.store.path:
            return 0
        return sum(path.stat().st_size for path in self.store.path.rglob('*') if path.is_file())


def main():
    rclpy.init()
    node = CollectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()
