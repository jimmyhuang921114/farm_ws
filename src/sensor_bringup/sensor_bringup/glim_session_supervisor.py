#!/usr/bin/env python3
"""Own GLIM process groups and split sessions at sensor discontinuities."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class GlimSessionSupervisor(Node):
    """Restart only GLIM and preserve an explicit boundary between sessions."""

    def __init__(self) -> None:
        super().__init__('glim_session_supervisor')
        self.declare_parameter('config_path', '/workspace/farm_ws/config/livox_mid360')
        self.declare_parameter('runtime_directory', '/tmp/farm_ws_runtime/manual')
        self.declare_parameter('max_automatic_restarts', 3)
        self.declare_parameter('backoff_initial_sec', 2.0)
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.config_path = str(self.get_parameter('config_path').value)
        self.runtime_directory = Path(
            str(self.get_parameter('runtime_directory').value)
        )
        self.max_restarts = int(
            self.get_parameter('max_automatic_restarts').value
        )
        self.backoff_initial = float(
            self.get_parameter('backoff_initial_sec').value
        )
        self.startup_timeout = float(
            self.get_parameter('startup_timeout_sec').value
        )
        self.sessions_directory = self.runtime_directory / 'glim_sessions'
        self.sessions_directory.mkdir(parents=True, exist_ok=True)
        self.events_path = self.sessions_directory / 'session_events.jsonl'
        self.events_file = self.events_path.open(
            'a', encoding='utf-8', buffering=1
        )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/livox/continuity/state', self.state_callback, transient_qos
        )
        self.create_subscription(
            Bool,
            '/livox/continuity/restart_required',
            self.restart_callback,
            transient_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            '/livox/continuity/diagnostics',
            self.continuity_diagnostic_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            '/glim_ros/odom',
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            '/glim_ros/points',
            self.points_callback,
            qos_profile_sensor_data,
        )
        self.ready_publisher = self.create_publisher(
            Bool, '/glim/session_supervisor/ready', transient_qos
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/glim/session_supervisor/diagnostics', 10
        )
        self.ack_client = self.create_client(
            Trigger, '/livox/continuity/acknowledge_restart'
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.process: Optional[subprocess.Popen] = None
        self.process_expected_exit = False
        self.continuity_state = 'UNKNOWN'
        self.restart_required = False
        self.handling_sensor_restart = False
        self.pending_reason = ''
        self.continuity_values: Dict[str, str] = {}
        self.session_index = 0
        self.restart_count = 0
        self.crash_count = 0
        self.next_start_time = 0.0
        self.session_started_monotonic = 0.0
        self.session_ready = False
        self.received_odom = False
        self.received_points = False
        self.disabled = False
        self.publish_ready(False)

    def state_callback(self, message: String) -> None:
        self.continuity_state = message.data

    def restart_callback(self, message: Bool) -> None:
        if message.data and not self.restart_required:
            self.restart_required = True
            self.pending_reason = self.continuity_values.get(
                'reason', 'sensor_discontinuity'
            )

    def continuity_diagnostic_callback(self, message: DiagnosticArray) -> None:
        if not message.status:
            return
        status = message.status[0]
        self.continuity_values = {
            value.key: value.value for value in status.values
        }
        if self.restart_required:
            self.pending_reason = self.continuity_values.get(
                'reason', self.pending_reason
            )

    def odom_callback(self, _message: Odometry) -> None:
        if self.process is not None:
            self.received_odom = True

    def points_callback(self, _message: PointCloud2) -> None:
        if self.process is not None:
            self.received_points = True

    def timer_callback(self) -> None:
        now = time.monotonic()
        if self.process is not None:
            return_code = self.process.poll()
            if return_code is not None:
                expected = self.process_expected_exit
                self.process = None
                self.publish_ready(False)
                if not expected:
                    self.crash_count += 1
                    self.pending_reason = f'glim_unexpected_exit_{return_code}'
                    self.record_event('unexpected_exit', return_code=return_code)
                    self.schedule_restart(now, count_restart=True)
                self.process_expected_exit = False
            elif self.restart_required and not self.handling_sensor_restart:
                self.handling_sensor_restart = True
                self.stop_session('sensor_discontinuity')
                self.schedule_restart(now, count_restart=True)
            elif self.received_odom and self.received_points and not self.session_ready:
                self.session_ready = True
                self.publish_ready(True)
                self.record_event('session_ready')
                if self.restart_required:
                    self.acknowledge_continuity_restart()
            elif (
                not self.session_ready
                and now - self.session_started_monotonic > self.startup_timeout
            ):
                self.pending_reason = 'glim_startup_timeout'
                self.stop_session('startup_timeout')
                self.schedule_restart(now, count_restart=True)

        if (
            self.process is None
            and not self.disabled
            and self.continuity_state == 'HEALTHY'
            and now >= self.next_start_time
        ):
            self.start_session()
        self.publish_diagnostics()

    def schedule_restart(self, now: float, count_restart: bool) -> None:
        if count_restart:
            self.restart_count += 1
        if self.restart_count > self.max_restarts:
            self.disabled = True
            self.record_event('restart_limit_reached')
            return
        delay = self.backoff_initial * (2 ** max(0, self.restart_count - 1))
        self.next_start_time = now + delay
        self.record_event('restart_scheduled', backoff_sec=delay)

    def start_session(self) -> None:
        self.session_index += 1
        timestamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        session_directory = self.sessions_directory / (
            f'session_{self.session_index:03d}_{timestamp}'
        )
        session_directory.mkdir(parents=True, exist_ok=False)
        log_handle = (session_directory / 'glim.log').open('ab', buffering=0)
        command = [
            'ros2', 'run', 'glim_ros', 'glim_rosnode', '--ros-args',
            '-p', f'config_path:={self.config_path}',
            '-p', f'dump_path:={session_directory / "dump"}',
        ]
        self.process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log_handle.close()
        self.process_expected_exit = False
        self.session_started_monotonic = time.monotonic()
        self.session_ready = False
        self.received_odom = False
        self.received_points = False
        self.current_session_directory = session_directory
        self.record_event('session_started', pid=self.process.pid)

    def stop_session(self, event: str) -> None:
        if self.process is None:
            return
        self.process_expected_exit = True
        self.record_event(event, pid=self.process.pid)
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        self.process = None
        self.publish_ready(False)

    def acknowledge_continuity_restart(self) -> None:
        if not self.ack_client.wait_for_service(timeout_sec=2.0):
            self.record_event('restart_ack_service_unavailable')
            return
        future = self.ack_client.call_async(Trigger.Request())
        future.add_done_callback(self.acknowledge_callback)

    def acknowledge_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.record_event('restart_ack_failed', error=str(error))
            return
        if response.success:
            self.restart_required = False
            self.handling_sensor_restart = False
            self.pending_reason = ''
            self.record_event('restart_acknowledged')
        else:
            self.record_event('restart_ack_rejected', message=response.message)

    def record_event(self, event: str, **extra) -> None:
        record = {
            'wall_time': time.time(),
            'event': event,
            'session_index': self.session_index,
            'continuity_state': self.continuity_state,
            'restart_required': self.restart_required,
            'handling_sensor_restart': self.handling_sensor_restart,
            'reason': self.pending_reason,
            'last_good_lidar_stamp': self.continuity_values.get(
                'last_good_lidar_stamp'
            ),
            'last_good_imu_stamp': self.continuity_values.get(
                'last_good_imu_stamp'
            ),
            **extra,
        }
        self.events_file.write(json.dumps(record, separators=(',', ':')) + '\n')

    def publish_ready(self, ready: bool) -> None:
        if not rclpy.ok():
            return
        message = Bool()
        message.data = ready
        self.ready_publisher.publish(message)

    def publish_diagnostics(self) -> None:
        self.publish_ready(self.session_ready)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'glim_session_supervisor'
        status.hardware_id = 'glim'
        if self.disabled:
            status.level = DiagnosticStatus.ERROR
            status.message = 'restart_limit_reached'
        elif self.session_ready:
            status.level = DiagnosticStatus.OK
            status.message = 'session_ready'
        else:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting_or_starting'
        values = {
            'continuity_state': self.continuity_state,
            'restart_required': self.restart_required,
            'session_index': self.session_index,
            'restart_count': self.restart_count,
            'crash_count': self.crash_count,
            'session_ready': self.session_ready,
            'reason': self.pending_reason,
        }
        status.values = [
            KeyValue(key=key, value=str(value)) for key, value in values.items()
        ]
        array.status = [status]
        self.diagnostic_publisher.publish(array)

    def destroy_node(self) -> bool:
        self.stop_session('supervisor_shutdown')
        self.events_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlimSessionSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
