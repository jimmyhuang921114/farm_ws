#!/usr/bin/env python3
"""ROS2 <-> STM32 FOC bridge with fixed-mechanical-zero yaw feedback.

- Preserve existing relative-degree person-tracking commands.
- Publish existing STM32 control telemetry.
- Publish actual mechanical yaw (radians) and PoseStamped.
- Default to STM32's automatic YAW: telemetry; optional YAW? polling.
"""

import math
import re
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped


# 支援舊格式：Angle ... | Target ... | Vel ...
# 以及新版：Angle ... | Target ... | Error ... | Vel ... | Drops ...
NUMBER = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)'
TELEMETRY_PATTERN = re.compile(
    rf'Angle\s+({NUMBER})\s+deg\s*\|\s*'
    rf'Target\s+({NUMBER})\s+deg\s*\|\s*'
    rf'(?:Error\s+{NUMBER}\s+deg\s*\|\s*)?'
    rf'Vel\s+({NUMBER})\s+rad/s'
)
YAW_PATTERN = re.compile(rf'^YAW:\s*({NUMBER})\s*$')


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # -------------------- ROS2 Parameters --------------------
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_interval', 0.02)
        self.declare_parameter('angle_deadband', 0.0)
        self.declare_parameter('pose_frame_id', 'gimbal_base')
        # STM32 已每 200 ms 自動傳 YAW:，預設不主動查詢。
        # 設成 0.1 才會每 100 ms 另外送出 YAW?。
        self.declare_parameter('yaw_query_interval', 0.0)

        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        self.send_interval = float(self.get_parameter('send_interval').value)
        self.angle_deadband = float(self.get_parameter('angle_deadband').value)
        self.pose_frame_id = str(self.get_parameter('pose_frame_id').value)
        self.yaw_query_interval = float(self.get_parameter('yaw_query_interval').value)

        if self.send_interval <= 0:
            raise ValueError('send_interval must be > 0')
        if self.angle_deadband < 0:
            raise ValueError('angle_deadband must be >= 0')
        if self.yaw_query_interval < 0:
            raise ValueError('yaw_query_interval must be >= 0')

        # -------------------- Person Tracker state --------------------
        self.latest_angle = 0.0
        self.have_angle = False
        self.target_locked = False
        self.was_in_deadband = False
        self.last_send_time = time.monotonic() - self.send_interval

        # -------------------- Serial --------------------
        self.rx_buffer = bytearray()
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0,
                write_timeout=0.2,
            )
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Cannot open serial port: {exc}')
            raise

        # -------------------- Subscribers --------------------
        self.angle_sub = self.create_subscription(
            Float32, '/person_tracker/error_angle', self.angle_callback, 10
        )
        self.locked_sub = self.create_subscription(
            Bool, '/person_tracker/locked', self.locked_callback, 10
        )

        # -------------------- Publishers (original) --------------------
        self.angle_pub = self.create_publisher(Float32, '/farmer/angle', 10)
        self.target_pub = self.create_publisher(Float32, '/farmer/target', 10)
        self.speed_pub = self.create_publisher(Float32, '/farmer/speed', 10)

        # -------------------- Publishers (new) --------------------
        # /gimbal/yaw : radians (actual Yaw, not target angle)
        self.yaw_pub = self.create_publisher(Float32, '/gimbal/yaw', 10)
        # /gimbal/pose : orientation around Z relative to gimbal_base
        self.pose_pub = self.create_publisher(PoseStamped, '/gimbal/pose', 10)

        # -------------------- Timers --------------------
        self.serial_timer = self.create_timer(0.01, self.read_serial)
        self.send_timer = self.create_timer(0.01, self.try_send_latest_angle)
        self.yaw_query_timer = None
        if self.yaw_query_interval > 0:
            self.yaw_query_timer = self.create_timer(
                self.yaw_query_interval, self.query_yaw
            )

        self.get_logger().info(f'Connected to STM32: {port} @ {baudrate}')
        self.get_logger().info(
            'Subscribing: /person_tracker/error_angle, /person_tracker/locked'
        )
        self.get_logger().info(
            'Publishing: /farmer/angle, /farmer/target, /farmer/speed'
        )
        self.get_logger().info('Publishing: /gimbal/yaw (rad), /gimbal/pose')
        self.get_logger().info(
            f'Send interval: {self.send_interval:.3f} s, '
            f'Deadband: +/-{self.angle_deadband:.2f} deg'
        )
        self.get_logger().info(
            'Yaw source: STM32 periodic telemetry'
            if self.yaw_query_interval == 0
            else f'Yaw polling interval: {self.yaw_query_interval:.3f} s'
        )

    # -------------------- Person Tracker callbacks --------------------
    def angle_callback(self, msg):
        self.latest_angle = float(msg.data)
        self.have_angle = True

    def locked_callback(self, msg):
        new_locked_state = bool(msg.data)
        if self.target_locked and not new_locked_state:
            self.get_logger().warn(
                'Target lost. Stop sending new angle commands.'
            )
            self.have_angle = False
            self.latest_angle = 0.0
        elif not self.target_locked and new_locked_state:
            self.get_logger().info('Target locked. Angle control enabled.')
            self.last_send_time = time.monotonic() - self.send_interval
        self.target_locked = new_locked_state

    # -------------------- ROS2 -> STM32 (original control) --------------------
    def try_send_latest_angle(self):
        if not self.target_locked or not self.have_angle:
            return

        angle = self.latest_angle
        if not math.isfinite(angle):
            return

        # 與原版一致：誤差進入死區時，不送角度指令。
        if abs(angle) <= self.angle_deadband:
            if not self.was_in_deadband:
                self.get_logger().info(
                    f'Angle {angle:+.2f} deg inside deadband '
                    f'+/-{self.angle_deadband:.2f} deg'
                )
            self.was_in_deadband = True
            return

        self.was_in_deadband = False
        now = time.monotonic()
        if now - self.last_send_time < self.send_interval:
            return

        self.send_angle_to_stm32(angle)
        self.last_send_time = now

    def send_angle_to_stm32(self, angle):
        # 保持原本協定：STM32 收到的是「相對角度（度）」。
        command = f'{angle:.4f}\n'
        try:
            self.ser.write(command.encode('ascii'))
            self.get_logger().info(f'ROS2 -> STM32: {angle:+.2f} deg')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    # -------------------- Optional yaw request --------------------
    def query_yaw(self):
        # 僅查詢角度，不修改馬達 target；正常情況不需要啟用。
        try:
            self.ser.write(b'YAW?\n')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Yaw query error: {exc}')

    # -------------------- STM32 -> ROS2 --------------------
    def read_serial(self):
        try:
            available = self.ser.in_waiting
            if available <= 0:
                return

            # 讀取所有目前可用資料，處理串口一次收到半行或多行的情況。
            self.rx_buffer.extend(self.ser.read(min(available, 4096)))
            if len(self.rx_buffer) > 8192:
                self.rx_buffer.clear()
                self.get_logger().warn('Serial RX buffer overflow; cleared')
                return

            while b'\n' in self.rx_buffer:
                raw_line, _, remainder = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(remainder)
                line = raw_line.decode('utf-8', errors='ignore').strip()
                self.process_line(line)

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial read error: {exc}')

    def process_line(self, line):
        if not line:
            return

        # 1. Fixed mechanical Yaw feedback; STM32 sends degrees.
        yaw_match = YAW_PATTERN.fullmatch(line)
        if yaw_match is not None:
            yaw_deg = float(yaw_match.group(1))
            if not math.isfinite(yaw_deg):
                return
            yaw_rad = math.radians(yaw_deg)

            self.yaw_pub.publish(Float32(data=yaw_rad))

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.pose_frame_id
            # position.x/y/z are 0 by default: relative rotation at base origin.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            self.pose_pub.publish(pose)
            return

        # 2. Original PID telemetry; support both old and new STM32 formats.
        match = TELEMETRY_PATTERN.search(line)
        if match is not None:
            angle_deg, target_deg, velocity_rad_s = map(float, match.groups())
            if all(math.isfinite(v) for v in
                   (angle_deg, target_deg, velocity_rad_s)):
                # 保留原版 topic 單位：角度=度、速度=rad/s。
                self.angle_pub.publish(Float32(data=angle_deg))
                self.target_pub.publish(Float32(data=target_deg))
                self.speed_pub.publish(Float32(data=velocity_rad_s))
            return

        # 3. STM32 status / error messages.
        if any(word in line for word in (
            'FOC', 'Command', 'Direction', 'AS5600',
            'ERROR', 'FAULT', 'YAW UNCALIBRATED', 'YAW OUTSIDE LIMIT'
        )):
            self.get_logger().info(f'STM32 -> {line}')

    # -------------------- Cleanup --------------------
    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()