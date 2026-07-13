#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuRPYViewer(Node):
    def __init__(self):
        super().__init__("imu_rpy_viewer")
        self.create_subscription(Imu, "/imu", self.cb, 10)

    def cb(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)

        print(
            f"roll={math.degrees(roll):7.2f} deg, "
            f"pitch={math.degrees(pitch):7.2f} deg, "
            f"yaw={math.degrees(yaw):7.2f} deg"
        )


def main():
    rclpy.init()
    node = ImuRPYViewer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
