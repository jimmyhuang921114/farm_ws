#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ImuVisualizer(Node):

    def __init__(self) -> None:
        super().__init__('imu_visualizer')

        self.declare_parameter(
            'input_topic',
            '/livox/imu_base',
        )
        self.declare_parameter(
            'marker_topic',
            '/livox/imu_markers',
        )
        self.declare_parameter(
            'acceleration_arrow_scale',
            0.10,
        )
        self.declare_parameter(
            'gyro_arrow_scale',
            5.0,
        )

        self.input_topic = str(
            self.get_parameter('input_topic').value
        )
        self.marker_topic = str(
            self.get_parameter('marker_topic').value
        )
        self.acceleration_arrow_scale = float(
            self.get_parameter(
                'acceleration_arrow_scale'
            ).value
        )
        self.gyro_arrow_scale = float(
            self.get_parameter(
                'gyro_arrow_scale'
            ).value
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'IMU input topic: {self.input_topic}'
        )
        self.get_logger().info(
            f'RViz marker topic: {self.marker_topic}'
        )

    @staticmethod
    def make_point(
        x: float,
        y: float,
        z: float,
    ) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def make_arrow(
        self,
        message: Imu,
        marker_id: int,
        namespace: str,
        end_x: float,
        end_y: float,
        end_z: float,
        red: float,
        green: float,
        blue: float,
    ) -> Marker:
        marker = Marker()

        marker.header.stamp = message.header.stamp
        marker.header.frame_id = message.header.frame_id

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # For an ARROW using points:
        # scale.x = shaft diameter
        # scale.y = head diameter
        # scale.z = head length
        marker.scale.x = 0.025
        marker.scale.y = 0.060
        marker.scale.z = 0.080

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        marker.points = [
            self.make_point(0.0, 0.0, 0.0),
            self.make_point(end_x, end_y, end_z),
        ]

        return marker

    def make_text(
        self,
        message: Imu,
        acceleration_magnitude: float,
        gyro_magnitude: float,
    ) -> Marker:
        marker = Marker()

        marker.header.stamp = message.header.stamp
        marker.header.frame_id = message.header.frame_id

        marker.ns = 'imu_text'
        marker.id = 2

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = (
            f'Acceleration: {acceleration_magnitude:.2f} m/s^2\n'
            f'Angular speed: {gyro_magnitude:.3f} rad/s'
        )

        return marker

    def imu_callback(self, message: Imu) -> None:
        if not message.header.frame_id:
            self.get_logger().warn(
                'Received IMU message with empty frame_id'
            )
            return

        acceleration_x = message.linear_acceleration.x
        acceleration_y = message.linear_acceleration.y
        acceleration_z = message.linear_acceleration.z

        gyro_x = message.angular_velocity.x
        gyro_y = message.angular_velocity.y
        gyro_z = message.angular_velocity.z

        acceleration_magnitude = math.sqrt(
            acceleration_x * acceleration_x
            + acceleration_y * acceleration_y
            + acceleration_z * acceleration_z
        )

        gyro_magnitude = math.sqrt(
            gyro_x * gyro_x
            + gyro_y * gyro_y
            + gyro_z * gyro_z
        )

        acceleration_arrow = self.make_arrow(
            message=message,
            marker_id=0,
            namespace='imu_acceleration',
            end_x=(
                acceleration_x
                * self.acceleration_arrow_scale
            ),
            end_y=(
                acceleration_y
                * self.acceleration_arrow_scale
            ),
            end_z=(
                acceleration_z
                * self.acceleration_arrow_scale
            ),
            red=0.1,
            green=1.0,
            blue=0.1,
        )

        gyro_arrow = self.make_arrow(
            message=message,
            marker_id=1,
            namespace='imu_gyro',
            end_x=gyro_x * self.gyro_arrow_scale,
            end_y=gyro_y * self.gyro_arrow_scale,
            end_z=gyro_z * self.gyro_arrow_scale,
            red=0.1,
            green=0.4,
            blue=1.0,
        )

        status_text = self.make_text(
            message,
            acceleration_magnitude,
            gyro_magnitude,
        )

        marker_array = MarkerArray()
        marker_array.markers = [
            acceleration_arrow,
            gyro_arrow,
            status_text,
        ]

        self.marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ImuVisualizer()

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
