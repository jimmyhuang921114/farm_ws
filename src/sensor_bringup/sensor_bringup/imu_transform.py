#!/usr/bin/env python3

import math
import time
from typing import List, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]
Matrix3 = List[List[float]]


def normalize_quaternion(
    quaternion: Quaternion,
) -> Quaternion:
    x, y, z, w = quaternion

    norm = math.sqrt(
        x * x + y * y + z * z + w * w
    )

    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0

    return (
        x / norm,
        y / norm,
        z / norm,
        w / norm,
    )


def inverse_quaternion(
    quaternion: Quaternion,
) -> Quaternion:
    x, y, z, w = normalize_quaternion(
        quaternion
    )

    return -x, -y, -z, w


def multiply_quaternions(
    first: Quaternion,
    second: Quaternion,
) -> Quaternion:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second

    result = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )

    return normalize_quaternion(result)


def quaternion_to_rotation_matrix(
    quaternion: Quaternion,
) -> Matrix3:
    x, y, z, w = normalize_quaternion(
        quaternion
    )

    return [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]


def rotate_vector(
    rotation: Matrix3,
    vector: Vector3,
) -> Vector3:
    x, y, z = vector

    return (
        rotation[0][0] * x
        + rotation[0][1] * y
        + rotation[0][2] * z,

        rotation[1][0] * x
        + rotation[1][1] * y
        + rotation[1][2] * z,

        rotation[2][0] * x
        + rotation[2][1] * y
        + rotation[2][2] * z,
    )


def transpose_matrix(matrix: Matrix3) -> Matrix3:
    return [
        [matrix[column][row] for column in range(3)]
        for row in range(3)
    ]


def multiply_matrices(
    first: Matrix3,
    second: Matrix3,
) -> Matrix3:
    result: Matrix3 = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]

    for row in range(3):
        for column in range(3):
            result[row][column] = sum(
                first[row][index]
                * second[index][column]
                for index in range(3)
            )

    return result


def transform_covariance(
    covariance: Sequence[float],
    rotation: Matrix3,
    scale: float = 1.0,
) -> List[float]:
    if len(covariance) != 9:
        return [0.0] * 9

    if covariance[0] < 0.0:
        return list(covariance)

    covariance_matrix: Matrix3 = [
        [
            float(covariance[0]),
            float(covariance[1]),
            float(covariance[2]),
        ],
        [
            float(covariance[3]),
            float(covariance[4]),
            float(covariance[5]),
        ],
        [
            float(covariance[6]),
            float(covariance[7]),
            float(covariance[8]),
        ],
    ]

    rotated = multiply_matrices(
        multiply_matrices(
            rotation,
            covariance_matrix,
        ),
        transpose_matrix(rotation),
    )

    variance_scale = scale * scale

    return [
        rotated[row][column] * variance_scale
        for row in range(3)
        for column in range(3)
    ]


class ImuTransformNode(Node):

    def __init__(self) -> None:
        super().__init__('imu_transform')

        self.declare_parameter(
            'input_topic',
            '/livox/imu',
        )

        self.declare_parameter(
            'output_topic',
            '/livox/imu_base',
        )

        self.declare_parameter(
            'target_frame',
            'base_link',
        )

        # Empty means using input message frame_id.
        self.declare_parameter(
            'source_frame_override',
            '',
        )

        # Use 9.80665 if input acceleration is measured in g.
        # Use 1.0 if input acceleration is already in m/s^2.
        self.declare_parameter(
            'accel_scale',
            9.80665,
        )

        self.declare_parameter(
            'use_latest_tf',
            True,
        )

        self.declare_parameter(
            'tf_timeout',
            0.1,
        )

        # Livox IMU commonly does not provide orientation.
        self.declare_parameter(
            'force_orientation_unavailable',
            True,
        )

        self.input_topic = str(
            self.get_parameter('input_topic').value
        )

        self.output_topic = str(
            self.get_parameter('output_topic').value
        )

        self.target_frame = str(
            self.get_parameter('target_frame').value
        )

        self.source_frame_override = str(
            self.get_parameter(
                'source_frame_override'
            ).value
        )

        self.accel_scale = float(
            self.get_parameter('accel_scale').value
        )

        self.use_latest_tf = bool(
            self.get_parameter('use_latest_tf').value
        )

        self.tf_timeout = float(
            self.get_parameter('tf_timeout').value
        )

        self.force_orientation_unavailable = bool(
            self.get_parameter(
                'force_orientation_unavailable'
            ).value
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.last_warning_time = 0.0

        self.get_logger().info(
            f'IMU input topic: {self.input_topic}'
        )

        self.get_logger().info(
            f'IMU output topic: {self.output_topic}'
        )

        self.get_logger().info(
            f'Target frame: {self.target_frame}'
        )

        self.get_logger().info(
            f'Acceleration scale: {self.accel_scale}'
        )

    def log_warning_throttled(
        self,
        message: str,
        interval: float = 2.0,
    ) -> None:
        current_time = time.monotonic()

        if current_time - self.last_warning_time >= interval:
            self.get_logger().warn(message)
            self.last_warning_time = current_time

    def get_rotation(
        self,
        source_frame: str,
        message: Imu,
    ) -> Tuple[Quaternion, Matrix3]:
        if source_frame == self.target_frame:
            quaternion: Quaternion = (
                0.0,
                0.0,
                0.0,
                1.0,
            )

            return (
                quaternion,
                quaternion_to_rotation_matrix(
                    quaternion
                ),
            )

        if self.use_latest_tf:
            lookup_time = Time()
        else:
            lookup_time = Time.from_msg(
                message.header.stamp
            )

        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            lookup_time,
            timeout=Duration(
                seconds=self.tf_timeout
            ),
        )

        quaternion: Quaternion = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )

        quaternion = normalize_quaternion(
            quaternion
        )

        return (
            quaternion,
            quaternion_to_rotation_matrix(
                quaternion
            ),
        )

    def imu_callback(self, message: Imu) -> None:
        source_frame = (
            self.source_frame_override
            if self.source_frame_override
            else message.header.frame_id
        )

        if not source_frame:
            self.log_warning_throttled(
                'Input IMU frame_id is empty'
            )
            return

        try:
            transform_quaternion, rotation = (
                self.get_rotation(
                    source_frame,
                    message,
                )
            )

        except Exception as error:
            self.log_warning_throttled(
                f'Cannot transform IMU from '
                f'{source_frame} to '
                f'{self.target_frame}: {error}'
            )
            return

        source_angular_velocity: Vector3 = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )

        source_linear_acceleration: Vector3 = (
            message.linear_acceleration.x
            * self.accel_scale,

            message.linear_acceleration.y
            * self.accel_scale,

            message.linear_acceleration.z
            * self.accel_scale,
        )

        target_angular_velocity = rotate_vector(
            rotation,
            source_angular_velocity,
        )

        target_linear_acceleration = rotate_vector(
            rotation,
            source_linear_acceleration,
        )

        output = Imu()

        output.header.stamp = message.header.stamp
        output.header.frame_id = self.target_frame

        output.angular_velocity.x = (
            target_angular_velocity[0]
        )
        output.angular_velocity.y = (
            target_angular_velocity[1]
        )
        output.angular_velocity.z = (
            target_angular_velocity[2]
        )

        output.linear_acceleration.x = (
            target_linear_acceleration[0]
        )
        output.linear_acceleration.y = (
            target_linear_acceleration[1]
        )
        output.linear_acceleration.z = (
            target_linear_acceleration[2]
        )

        output.angular_velocity_covariance = (
            transform_covariance(
                message.angular_velocity_covariance,
                rotation,
                scale=1.0,
            )
        )

        output.linear_acceleration_covariance = (
            transform_covariance(
                message.linear_acceleration_covariance,
                rotation,
                scale=self.accel_scale,
            )
        )

        orientation_is_unavailable = (
            self.force_orientation_unavailable
            or message.orientation_covariance[0] < 0.0
        )

        if orientation_is_unavailable:
            output.orientation.x = 0.0
            output.orientation.y = 0.0
            output.orientation.z = 0.0
            output.orientation.w = 1.0

            output.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
            ]

        else:
            input_orientation: Quaternion = (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            )

            # Change the sensor coordinate basis:
            #
            # q_target =
            #     q_source * inverse(q_target_source)
            output_orientation = multiply_quaternions(
                normalize_quaternion(
                    input_orientation
                ),
                inverse_quaternion(
                    transform_quaternion
                ),
            )

            output.orientation.x = (
                output_orientation[0]
            )
            output.orientation.y = (
                output_orientation[1]
            )
            output.orientation.z = (
                output_orientation[2]
            )
            output.orientation.w = (
                output_orientation[3]
            )

            output.orientation_covariance = (
                transform_covariance(
                    message.orientation_covariance,
                    rotation,
                    scale=1.0,
                )
            )

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ImuTransformNode()

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
