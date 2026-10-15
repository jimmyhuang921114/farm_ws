
#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class ObjectController(Node):

    def __init__(self):
        super().__init__("object_controller")

        # Parameters
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("kp_linear", 0.4)
        self.declare_parameter("kp_angular", 1.5)
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("turn_threshold", 0.35)
        self.declare_parameter("target_timeout", 0.5)

        self.stop_distance = self.get_parameter(
            "stop_distance").value
        self.kp_linear = self.get_parameter(
            "kp_linear").value
        self.kp_angular = self.get_parameter(
            "kp_angular").value
        self.max_linear = self.get_parameter(
            "max_linear").value
        self.max_angular = self.get_parameter(
            "max_angular").value
        self.turn_threshold = self.get_parameter(
            "turn_threshold").value
        self.target_timeout = self.get_parameter(
            "target_timeout").value

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self)

        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/person/pose",
            self.pose_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Target state
        self.target_distance = 0.0
        self.target_angle = 0.0
        self.target_valid = False
        self.last_target_time = 0.0

        # 20 Hz control loop
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            "Object Controller started"
        )

    def pose_callback(self, msg):

        if not msg.header.frame_id:
            self.target_valid = False
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                msg.header.frame_id,
                rclpy.time.Time.from_msg(
                    msg.header.stamp
                ),
                timeout=Duration(seconds=0.1)
            )

            pose = do_transform_pose_stamped(
                msg, transform
            )

            x = pose.pose.position.x
            y = pose.pose.position.y

            if not (
                math.isfinite(x)
                and math.isfinite(y)
            ):
                self.target_valid = False
                return

            # ROS base_link:
            # X = forward
            # Y = left
            self.target_distance = math.hypot(x, y)
            self.target_angle = math.atan2(y, x)

            self.last_target_time = time.monotonic()
            self.target_valid = True

        except TransformException as ex:
            self.get_logger().warning(
                f"TF failed: {ex}"
            )
            self.target_valid = False

    def control_loop(self):

        cmd = Twist()

        # Invalid target -> stop
        if not self.target_valid:
            self.cmd_pub.publish(cmd)
            return

        elapsed = (
            time.monotonic()
            - self.last_target_time
        )

        if elapsed > self.target_timeout:
            self.target_valid = False
            self.cmd_pub.publish(cmd)
            return

        distance_error = (
            self.target_distance
            - self.stop_distance
        )

        # Stop at target distance
        if distance_error <= 0.0:
            self.cmd_pub.publish(cmd)
            return

        # Angular P Controller
        cmd.angular.z = max(
            -self.max_angular,
            min(
                self.max_angular,
                self.kp_angular
                * self.target_angle
            )
        )

        # Rotate first
        if abs(self.target_angle) > self.turn_threshold:
            cmd.linear.x = 0.0

        else:
            # Linear P Controller
            cmd.linear.x = max(
                0.0,
                min(
                    self.max_linear,
                    self.kp_linear
                    * distance_error
                )
            )

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)

    node = ObjectController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
