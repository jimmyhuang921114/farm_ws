import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# camera_main.py、camera.py、depth_estimate.py 位於同一個 Python package
from .camera import Camera
from .depth_estimate import DepthEstimate


class CameraMainNode(Node):
    def __init__(self):
        super().__init__("camera_main")

        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "camera_link")

        self.fps = float(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.bridge = CvBridge()

        # 啟動相機與深度模型
        self.camera = Camera()
        self.camera.open()

        self.depth_estimator = DepthEstimate()

        # 發布給其他 ROS 2 節點，例如 C++ nvblox
        self.rgb_pub = self.create_publisher(
            Image,
            "/camera/color/image_raw",
            qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image,
            "/camera/depth/image_estimated",
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / self.fps, self.process_frame)
        self.get_logger().info("Camera and depth estimator started")

    def process_frame(self):
        try:
            # 預期 camera.read() 回傳 OpenCV BGR 影像
            image_bgr = self.camera.read()
            if image_bgr is None:
                self.get_logger().warning("No camera frame received")
                return

            # 同一張 RGB/BGR 影像直接交給 Python 深度模型
            depth_m = self.depth_estimator.estimate(image_bgr)

            depth_m = np.asarray(depth_m, dtype=np.float32)
            if depth_m.ndim == 3:
                depth_m = depth_m.squeeze()

            if depth_m.ndim != 2:
                self.get_logger().error(
                    f"Expected HxW depth image, got shape {depth_m.shape}"
                )
                return

            # 同一組時間戳，讓下游節點能配對 RGB 和 depth
            stamp = self.get_clock().now().to_msg()

            rgb_msg = self.bridge.cv2_to_imgmsg(
                image_bgr,
                encoding="bgr8",
            )
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self.frame_id

            depth_msg = self.bridge.cv2_to_imgmsg(
                depth_m,
                encoding="32FC1",
            )
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id

            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)

        except Exception as exc:
            self.get_logger().error(f"Frame processing failed: {exc}")

    def destroy_node(self):
        if hasattr(self, "camera"):
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraMainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()