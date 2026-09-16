#!/usr/bin/env python3

import cv2
import math
import glob
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Bool

from ultralytics import YOLO


# ============================================================
# Configuration
# ============================================================

USB_VENDOR_ID = "1bcf"
USB_PRODUCT_ID = "2cd1"

MODEL_PATH = "yolo11m.pt"

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

CONF_THRESHOLD = 0.4
PERSON_CLASS_ID = 0

DEADBAND = 10


# ============================================================
# Camera functions
# ============================================================

def get_usb_vid_pid(video_device):

    video_name = os.path.basename(video_device)

    sys_path = os.path.realpath(
        f"/sys/class/video4linux/{video_name}/device"
    )

    current = sys_path

    while current != "/":

        vendor_file = os.path.join(
            current,
            "idVendor"
        )

        product_file = os.path.join(
            current,
            "idProduct"
        )

        if (
            os.path.exists(vendor_file)
            and os.path.exists(product_file)
        ):

            try:

                with open(vendor_file, "r") as f:
                    vendor = (
                        f.read()
                        .strip()
                        .lower()
                    )

                with open(product_file, "r") as f:
                    product = (
                        f.read()
                        .strip()
                        .lower()
                    )

                return vendor, product

            except OSError:
                pass

        current = os.path.dirname(current)

    return None, None


def find_camera_by_usb_id(
    vendor_id,
    product_id
):

    print(
        f"[INFO] Searching camera USB "
        f"{vendor_id}:{product_id}"
    )

    matching_devices = []

    for device in sorted(
        glob.glob("/dev/video*")
    ):

        vendor, product = get_usb_vid_pid(
            device
        )

        if (
            vendor == vendor_id.lower()
            and
            product == product_id.lower()
        ):

            print(
                f"[INFO] Found matching device: "
                f"{device} "
                f"({vendor}:{product})"
            )

            matching_devices.append(device)


    if not matching_devices:
        return None, None


    for device in matching_devices:

        print(
            f"[INFO] Testing {device}..."
        )

        cap = cv2.VideoCapture(
            device,
            cv2.CAP_V4L2
        )

        if not cap.isOpened():

            print(
                f"[WARN] Cannot open {device}"
            )

            cap.release()
            continue


        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            CAMERA_WIDTH
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            CAMERA_HEIGHT
        )

        cap.set(
            cv2.CAP_PROP_FPS,
            CAMERA_FPS
        )


        frame_ok = False

        for _ in range(10):

            ret, frame = cap.read()

            if (
                ret
                and
                frame is not None
            ):

                frame_ok = True
                break


        if frame_ok:

            print(
                f"[INFO] Using camera: "
                f"{device}"
            )

            return cap, device


        print(
            f"[WARN] {device} "
            f"does not provide valid frames"
        )

        cap.release()


    return None, None


# ============================================================
# ROS2 Node
# ============================================================

class PersonTrackerNode(Node):

    def __init__(self):

        super().__init__(
            "person_tracker"
        )


        # ====================================================
        # Publishers
        # ====================================================

        self.error_x_pub = (
            self.create_publisher(
                Int32,
                "/person_tracker/error_x",
                10
            )
        )

        self.target_id_pub = (
            self.create_publisher(
                Int32,
                "/person_tracker/target_id",
                10
            )
        )

        self.locked_pub = (
            self.create_publisher(
                Bool,
                "/person_tracker/locked",
                10
            )
        )


        # ====================================================
        # Camera
        # ====================================================

        self.cap, self.camera_device = (
            find_camera_by_usb_id(
                USB_VENDOR_ID,
                USB_PRODUCT_ID
            )
        )

        if self.cap is None:

            self.get_logger().error(
                f"Cannot find working camera "
                f"{USB_VENDOR_ID}:"
                f"{USB_PRODUCT_ID}"
            )

            raise RuntimeError(
                "Camera not found"
            )


        self.get_logger().info(
            f"Camera opened: "
            f"{self.camera_device}"
        )


        # ====================================================
        # YOLO
        # ====================================================

        self.get_logger().info(
            "Loading YOLO..."
        )

        self.model = YOLO(
            MODEL_PATH
        )

        self.get_logger().info(
            "YOLO loaded."
        )


        # ====================================================
        # Tracking state
        # ====================================================

        self.locked_track_id = None


        # ====================================================
        # Timer
        # ====================================================

        self.timer = self.create_timer(
            1.0 / CAMERA_FPS,
            self.timer_callback
        )


        self.get_logger().info(
            "================================"
        )

        self.get_logger().info(
            "YOLO Person Tracker"
        )

        self.get_logger().info(
            "SPACE : lock nearest person"
        )

        self.get_logger().info(
            "R     : release target"
        )

        self.get_logger().info(
            "ESC   : exit"
        )

        self.get_logger().info(
            "================================"
        )


    # ========================================================
    # Publish helper
    # ========================================================

    def publish_status(
        self,
        error_x,
        target_id,
        locked
    ):

        error_msg = Int32()
        error_msg.data = int(error_x)

        self.error_x_pub.publish(
            error_msg
        )


        id_msg = Int32()

        if target_id is None:
            id_msg.data = -1
        else:
            id_msg.data = int(
                target_id
            )

        self.target_id_pub.publish(
            id_msg
        )


        locked_msg = Bool()
        locked_msg.data = bool(
            locked
        )

        self.locked_pub.publish(
            locked_msg
        )


    # ========================================================
    # Main loop
    # ========================================================

    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:

            self.get_logger().error(
                "Failed to read camera frame."
            )

            return


        # ====================================================
        # Image center
        # ====================================================

        height, width = frame.shape[:2]

        camera_center_x = (
            width // 2
        )

        camera_center_y = (
            height // 2
        )


        # ====================================================
        # YOLO + ByteTrack
        # ====================================================

        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            classes=[
                PERSON_CLASS_ID
            ],
            conf=CONF_THRESHOLD,
            verbose=False
        )


        persons = []


        # ====================================================
        # Read detections
        # ====================================================

        if (
            results
            and
            results[0].boxes
            is not None
        ):

            for box in (
                results[0].boxes
            ):

                if box.id is None:
                    continue


                track_id = int(
                    box.id.item()
                )


                x1, y1, x2, y2 = map(
                    int,
                    box.xyxy[0].tolist()
                )


                confidence = float(
                    box.conf.item()
                )


                person_center_x = (
                    x1 + x2
                ) // 2


                person_center_y = (
                    y1 + y2
                ) // 2


                distance_to_center = (
                    math.sqrt(
                        (
                            person_center_x
                            - camera_center_x
                        ) ** 2
                        +
                        (
                            person_center_y
                            - camera_center_y
                        ) ** 2
                    )
                )


                persons.append({
                    "id": track_id,
                    "bbox": (
                        x1,
                        y1,
                        x2,
                        y2
                    ),
                    "cx": person_center_x,
                    "cy": person_center_y,
                    "conf": confidence,
                    "distance":
                        distance_to_center
                })


        # ====================================================
        # Camera center
        # ====================================================

        cv2.drawMarker(
            frame,
            (
                camera_center_x,
                camera_center_y
            ),
            (
                255,
                255,
                255
            ),
            cv2.MARKER_CROSS,
            30,
            2
        )


        target_found = False
        output_error_x = 0


        # ====================================================
        # Draw people
        # ====================================================

        for person in persons:

            track_id = person["id"]

            x1, y1, x2, y2 = (
                person["bbox"]
            )

            person_center_x = (
                person["cx"]
            )

            person_center_y = (
                person["cy"]
            )

            confidence = (
                person["conf"]
            )


            if (
                track_id
                ==
                self.locked_track_id
            ):

                target_found = True

                color = (
                    0,
                    0,
                    255
                )


                # =============================================
                # Pixel error
                # =============================================

                pixel_error_x = (
                    person_center_x
                    -
                    camera_center_x
                )


                if (
                    abs(pixel_error_x)
                    <
                    DEADBAND
                ):

                    output_error_x = 0

                else:

                    output_error_x = (
                        pixel_error_x
                    )


                # =============================================
                # ROS publish
                # =============================================

                self.publish_status(
                    output_error_x,
                    track_id,
                    True
                )


                print(
                    f"\r"
                    f"TRACK_ID="
                    f"{track_id:3d}"
                    f" | "
                    f"target_x="
                    f"{person_center_x:4d}"
                    f" | "
                    f"center_x="
                    f"{camera_center_x:4d}"
                    f" | "
                    f"error_x="
                    f"{output_error_x:+5d}",
                    end="",
                    flush=True
                )


                cv2.line(
                    frame,
                    (
                        camera_center_x,
                        camera_center_y
                    ),
                    (
                        person_center_x,
                        person_center_y
                    ),
                    color,
                    2
                )


                cv2.putText(
                    frame,
                    f"ERROR X: "
                    f"{output_error_x:+d} px",
                    (
                        20,
                        50
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    color,
                    2
                )

            else:

                color = (
                    0,
                    255,
                    0
                )


            cv2.rectangle(
                frame,
                (
                    x1,
                    y1
                ),
                (
                    x2,
                    y2
                ),
                color,
                2
            )


            cv2.circle(
                frame,
                (
                    person_center_x,
                    person_center_y
                ),
                5,
                color,
                -1
            )


            cv2.putText(
                frame,
                f"ID {track_id} "
                f"{confidence:.2f}",
                (
                    x1,
                    max(
                        y1 - 10,
                        20
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2
            )


        # ====================================================
        # Publish unlocked/lost state
        # ====================================================

        if self.locked_track_id is None:

            self.publish_status(
                0,
                None,
                False
            )

        elif not target_found:

            self.publish_status(
                0,
                self.locked_track_id,
                False
            )


        # ====================================================
        # Status text
        # ====================================================

        if self.locked_track_id is None:

            status_text = (
                "SEARCHING - "
                "PRESS SPACE TO LOCK"
            )

        elif target_found:

            status_text = (
                f"LOCKED ID: "
                f"{self.locked_track_id}"
            )

        else:

            status_text = (
                f"TARGET LOST: "
                f"{self.locked_track_id}"
            )


        cv2.putText(
            frame,
            status_text,
            (
                20,
                height - 30
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (
                255,
                255,
                255
            ),
            2
        )


        # ====================================================
        # Show image
        # ====================================================

        cv2.imshow(
            "YOLO Person Tracker",
            frame
        )


        key = (
            cv2.waitKey(1)
            & 0xFF
        )


        # ====================================================
        # SPACE
        # ====================================================

        if key == ord(" "):

            if not persons:

                self.get_logger().warn(
                    "No person detected."
                )

            else:

                nearest_person = min(
                    persons,
                    key=lambda p:
                        p["distance"]
                )


                self.locked_track_id = (
                    nearest_person["id"]
                )


                self.get_logger().info(
                    f"Locked target ID: "
                    f"{self.locked_track_id}"
                )


        # ====================================================
        # R
        # ====================================================

        elif key in (
            ord("r"),
            ord("R")
        ):

            self.get_logger().info(
                "Target released."
            )

            self.locked_track_id = None


        # ====================================================
        # ESC
        # ====================================================

        elif key == 27:

            self.get_logger().info(
                "Exit."
            )

            rclpy.shutdown()


    # ========================================================
    # Cleanup
    # ========================================================

    def destroy_node(self):

        if hasattr(
            self,
            "cap"
        ):

            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = PersonTrackerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

