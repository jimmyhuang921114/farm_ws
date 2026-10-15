#!/usr/bin/env python3

import cv2
import math
import glob
import os
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32
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
CAMERA_FPS = 60

CONF_THRESHOLD = 0.4
PERSON_CLASS_ID = 0

# pixel deadband
DEADBAND = 20

# 拿掉 ByteTrack 後，用上一幀目標中心做簡單最近鄰匹配。
# 如果下一幀最近的人距離上一幀位置超過這個值，
# 就視為目前找不到原本目標。
TARGET_MATCH_MAX_DISTANCE = 250.0

# ============================================================
# Target center low-pass filter
#
# 1.0 = 不濾波，完全使用最新 YOLO 結果
# 越小 = 越平滑，但反應越慢
# 建議先從 0.3 開始測試
# ============================================================

TARGET_LPF_ALPHA = 0.8


# ============================================================
# Pixel error -> angle calibration
#
# Based on current measured data:
#   0 px   ->  0 deg
#   68 px  -> 10 deg
#   158 px -> 20 deg
#   381 px -> 40 deg
#
# Left/right are assumed symmetric.
# Positive error_x -> positive angle
# Negative error_x -> negative angle
# ============================================================

ANGLE_CALIBRATION = [
    (0.0,   0.0),
    (68.0, 10.0),
    (158.0, 20.0),
    (381.0, 40.0),
]


def pixel_error_to_angle(error_x):
    """Convert horizontal pixel error to angle in degrees."""

    if error_x == 0:
        return 0.0

    sign = 1.0 if error_x > 0 else -1.0
    px = abs(float(error_x))

    # Interpolate inside the measured range.
    for i in range(len(ANGLE_CALIBRATION) - 1):

        px0, deg0 = ANGLE_CALIBRATION[i]
        px1, deg1 = ANGLE_CALIBRATION[i + 1]

        if px <= px1:

            ratio = (
                (px - px0)
                /
                (px1 - px0)
            )

            angle = (
                deg0
                +
                ratio * (deg1 - deg0)
            )

            return sign * angle

    # Extrapolate beyond the largest measured point
    # using the slope of the final calibration segment.
    px0, deg0 = ANGLE_CALIBRATION[-2]
    px1, deg1 = ANGLE_CALIBRATION[-1]

    slope_deg_per_px = (
        (deg1 - deg0)
        /
        (px1 - px0)
    )

    angle = (
        deg1
        +
        (px - px1) * slope_deg_per_px
    )

    return sign * angle


# ============================================================
# Camera functions
# ============================================================

def get_usb_vid_pid(video_device):

    video_name = os.path.basename(
        video_device
    )

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
            and
            os.path.exists(product_file)
        ):

            try:

                with open(
                    vendor_file,
                    "r"
                ) as f:

                    vendor = (
                        f.read()
                        .strip()
                        .lower()
                    )

                with open(
                    product_file,
                    "r"
                ) as f:

                    product = (
                        f.read()
                        .strip()
                        .lower()
                    )

                return vendor, product

            except OSError:
                pass

        current = os.path.dirname(
            current
        )

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

            matching_devices.append(
                device
            )

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


        # ============================================================
        # Camera format
        # ============================================================
        # 使用 MJPG。
        # 此相機在 1280x720：
        #
        #   YUYV -> 最高 10 FPS
        #   MJPG -> 最高 60 FPS
        #
        # 因此必須先指定 MJPG，再設定解析度與 FPS。
        # ============================================================

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                "M",
                "J",
                "P",
                "G"
            )
        )


        # ============================================================
        # Resolution
        # ============================================================

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            CAMERA_WIDTH
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            CAMERA_HEIGHT
        )


        # ============================================================
        # FPS
        # ============================================================

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

        self.error_angle_pub = (
            self.create_publisher(
                Float32,
                "/person_tracker/error_angle",
                10
            )
        )

        # 為了保留原本 topic 相容性，仍保留 target_id。
        # 因為現在沒有 ByteTrack：
        #   0  = 有鎖定目標
        #  -1  = 沒有鎖定目標
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
        # Camera actual settings
        # ====================================================

        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))

        fourcc = "".join([
            chr(actual_fourcc & 0xFF),
            chr((actual_fourcc >> 8) & 0xFF),
            chr((actual_fourcc >> 16) & 0xFF),
            chr((actual_fourcc >> 24) & 0xFF),
        ])

        self.get_logger().info(
            f"Actual camera settings: "
            f"{actual_width:.0f}x{actual_height:.0f} "
            f"@ {actual_fps:.1f} FPS "
            f"| FOURCC={fourcc}"
        )

        # 診斷資訊每 1 秒印一次
        self.last_diagnostic_time = time.perf_counter()
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
        #
        # ByteTrack 已移除。
        # 目前改用：
        #   SPACE 鎖定離畫面中心最近的人
        #   後續用「離上一幀目標位置最近的人」做簡單匹配
        # ====================================================

        self.target_locked = False

        # 上一幀鎖定人物中心 (cx, cy)
        self.locked_target_center = None

        # ====================================================
        # Target center LPF state
        # ====================================================

        # 儲存上一幀經過低通濾波後的人物 X 座標
        self.filtered_target_x = None

        # ====================================================
        # FPS state
        # ====================================================

        self.last_frame_time = (
            time.perf_counter()
        )

        self.current_fps = 0.0

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
            "YOLO Person Tracker "
            "(ByteTrack OFF)"
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
        target_present,
        locked
    ):

        # error_x
        error_msg = Int32()
        error_msg.data = int(
            error_x
        )

        self.error_x_pub.publish(
            error_msg
        )

        # error_angle
        angle_msg = Float32()
        angle_msg.data = float(
            pixel_error_to_angle(
                error_x
            )
        )

        self.error_angle_pub.publish(
            angle_msg
        )

        # target_id
        #
        # ByteTrack 拿掉後沒有真正 ID。
        # 保留這個 topic 只是為了不破壞其他程式：
        # 0  = target exists
        # -1 = no target
        id_msg = Int32()

        if target_present:
            id_msg.data = 0
        else:
            id_msg.data = -1

        self.target_id_pub.publish(
            id_msg
        )

        # locked
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

        # ====================================================
        # Total callback timer
        # ====================================================

        callback_start = time.perf_counter()

        # ====================================================
        # Camera read timer
        # ====================================================

        camera_start = time.perf_counter()

        ret, frame = self.cap.read()

        camera_end = time.perf_counter()

        camera_ms = (
            camera_end - camera_start
        ) * 1000.0

        if not ret:

            self.get_logger().error(
                "Failed to read camera frame."
            )

            return
        # ====================================================
        # Actual FPS
        # ====================================================

        now = time.perf_counter()

        frame_dt = (
            now - self.last_frame_time
        )

        self.last_frame_time = now

        if frame_dt > 0.0:

            instant_fps = (
                1.0 / frame_dt
            )

            if self.current_fps == 0.0:

                self.current_fps = (
                    instant_fps
                )

            else:

                self.current_fps = (
                    0.9 * self.current_fps
                    +
                    0.1 * instant_fps
                )

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
        # YOLO detection
        #
        # ByteTrack 已完全移除。
        # 原本：
        #
        # self.model.track(...)
        #
        # 現在：
        #
        # self.model.predict(...)
        # ====================================================

        yolo_start = time.perf_counter()

        results = self.model.predict(
            frame,
            classes=[
                PERSON_CLASS_ID
            ],
            conf=CONF_THRESHOLD,
            device=0,
            verbose=False
        )

        yolo_end = time.perf_counter()

        yolo_ms = (
            yolo_end - yolo_start
        ) * 1000.0

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

            for box in results[0].boxes:

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
                            -
                            camera_center_x
                        ) ** 2
                        +
                        (
                            person_center_y
                            -
                            camera_center_y
                        ) ** 2
                    )
                )

                persons.append({
                    "bbox": (
                        x1,
                        y1,
                        x2,
                        y2
                    ),
                    "cx":
                        person_center_x,
                    "cy":
                        person_center_y,
                    "conf":
                        confidence,
                    "distance":
                        distance_to_center,
                })

        # ====================================================
        # Find current target without ByteTrack
        # ====================================================

        target_person = None
        target_found = False

        if (
            self.target_locked
            and
            self.locked_target_center
            is not None
            and
            persons
        ):

            last_x, last_y = (
                self.locked_target_center
            )

            # 找離上一幀鎖定位置最近的人
            target_person = min(
                persons,
                key=lambda p:
                    math.sqrt(
                        (
                            p["cx"] - last_x
                        ) ** 2
                        +
                        (
                            p["cy"] - last_y
                        ) ** 2
                    )
            )

            match_distance = math.sqrt(
                (
                    target_person["cx"]
                    - last_x
                ) ** 2
                +
                (
                    target_person["cy"]
                    - last_y
                ) ** 2
            )

            # 距離太遠就先視為目標遺失
            if (
                match_distance
                <=
                TARGET_MATCH_MAX_DISTANCE
            ):

                target_found = True

                # 更新上一幀目標位置
                self.locked_target_center = (
                    target_person["cx"],
                    target_person["cy"]
                )

            else:

                target_person = None

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

        output_error_x = 0
        output_error_angle = 0.0

        # ====================================================
        # Draw people
        # ====================================================

        for person in persons:

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

            is_target = (
                target_found
                and
                person is target_person
            )

            if is_target:

                color = (
                    0,
                    0,
                    255
                )

                # =============================================
                # Target center low-pass filter
                # =============================================

                if self.filtered_target_x is None:

                    # 第一次取得目標時直接使用目前位置，
                    # 避免濾波器從 0 開始造成額外延遲。
                    self.filtered_target_x = float(
                        person_center_x
                    )

                else:

                    self.filtered_target_x = (
                        TARGET_LPF_ALPHA
                        *
                        float(person_center_x)
                        +
                        (
                            1.0
                            -
                            TARGET_LPF_ALPHA
                        )
                        *
                        self.filtered_target_x
                    )

                # =============================================
                # Pixel error (use filtered target center)
                # =============================================

                pixel_error_x = int(
                    round(
                        self.filtered_target_x
                        -
                        camera_center_x
                    )
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
                # Pixel error -> angle
                # =============================================

                output_error_angle = (
                    pixel_error_to_angle(
                        output_error_x
                    )
                )

                # =============================================
                # ROS publish
                # =============================================

                self.publish_status(
                    output_error_x,
                    True,
                    True
                )

                print(
                    f"\r"
                    f"TARGET"
                    f" | "
                    f"RawX="
                    f"{person_center_x:4d}"
                    f" | "
                    f"FilterX="
                    f"{self.filtered_target_x:7.2f}"
                    f" | "
                    f"center_x="
                    f"{camera_center_x:4d}"
                    f" | "
                    f"error_x="
                    f"{output_error_x:+5d}"
                    f" | "
                    f"angle="
                    f"{output_error_angle:+6.2f} deg"
                    f" | "
                    f"FPS="
                    f"{self.current_fps:5.1f}",
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
                        int(round(self.filtered_target_x)),
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

                cv2.putText(
                    frame,
                    f"ANGLE: "
                    f"{output_error_angle:+.2f} deg",
                    (
                        20,
                        90
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

            label = (
                f"TARGET "
                if is_target
                else
                f"PERSON "
            )

            cv2.putText(
                frame,
                f"{label}"
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
        # Publish unlocked / lost state
        # ====================================================

        if not self.target_locked:

            self.publish_status(
                0,
                False,
                False
            )

        elif not target_found:

            self.publish_status(
                0,
                True,
                False
            )

        # ====================================================
        # FPS text
        # ====================================================

        cv2.putText(
            frame,
            f"FPS: {self.current_fps:.1f}",
            (
                width - 180,
                40
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (
                255,
                255,
                255
            ),
            2
        )

        # ====================================================
        # Status text
        # ====================================================

        if not self.target_locked:

            status_text = (
                "SEARCHING - "
                "PRESS SPACE TO LOCK"
            )

        elif target_found:

            status_text = (
                "TARGET LOCKED "
                "(NO BYTETRACK)"
            )

        else:

            status_text = (
                "TARGET LOST"
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
            "YOLO Person Tracker - No ByteTrack",
            frame
        )

        key = (
            cv2.waitKey(1)
            &
            0xFF
        )
        # ====================================================
        # Performance diagnostics
        # ====================================================

        callback_end = time.perf_counter()

        total_ms = (
            callback_end - callback_start
        ) * 1000.0

        diagnostic_now = time.perf_counter()

        if (
            diagnostic_now
            - self.last_diagnostic_time
            >= 1.0
        ):

            self.last_diagnostic_time = diagnostic_now

            theoretical_fps = (
                1000.0 / total_ms
                if total_ms > 0.0
                else 0.0
            )

            print(
                "\n"
                "========== PERFORMANCE ==========\n"
                f"Camera read : {camera_ms:6.1f} ms\n"
                f"YOLO        : {yolo_ms:6.1f} ms\n"
                f"Total       : {total_ms:6.1f} ms\n"
                f"Actual FPS  : {self.current_fps:6.1f}\n"
                f"Max possible: {theoretical_fps:6.1f} FPS\n"
                "================================="
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

                # 新目標重新初始化 LPF，避免沿用上一個人的位置
                self.filtered_target_x = None

                self.target_locked = True

                self.locked_target_center = (
                    nearest_person["cx"],
                    nearest_person["cy"]
                )

                self.get_logger().info(
                    "Target locked "
                    "(nearest person to image center)."
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

            self.target_locked = False
            self.locked_target_center = None
            self.filtered_target_x = None

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

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()