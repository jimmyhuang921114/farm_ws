#!/usr/bin/env python3

import cv2
import math
import glob
import os
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
# Find camera by USB VID:PID
# ============================================================

def get_usb_vid_pid(video_device):
    """
    Find USB VID/PID for /dev/videoX through sysfs.
    """

    video_name = os.path.basename(video_device)

    sys_path = os.path.realpath(
        f"/sys/class/video4linux/{video_name}/device"
    )

    current = sys_path

    while current != "/":

        vendor_file = os.path.join(current, "idVendor")
        product_file = os.path.join(current, "idProduct")

        if os.path.exists(vendor_file) and os.path.exists(product_file):

            try:
                with open(vendor_file, "r") as f:
                    vendor = f.read().strip().lower()

                with open(product_file, "r") as f:
                    product = f.read().strip().lower()

                return vendor, product

            except OSError:
                pass

        current = os.path.dirname(current)

    return None, None


def find_camera_by_usb_id(vendor_id, product_id):

    print(
        f"[INFO] Searching camera USB "
        f"{vendor_id}:{product_id}"
    )

    matching_devices = []

    for device in sorted(glob.glob("/dev/video*")):

        vendor, product = get_usb_vid_pid(device)

        if vendor == vendor_id.lower() and product == product_id.lower():

            print(
                f"[INFO] Found matching device: "
                f"{device} ({vendor}:{product})"
            )

            matching_devices.append(device)


    if not matching_devices:

        return None, None


    # --------------------------------------------------------
    # Test which node can actually capture frames
    # --------------------------------------------------------

    for device in matching_devices:

        print(f"[INFO] Testing {device}...")

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


        # Try several frames because some USB cameras
        # need a short warm-up period.
        frame_ok = False

        for _ in range(10):

            ret, frame = cap.read()

            if ret and frame is not None:
                frame_ok = True
                break


        if frame_ok:

            print(
                f"[INFO] Using camera: {device}"
            )

            return cap, device


        print(
            f"[WARN] {device} does not provide valid frames"
        )

        cap.release()


    return None, None


# ============================================================
# Open Camera
# ============================================================

cap, CAMERA_DEVICE = find_camera_by_usb_id(
    USB_VENDOR_ID,
    USB_PRODUCT_ID
)


if cap is None:

    print(
        f"[ERROR] Cannot find working camera "
        f"{USB_VENDOR_ID}:{USB_PRODUCT_ID}"
    )

    raise SystemExit(1)


print(
    f"[INFO] Camera opened: {CAMERA_DEVICE}"
)


# ============================================================
# Load YOLO
# ============================================================

print("[INFO] Loading YOLO...")

model = YOLO(MODEL_PATH)

print("[INFO] YOLO loaded.")


# ============================================================
# Tracking state
# ============================================================

locked_track_id = None


print("============================================================")
print("YOLO Person Tracker")
print("============================================================")
print(f"Camera: {CAMERA_DEVICE}")
print(f"USB ID: {USB_VENDOR_ID}:{USB_PRODUCT_ID}")
print("SPACE : lock nearest person")
print("R     : release target")
print("ESC   : exit")
print("============================================================")


while True:

    ret, frame = cap.read()

    if not ret:

        print(
            "\n[ERROR] Failed to read camera frame."
        )

        break


    # ========================================================
    # Image center
    # ========================================================

    height, width = frame.shape[:2]

    camera_center_x = width // 2
    camera_center_y = height // 2


    # ========================================================
    # YOLO + ByteTrack
    # ========================================================

    results = model.track(
        frame,
        persist=True,
        tracker="bytetrack.yaml",
        classes=[PERSON_CLASS_ID],
        conf=CONF_THRESHOLD,
        verbose=False
    )


    persons = []


    # ========================================================
    # Read detections
    # ========================================================

    if results and results[0].boxes is not None:

        for box in results[0].boxes:

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


            distance_to_center = math.sqrt(
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
                "distance": distance_to_center
            })


    # ========================================================
    # Draw camera center
    # ========================================================

    cv2.drawMarker(
        frame,
        (
            camera_center_x,
            camera_center_y
        ),
        (255, 255, 255),
        cv2.MARKER_CROSS,
        30,
        2
    )


    target_found = False


    # ========================================================
    # Draw people
    # ========================================================

    for person in persons:

        track_id = person["id"]

        x1, y1, x2, y2 = person["bbox"]

        person_center_x = person["cx"]
        person_center_y = person["cy"]

        confidence = person["conf"]


        if track_id == locked_track_id:

            target_found = True

            color = (0, 0, 255)


            # =================================================
            # Pixel error
            # =================================================

            pixel_error_x = (
                person_center_x
                - camera_center_x
            )

            pixel_error_y = (
                person_center_y
                - camera_center_y
            )


            if abs(pixel_error_x) < DEADBAND:
                output_error_x = 0
            else:
                output_error_x = pixel_error_x


            # =================================================
            # Output error
            # =================================================

            print(
                f"\r"
                f"TRACK_ID={track_id:3d} | "
                f"target_x={person_center_x:4d} | "
                f"center_x={camera_center_x:4d} | "
                f"error_x={output_error_x:+5d}",
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
                f"ERROR X: {output_error_x:+d} px",
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2
            )

        else:

            color = (0, 255, 0)


        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
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
            f"ID {track_id} {confidence:.2f}",
            (
                x1,
                max(y1 - 10, 20)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )


    # ========================================================
    # Target status
    # ========================================================

    if locked_track_id is None:

        status_text = (
            "SEARCHING - PRESS SPACE TO LOCK"
        )

    elif target_found:

        status_text = (
            f"LOCKED ID: {locked_track_id}"
        )

    else:

        status_text = (
            f"TARGET LOST: {locked_track_id}"
        )


    cv2.putText(
        frame,
        status_text,
        (20, height - 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )


    # ========================================================
    # Show image
    # ========================================================

    cv2.imshow(
        "YOLO Person Tracker",
        frame
    )


    key = cv2.waitKey(1) & 0xFF


    # ========================================================
    # SPACE -> lock nearest person
    # ========================================================

    if key == ord(" "):

        if not persons:

            print(
                "\n[WARN] No person detected."
            )

        else:

            nearest_person = min(
                persons,
                key=lambda p: p["distance"]
            )

            locked_track_id = (
                nearest_person["id"]
            )

            print(
                f"\n[INFO] Locked target ID: "
                f"{locked_track_id}"
            )


    # ========================================================
    # R -> release target
    # ========================================================

    elif key in (
        ord("r"),
        ord("R")
    ):

        print(
            "\n[INFO] Target released."
        )

        locked_track_id = None


    # ========================================================
    # ESC -> exit
    # ========================================================

    elif key == 27:

        print(
            "\n[INFO] Exit."
        )

        break


# ============================================================
# Cleanup
# ============================================================

cap.release()

cv2.destroyAllWindows()
