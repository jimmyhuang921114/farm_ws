#!/usr/bin/env python3

import cv2
import numpy as np
from pathlib import Path
import re
import subprocess
import sys
import time
import yaml


# ============================================================
# DECXIN CAMERA
# ============================================================

TARGET_VID = "1bcf"
TARGET_PID = "2cd1"

WIDTH = 640
HEIGHT = 480
FPS = 60

# ============================================================
# ChArUco board configuration
# Based on your C++ detector
# ============================================================

SQUARES_X = 10
SQUARES_Y = 14

SQUARE_LENGTH = 0.020      # 20 mm
MARKER_LENGTH = 0.015      # 15 mm

ARUCO_DICT = cv2.aruco.DICT_6X6_250

TARGET_IMAGES = 30


ROOT = Path(__file__).resolve().parent

CAPTURE_DIR = ROOT / "calibration_images"

OUTPUT_YAML = ROOT / "camera_info.yaml"

OUTPUT_INI = ROOT / "camera_calibration.ini"


# ============================================================
# USB / VIDEO DEVICE
# ============================================================

def usb_id(device):
    """
    Get USB VID/PID for /dev/videoX.
    """

    name = Path(device).name

    sys_path = (
        Path("/sys/class/video4linux")
        / name
        / "device"
    )

    try:
        current = sys_path.resolve()
    except Exception:
        return None, None

    for parent in [current, *current.parents]:

        vendor = parent / "idVendor"
        product = parent / "idProduct"

        if vendor.exists() and product.exists():

            try:
                vid = vendor.read_text().strip().lower()
                pid = product.read_text().strip().lower()

                return vid, pid

            except Exception:
                pass

    return None, None


def video_nodes():

    devices = []

    for path in Path("/dev").glob("video*"):

        m = re.fullmatch(
            r"video(\d+)",
            path.name,
        )

        if m:

            devices.append(
                (
                    int(m.group(1)),
                    str(path),
                )
            )

    devices.sort()

    return [
        device
        for _, device in devices
    ]


def detect_camera():

    print(
        f"[INFO] Searching DECXIN CAMERA "
        f"{TARGET_VID}:{TARGET_PID}"
    )

    matches = []

    for device in video_nodes():

        vid, pid = usb_id(device)

        if (
            vid == TARGET_VID
            and pid == TARGET_PID
        ):

            print(
                f"[INFO] Found USB match: "
                f"{device}"
            )

            matches.append(device)

    if not matches:

        raise RuntimeError(
            f"Camera {TARGET_VID}:{TARGET_PID} "
            "not found."
        )

    # One USB camera can expose multiple /dev/videoX.
    # Test which one can actually capture RGB frames.
    for device in matches:

        print(
            f"[INFO] Testing {device}..."
        )

        cap = cv2.VideoCapture(
            device,
            cv2.CAP_V4L2,
        )

        if not cap.isOpened():

            cap.release()

            continue

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                *"MJPG"
            ),
        )

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            WIDTH,
        )

        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            HEIGHT,
        )

        cap.set(
            cv2.CAP_PROP_FPS,
            FPS,
        )

        ok = False

        for _ in range(10):

            ret, frame = cap.read()

            if (
                ret
                and frame is not None
                and frame.size > 0
            ):

                ok = True
                break

        cap.release()

        if ok:

            print(
                f"[INFO] Using camera: "
                f"{device}"
            )

            return device

    raise RuntimeError(
        "USB camera exists, but no "
        "capture-capable video node found."
    )


# ============================================================
# CHARUCO BOARD
# ============================================================

def create_board():

    dictionary = (
        cv2.aruco.getPredefinedDictionary(
            ARUCO_DICT
        )
    )

    # Compatible with different OpenCV generations.
    try:

        board = cv2.aruco.CharucoBoard(
            (
                SQUARES_X,
                SQUARES_Y,
            ),
            SQUARE_LENGTH,
            MARKER_LENGTH,
            dictionary,
        )

    except Exception:

        board = (
            cv2.aruco.CharucoBoard_create(
                SQUARES_X,
                SQUARES_Y,
                SQUARE_LENGTH,
                MARKER_LENGTH,
                dictionary,
            )
        )

    return dictionary, board


# ============================================================
# DETECTION
# ============================================================

def detect_charuco(
    frame,
    dictionary,
    board,
):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY,
    )

    detector_parameters = (
        cv2.aruco.DetectorParameters()
    )

    try:

        detector = cv2.aruco.ArucoDetector(
            dictionary,
            detector_parameters,
        )

        marker_corners, marker_ids, rejected = (
            detector.detectMarkers(gray)
        )

    except AttributeError:

        marker_corners, marker_ids, rejected = (
            cv2.aruco.detectMarkers(
                gray,
                dictionary,
                parameters=detector_parameters,
            )
        )

    charuco_corners = None
    charuco_ids = None

    if (
        marker_ids is not None
        and len(marker_ids) > 0
    ):

        try:

            count, charuco_corners, charuco_ids = (
                cv2.aruco.interpolateCornersCharuco(
                    marker_corners,
                    marker_ids,
                    gray,
                    board,
                )
            )

        except cv2.error:

            return (
                gray,
                marker_corners,
                marker_ids,
                None,
                None,
                rejected,
            )

    return (
        gray,
        marker_corners,
        marker_ids,
        charuco_corners,
        charuco_ids,
        rejected,
    )


# ============================================================
# SAVE ROS CAMERA INFO YAML
# ============================================================

def save_ros_yaml(
    camera_matrix,
    distortion,
    rms,
):

    fx = float(
        camera_matrix[0, 0]
    )

    fy = float(
        camera_matrix[1, 1]
    )

    cx = float(
        camera_matrix[0, 2]
    )

    cy = float(
        camera_matrix[1, 2]
    )

    d = distortion.flatten().tolist()

    # Make sure at least 5 values exist.
    while len(d) < 5:
        d.append(0.0)

    data = {
        "image_width": WIDTH,
        "image_height": HEIGHT,

        "camera_name":
            "decxin_camera",

        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                fx,
                0.0,
                cx,

                0.0,
                fy,
                cy,

                0.0,
                0.0,
                1.0,
            ],
        },

        "distortion_model":
            "plumb_bob",

        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [
                float(d[0]),
                float(d[1]),
                float(d[2]),
                float(d[3]),
                float(d[4]),
            ],
        },

        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        },

        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },

        "calibration_rms_error":
            float(rms),
    }

    with OUTPUT_YAML.open(
        "w"
    ) as f:

        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
        )


# ============================================================
# SAVE YOUR C++ INI FORMAT
# ============================================================

def save_ini(
    camera_matrix,
    distortion,
):

    d = distortion.flatten().tolist()

    while len(d) < 5:
        d.append(0.0)

    with OUTPUT_INI.open(
        "w"
    ) as f:

        f.write(
            "[Intrinsic]\n"
        )

        for i in range(3):

            for j in range(3):

                f.write(
                    f"{i}_{j}="
                    f"{camera_matrix[i, j]:.12f}\n"
                )

        f.write(
            "\n[Distortion]\n"
        )

        f.write(
            f"k1={d[0]:.12f}\n"
        )

        f.write(
            f"k2={d[1]:.12f}\n"
        )

        f.write(
            f"t1={d[2]:.12f}\n"
        )

        f.write(
            f"t2={d[3]:.12f}\n"
        )

        f.write(
            f"k3={d[4]:.12f}\n"
        )


# ============================================================
# CALIBRATION
# ============================================================

def calibrate(
    all_corners,
    all_ids,
    board,
):

    print()
    print(
        "======================================"
    )

    print(
        "[INFO] Running camera calibration..."
    )

    print(
        "======================================"
    )

    image_size = (
        WIDTH,
        HEIGHT,
    )

    result = (
        cv2.aruco.calibrateCameraCharuco(
            charucoCorners=
                all_corners,

            charucoIds=
                all_ids,

            board=
                board,

            imageSize=
                image_size,

            cameraMatrix=None,

            distCoeffs=None,
        )
    )

    (
        rms,
        camera_matrix,
        distortion,
        rvecs,
        tvecs,
    ) = result

    print()
    print(
        f"RMS reprojection error: "
        f"{rms:.6f}"
    )

    print()
    print(
        "Camera matrix:"
    )

    print(
        camera_matrix
    )

    print()
    print(
        "Distortion:"
    )

    print(
        distortion
    )

    save_ros_yaml(
        camera_matrix,
        distortion,
        rms,
    )

    save_ini(
        camera_matrix,
        distortion,
    )

    print()
    print(
        f"[SAVE] {OUTPUT_YAML}"
    )

    print(
        f"[SAVE] {OUTPUT_INI}"
    )

    return (
        rms,
        camera_matrix,
        distortion,
    )


# ============================================================
# MAIN
# ============================================================

def main():

    CAPTURE_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    dictionary, board = (
        create_board()
    )

    device = detect_camera()

    cap = cv2.VideoCapture(
        device,
        cv2.CAP_V4L2,
    )

    if not cap.isOpened():

        raise RuntimeError(
            f"Cannot open {device}"
        )

    # --------------------------------------------------------
    # MJPG
    # --------------------------------------------------------

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(
            *"MJPG"
        ),
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        WIDTH,
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        HEIGHT,
    )

    cap.set(
        cv2.CAP_PROP_FPS,
        FPS,
    )

    # Small buffer helps reduce old frames.
    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    print()
    print(
        "======================================"
    )

    print(
        "DECXIN ChArUco Camera Calibration"
    )

    print(
        "======================================"
    )

    print(
        f"Camera: {device}"
    )

    print(
        f"USB ID: "
        f"{TARGET_VID}:{TARGET_PID}"
    )

    print(
        f"Resolution: "
        f"{WIDTH}x{HEIGHT}"
    )

    print(
        f"Requested FPS: {FPS}"
    )

    print()

    print(
        f"Board: "
        f"{SQUARES_X} x "
        f"{SQUARES_Y}"
    )

    print(
        f"Square: "
        f"{SQUARE_LENGTH * 1000:.1f} mm"
    )

    print(
        f"Marker: "
        f"{MARKER_LENGTH * 1000:.1f} mm"
    )

    print(
        "Dictionary: DICT_6X6_250"
    )

    print()
    print(
        "SPACE = save calibration image"
    )

    print(
        "BACKSPACE = remove last image"
    )

    print(
        "Q = quit"
    )

    print()
    print(
        f"Target: {TARGET_IMAGES} images"
    )

    print(
        "======================================"
    )

    all_corners = []
    all_ids = []

    accepted = 0

    while True:

        ok, frame = cap.read()

        if (
            not ok
            or frame is None
        ):

            print(
                "[WARN] Camera frame failed"
            )

            continue

        (
            gray,
            marker_corners,
            marker_ids,
            charuco_corners,
            charuco_ids,
            rejected,
        ) = detect_charuco(
            frame,
            dictionary,
            board,
        )

        display = frame.copy()

        marker_count = 0
        corner_count = 0

        if (
            marker_ids is not None
            and len(marker_ids) > 0
        ):

            marker_count = len(
                marker_ids
            )

            cv2.aruco.drawDetectedMarkers(
                display,
                marker_corners,
                marker_ids,
            )

        if (
            charuco_ids is not None
            and charuco_corners is not None
        ):

            corner_count = len(
                charuco_ids
            )

            cv2.aruco.drawDetectedCornersCharuco(
                display,
                charuco_corners,
                charuco_ids,
                (
                    0,
                    255,
                    0,
                ),
            )

        # Require enough corners to avoid useless samples.
        valid = (
            charuco_ids is not None
            and corner_count >= 12
        )

        status_color = (
            (0, 255, 0)
            if valid
            else (0, 0, 255)
        )

        cv2.putText(
            display,

            f"Images: "
            f"{accepted}/{TARGET_IMAGES}",

            (
                10,
                30,
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.8,

            (
                255,
                255,
                255,
            ),

            2,
        )

        cv2.putText(
            display,

            f"Markers: "
            f"{marker_count}  "
            f"Corners: "
            f"{corner_count}",

            (
                10,
                60,
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            status_color,

            2,
        )

        cv2.putText(
            display,

            (
                "GOOD - SPACE to capture"
                if valid
                else
                "Move board / improve detection"
            ),

            (
                10,
                90,
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.65,

            status_color,

            2,
        )

        cv2.imshow(
            "ChArUco Calibration",
            display,
        )

        key = (
            cv2.waitKey(1)
            & 0xFF
        )

        # Q
        if key in (
            ord("q"),
            ord("Q"),
        ):

            break

        # SPACE
        if key == 32:

            if not valid:

                print(
                    "[SKIP] Not enough "
                    "ChArUco corners."
                )

                continue

            # Important: copy calibration data.
            all_corners.append(
                charuco_corners.copy()
            )

            all_ids.append(
                charuco_ids.copy()
            )

            filename = (
                CAPTURE_DIR
                / f"calib_{accepted + 1:02d}.jpg"
            )

            cv2.imwrite(
                str(filename),
                frame,
            )

            accepted += 1

            print(
                f"[CAPTURE] "
                f"{accepted:02d}/"
                f"{TARGET_IMAGES} "
                f"| markers="
                f"{marker_count} "
                f"| corners="
                f"{corner_count}"
            )

            # Visual confirmation.
            white = np.full_like(
                display,
                255,
            )

            cv2.addWeighted(
                white,
                0.35,
                display,
                0.65,
                0,
                display,
            )

            cv2.imshow(
                "ChArUco Calibration",
                display,
            )

            cv2.waitKey(100)

            if (
                accepted
                >= TARGET_IMAGES
            ):

                print()
                print(
                    "[INFO] 30 images collected."
                )

                break

        # Backspace
        if key in (
            8,
            127,
        ):

            if accepted > 0:

                accepted -= 1

                all_corners.pop()
                all_ids.pop()

                filename = (
                    CAPTURE_DIR
                    / f"calib_{accepted + 1:02d}.jpg"
                )

                if filename.exists():
                    filename.unlink()

                print(
                    f"[REMOVE] "
                    f"Now {accepted}/"
                    f"{TARGET_IMAGES}"
                )

    cap.release()

    cv2.destroyAllWindows()

    if accepted < TARGET_IMAGES:

        print()
        print(
            f"[INFO] Calibration stopped "
            f"with {accepted} images."
        )

        return

    # ========================================================
    # Calibration
    # ========================================================

    rms, matrix, distortion = calibrate(
        all_corners,
        all_ids,
        board,
    )

    print()
    print(
        "======================================"
    )

    print(
        "CALIBRATION COMPLETE"
    )

    print(
        "======================================"
    )

    print(
        f"RMS = {rms:.6f}"
    )

    print()
    print(
        "K ="
    )

    print(matrix)

    print()
    print(
        "D ="
    )

    print(distortion)

    print()
    print(
        "ROS camera info:"
    )

    print(
        OUTPUT_YAML
    )

    print()
    print(
        "ChArUco detector INI:"
    )

    print(
        OUTPUT_INI
    )


if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print(
            "\n[INFO] Interrupted."
        )

    except Exception as exc:

        print(
            f"\n[ERROR] {exc}",
            file=sys.stderr,
        )

        sys.exit(1)