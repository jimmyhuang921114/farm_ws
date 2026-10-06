#!/usr/bin/env python3

"""
USB RGB dataset recorder.

Features
--------
- Automatically detects /dev/video* camera
- V4L2 backend
- MJPG capture
- Default 640x480 @ 60 FPS
- Preview window
- R = start recording
- S = stop recording
- Q = quit
- JPEG dataset output
- timestamps.csv
- metadata.yaml
- optional preview AVI
- manual / auto exposure controls
- writer thread separated from capture thread
- no dependency on scripts.camera_test
"""

import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time

import cv2
import yaml


# ============================================================
# Project root
# ============================================================

ROOT = Path(__file__).resolve().parent


# ============================================================
# V4L2 helper functions
# ============================================================

def v4l(device, *args):
    """
    Run v4l2-ctl on a video device.

    Example:
        v4l("/dev/video2", "--all")
        v4l("/dev/video2", "--set-ctrl=gain=100")
    """

    cmd = [
        "v4l2-ctl",
        "-d",
        str(device),
        *[str(arg) for arg in args],
    ]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"v4l2-ctl failed:\n"
            f"Command: {' '.join(cmd)}\n"
            f"{result.stderr.strip()}"
        )

    return result.stdout


def controls(device):
    """
    Read all V4L2 controls.

    Returns
    -------
    values : dict
        Current control values.

    raw : str
        Original v4l2-ctl output.
    """

    raw = v4l(device, "--list-ctrls-menus")

    values = {}

    pattern = re.compile(
        r"^\s*([A-Za-z0-9_]+)\s+"
        r"0x[0-9a-fA-F]+\s+"
        r"\([^)]+\)\s*:"
        r".*?\bvalue=(-?\d+)",
        re.MULTILINE,
    )

    for match in pattern.finditer(raw):
        name = match.group(1)
        value = int(match.group(2))
        values[name] = value

    return values, raw


def usb_id(device):
    """
    Find USB VID/PID belonging to /dev/videoX.

    Returns
    -------
    (vid, pid)

    or

    (None, None)
    """

    device_name = Path(device).name

    sys_path = Path(
        "/sys/class/video4linux"
    ) / device_name / "device"

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

            except OSError:
                pass

    return None, None


def video_nodes():
    """
    Return /dev/video* devices ordered numerically.
    """

    devices = []

    for path in Path("/dev").glob("video*"):

        match = re.fullmatch(
            r"video(\d+)",
            path.name,
        )

        if match:
            devices.append(
                (
                    int(match.group(1)),
                    str(path),
                )
            )

    devices.sort(
        key=lambda item: item[0]
    )

    return [
        device
        for _, device in devices
    ]

TARGET_VID = "1bcf"
TARGET_PID = "2cd1"


def detect():
    devices = video_nodes()

    if not devices:
        raise RuntimeError("No /dev/video* devices found.")

    print(
        f"[INFO] Searching for DECXIN CAMERA "
        f"{TARGET_VID}:{TARGET_PID}...",
        flush=True,
    )

    matches = []

    for device in devices:
        try:
            info = v4l(device, "--all")
        except Exception:
            continue

        if (
            "Video Capture" not in info
            and "Video Capture Multiplanar" not in info
        ):
            continue

        vid, pid = usb_id(device)

        if vid == TARGET_VID and pid == TARGET_PID:
            matches.append(device)

    if not matches:
        raise RuntimeError(
            f"DECXIN CAMERA {TARGET_VID}:{TARGET_PID} "
            "not found."
        )

    print(
        f"[INFO] Matching DECXIN nodes: {matches}",
        flush=True,
    )

    # Usually one USB camera exposes multiple /dev/videoX nodes.
    # Try them in numeric order and return the first usable one.
    for device in matches:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not cap.isOpened():
            cap.release()
            continue

        ok, frame = cap.read()
        cap.release()

        if ok and frame is not None:
            print(
                f"[INFO] Using DECXIN CAMERA: {device} "
                f"(USB {TARGET_VID}:{TARGET_PID})",
                flush=True,
            )
            return device

    raise RuntimeError(
        f"Found DECXIN CAMERA {TARGET_VID}:{TARGET_PID}, "
        "but none of its /dev/video* nodes could capture frames."
    )


# ============================================================
# Camera initialization
# ============================================================

def decode_fourcc(value):
    value = int(value)

    return "".join(
        chr(
            (value >> (8 * i))
            & 0xFF
        )
        for i in range(4)
    )


def open_camera(
    device,
    width,
    height,
    fps,
    buffers=1,
):
    """
    Open USB camera using OpenCV + V4L2.

    Requests:
        MJPG
        width x height
        FPS
        buffer size

    Returns
    -------
    cap
    settings
    """

    print(
        f"[INFO] Opening {device}...",
        flush=True,
    )

    cap = cv2.VideoCapture(
        device,
        cv2.CAP_V4L2,
    )

    if not cap.isOpened():

        cap.release()

        raise RuntimeError(
            f"Unable to open camera {device}"
        )

    # --------------------------------------------------------
    # MJPG
    # --------------------------------------------------------

    mjpg = cv2.VideoWriter_fourcc(
        *"MJPG"
    )

    fourcc_accepted = cap.set(
        cv2.CAP_PROP_FOURCC,
        mjpg,
    )

    # --------------------------------------------------------
    # Resolution
    # --------------------------------------------------------

    width_accepted = cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        int(width),
    )

    height_accepted = cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        int(height),
    )

    # --------------------------------------------------------
    # FPS
    # --------------------------------------------------------

    fps_accepted = cap.set(
        cv2.CAP_PROP_FPS,
        float(fps),
    )

    # --------------------------------------------------------
    # Buffer
    # --------------------------------------------------------

    buffersize_accepted = cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        int(buffers),
    )

    # Give driver time to finish negotiation.
    time.sleep(0.2)

    reported_width = int(
        round(
            cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )
    )

    reported_height = int(
        round(
            cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )
    )

    reported_fps = cap.get(
        cv2.CAP_PROP_FPS
    )

    reported_buffers = cap.get(
        cv2.CAP_PROP_BUFFERSIZE
    )

    reported_fourcc = decode_fourcc(
        cap.get(
            cv2.CAP_PROP_FOURCC
        )
    )

    settings = {
        "fourcc_accepted":
            bool(fourcc_accepted),

        "fourcc_reported":
            reported_fourcc,

        "width_accepted":
            bool(width_accepted),

        "width_reported":
            reported_width,

        "height_accepted":
            bool(height_accepted),

        "height_reported":
            reported_height,

        "fps_accepted":
            bool(fps_accepted),

        "fps_reported":
            float(reported_fps),

        "buffersize_accepted":
            bool(buffersize_accepted),

        "buffersize_reported":
            float(reported_buffers),
    }

    print(
        "[INFO] Negotiated: "
        f"{reported_fourcc} "
        f"{reported_width}x"
        f"{reported_height} "
        f"@ {reported_fps:.2f} FPS "
        f"| buffer="
        f"{reported_buffers:g}",
        flush=True,
    )

    if (
        reported_width != width
        or reported_height != height
    ):

        cap.release()

        raise RuntimeError(
            "Camera rejected requested "
            "resolution. "
            f"Requested "
            f"{width}x{height}, "
            f"got "
            f"{reported_width}x"
            f"{reported_height}"
        )

    # --------------------------------------------------------
    # Test read
    # --------------------------------------------------------

    success = False

    for _ in range(20):

        ok, frame = cap.read()

        if ok and frame is not None:

            success = True
            break

        time.sleep(0.05)

    if not success:

        cap.release()

        raise RuntimeError(
            f"Camera opened but no frame "
            f"could be read from {device}"
        )

    return cap, settings


# ============================================================
# Camera controls
# ============================================================

def control_snapshot(device):

    try:

        values, raw = controls(device)

        error = None

    except Exception as exc:

        values = {}

        raw = None

        error = str(exc)

    def active_value(name):

        if raw and re.search(
            r"^\s*"
            + re.escape(name)
            + r"\s+.*flags=.*inactive",
            raw,
            re.MULTILINE,
        ):

            return None

        return values.get(name)

    return {
        "exposure_mode":
            values.get(
                "auto_exposure"
            ),

        "exposure_value":
            active_value(
                "exposure_time_absolute"
            ),

        "gain":
            values.get("gain"),

        "white_balance": {
            "automatic":
                values.get(
                    "white_balance_automatic"
                ),

            "temperature":
                active_value(
                    "white_balance_temperature"
                ),
        },

        "focus": {
            "automatic":
                values.get(
                    "focus_automatic_continuous"
                ),

            "absolute":
                active_value(
                    "focus_absolute"
                ),
        },

        "controls_readback":
            values,

        "controls_raw":
            raw,

        "controls_error":
            error,
    }


def set_control(
    device,
    name,
    value,
):

    _, raw = controls(device)

    line = next(
        (
            line
            for line in raw.splitlines()
            if re.match(
                r"^\s*"
                + re.escape(name)
                + r"\s+0x",
                line,
            )
        ),
        None,
    )

    if line is None:

        raise ValueError(
            f"Control {name} "
            f"is unavailable"
        )

    fields = {
        key: int(value_string)
        for key, value_string
        in re.findall(
            r"(min|max|step)="
            r"(-?\d+)",
            line,
        )
    }

    if (
        "min" in fields
        and "max" in fields
        and not (
            fields["min"]
            <= value
            <= fields["max"]
        )
    ):

        raise ValueError(
            f"{name}={value} "
            f"outside {fields}"
        )

    if (
        fields.get("step")
        and "min" in fields
        and (
            value
            - fields["min"]
        )
        % fields["step"]
        != 0
    ):

        raise ValueError(
            f"{name}={value} "
            f"violates step "
            f"{fields['step']}"
        )

    v4l(
        device,
        f"--set-ctrl="
        f"{name}={value}",
    )

    got, _ = controls(device)

    if got.get(name) != value:

        raise RuntimeError(
            f"{name} readback "
            f"differs: "
            f"{got.get(name)}"
        )


def apply_controls(
    device,
    args,
    saved,
):

    try:
        before, _ = controls(device)

    except Exception as exc:

        print(
            "[WARN] Could not read "
            f"V4L2 controls: {exc}",
            flush=True,
        )

        return

    names = [
        "auto_exposure",
        "exposure_time_absolute",
        "gain",
        "power_line_frequency",
    ]

    saved.update(
        {
            key: before[key]
            for key in names
            if key in before
        }
    )

    try:

        if (
            args.exposure_mode
            == "manual"
        ):

            if (
                "auto_exposure"
                in before
            ):

                set_control(
                    device,
                    "auto_exposure",
                    1,
                )

            if (
                "exposure_time_absolute"
                in before
            ):

                set_control(
                    device,
                    "exposure_time_absolute",
                    args.exposure,
                )

        elif (
            args.exposure_mode
            == "auto"
        ):

            if (
                "auto_exposure"
                in before
            ):

                set_control(
                    device,
                    "auto_exposure",
                    3,
                )

        if (
            args.gain is not None
            and "gain" in before
        ):

            set_control(
                device,
                "gain",
                args.gain,
            )

        if (
            args.power_line_frequency
            is not None
            and "power_line_frequency"
            in before
        ):

            set_control(
                device,
                "power_line_frequency",
                args.power_line_frequency,
            )

    except Exception as exc:

        print(
            "[WARN] Camera control "
            f"configuration failed: "
            f"{exc}",
            flush=True,
        )


def restore_controls(
    device,
    saved,
):

    if not saved:
        return

    try:

        if (
            "exposure_time_absolute"
            in saved
        ):

            if (
                "auto_exposure"
                in saved
            ):

                set_control(
                    device,
                    "auto_exposure",
                    1,
                )

            set_control(
                device,
                "exposure_time_absolute",
                saved[
                    "exposure_time_absolute"
                ],
            )

        for name in [
            "gain",
            "power_line_frequency",
            "auto_exposure",
        ]:

            if name in saved:

                set_control(
                    device,
                    name,
                    saved[name],
                )

    except Exception as exc:

        print(
            "[WARN] Restore controls "
            f"failed: {exc}",
            flush=True,
        )


# ============================================================
# Metadata
# ============================================================

def save_metadata(
    path,
    data,
):

    temporary = path.with_suffix(
        ".yaml.tmp"
    )

    temporary.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    temporary.replace(path)


# ============================================================
# Recording session
# ============================================================

class Session:

    def __init__(
        self,
        args,
        base,
    ):

        args.output.mkdir(
            parents=True,
            exist_ok=True,
        )

        stem = (
            "session_"
            + datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
        )

        for number in range(10000):

            if number == 0:

                name = stem

            else:

                name = (
                    f"{stem}_"
                    f"{number:02d}"
                )

            self.path = (
                args.output / name
            )

            try:

                self.path.mkdir()

                break

            except FileExistsError:

                continue

        else:

            raise RuntimeError(
                "Cannot allocate "
                "unique session directory"
            )

        (
            self.path / "rgb"
        ).mkdir()

        self.args = args

        self.queue = queue.Queue(
            maxsize=args.queue_size
        )

        self.closing = (
            threading.Event()
        )

        self.ready = (
            threading.Event()
        )

        self.lock = (
            threading.Lock()
        )

        self.captured = 0
        self.saved = 0
        self.dropped = 0
        self.write_failed = 0

        self.avi_frames = 0

        self.error = None
        self.avi_error = None

        self.read_ms = 0.0

        self.start = time.monotonic()
        self.end = None

        self.meta = dict(
            base,

            recording_start_time=
                datetime.now()
                .astimezone()
                .isoformat(),

            recording_start_unix=
                time.time(),

            recording_start_monotonic=
                self.start,

            status="initializing",

            jpeg_quality=
                args.jpeg_quality,

            queue_capacity=
                args.queue_size,

            avi_requested=
                args.avi,

            timestamp_semantics=(
                "Host read completion, "
                "not sensor exposure time"
            ),
        )

        save_metadata(
            self.path
            / "metadata.yaml",
            self.meta,
        )

        self.thread = threading.Thread(
            target=self._write,
            name="dataset-writer",
            daemon=True,
        )

        self.thread.start()

        if not self.ready.wait(10):

            self.closing.set()

            raise RuntimeError(
                "Writer initialization "
                f"timed out: {self.path}"
            )

        if self.error:

            self.close()

            raise RuntimeError(
                self.error
            )

        self.start = time.monotonic()

        self.meta.update(
            recording_start_monotonic=
                self.start,

            recording_start_unix=
                time.time(),

            recording_start_time=
                datetime.now()
                .astimezone()
                .isoformat(),

            status="recording",
        )

        save_metadata(
            self.path
            / "metadata.yaml",
            self.meta,
        )

        print(
            f"[REC] {self.path}",
            flush=True,
        )


    def offer(
        self,
        frame,
        mono,
        unix,
        sequence,
        read_ms,
    ):

        with self.lock:

            self.captured += 1

            self.read_ms += read_ms

            try:

                self.queue.put_nowait(
                    (
                        frame,
                        mono,
                        unix,
                        sequence,
                    )
                )

            except queue.Full:

                self.dropped += 1


    def _write(self):

        video = None

        try:

            timestamp_file = (
                self.path
                / "timestamps.csv"
            )

            with timestamp_file.open(
                "x",
                newline="",
            ) as file:

                writer = csv.writer(file)

                writer.writerow(
                    [
                        "frame_id",
                        "timestamp_monotonic",
                        "timestamp_unix",
                        "capture_sequence",
                    ]
                )

                file.flush()

                if self.args.avi:

                    video = (
                        cv2.VideoWriter(
                            str(
                                self.path
                                / "preview.avi"
                            ),

                            cv2.VideoWriter_fourcc(
                                *"MJPG"
                            ),

                            self.args.fps,

                            (
                                self.args.width,
                                self.args.height,
                            ),
                        )
                    )

                    if not video.isOpened():

                        self.avi_error = (
                            "VideoWriter "
                            "could not open"
                        )

                        video.release()
                        video = None

                self.ready.set()

                while (
                    not self.closing.is_set()
                    or not self.queue.empty()
                ):

                    try:

                        (
                            frame,
                            mono,
                            unix,
                            sequence,
                        ) = self.queue.get(
                            timeout=0.1
                        )

                    except queue.Empty:

                        continue

                    try:

                        frame_id = (
                            self.saved
                        )

                        filename = (
                            self.path
                            / "rgb"
                            / f"{frame_id:08d}.jpg"
                        )

                        success = cv2.imwrite(
                            str(filename),
                            frame,
                            [
                                cv2.IMWRITE_JPEG_QUALITY,
                                self.args.jpeg_quality,
                            ],
                        )

                        if not success:

                            raise IOError(
                                "JPEG write "
                                "returned False"
                            )

                        writer.writerow(
                            [
                                f"{frame_id:08d}",
                                f"{mono:.9f}",
                                f"{unix:.9f}",
                                sequence,
                            ]
                        )

                        file.flush()

                        with self.lock:

                            self.saved += 1

                        if video is not None:

                            try:

                                video.write(
                                    frame
                                )

                                self.avi_frames += 1

                            except Exception as exc:

                                self.avi_error = (
                                    str(exc)
                                )

                                video.release()

                                video = None

                    except Exception as exc:

                        self.error = str(exc)

                        self.write_failed += 1

                    finally:

                        self.queue.task_done()

        except Exception as exc:

            self.error = str(exc)

        finally:

            if video is not None:

                video.release()

            self.ready.set()


    def close(
        self,
        device=None,
    ):

        if self.end is not None:
            return self.meta

        self.end = time.monotonic()

        end_unix = time.time()

        self.closing.set()

        self.thread.join(
            timeout=30
        )

        if self.thread.is_alive():

            self.error = (
                "Writer did not drain "
                "within 30 seconds"
            )

        duration = max(
            self.end - self.start,
            1e-9,
        )

        self.meta.update(

            status=(
                "incomplete"
                if self.error
                else "complete"
            ),

            duration_seconds=
                duration,

            recording_end_monotonic=
                self.end,

            recording_end_unix=
                end_unix,

            captured_frames=
                self.captured,

            saved_frames=
                self.saved,

            dropped_frames=
                self.dropped,

            write_failed_frames=
                self.write_failed,

            unsaved_frames=
                self.captured
                - self.saved,

            measured_fps=
                self.captured
                / duration,

            effective_saved_fps=
                self.saved
                / duration,

            average_read_latency_ms=
                self.read_ms
                / max(
                    self.captured,
                    1,
                ),

            writer_error=
                self.error,

            avi_error=
                self.avi_error,

            avi_frames=
                self.avi_frames,
        )

        if device:

            self.meta[
                "controls_at_stop"
            ] = control_snapshot(
                device
            )

        save_metadata(
            self.path
            / "metadata.yaml",
            self.meta,
        )

        print(
            "\n"
            "[SESSION COMPLETE]\n"
            f"Duration: "
            f"{duration:.3f}s\n"
            f"Captured: "
            f"{self.captured}\n"
            f"Saved: "
            f"{self.saved}\n"
            f"Dropped: "
            f"{self.dropped}\n"
            f"Capture FPS: "
            f"{self.captured / duration:.2f}\n"
            f"Saved FPS: "
            f"{self.saved / duration:.2f}",
            flush=True,
        )

        return self.meta


# ============================================================
# Capture thread
# ============================================================

class Capture:

    def __init__(
        self,
        cap,
        args,
    ):

        self.cap = cap
        self.args = args

        self.lock = (
            threading.Lock()
        )

        self.stop = (
            threading.Event()
        )

        self.active = None
        self.latest = None

        self.total = 0
        self.failed = 0

        self.read_ms = 0.0

        self.last_success = (
            time.monotonic()
        )

        self.error = None

        self.thread = threading.Thread(
            target=self._run,
            name="camera-capture",
            daemon=True,
        )


    def _run(self):

        try:

            while not self.stop.is_set():

                start = time.monotonic()

                ok, frame = (
                    self.cap.read()
                )

                mono = time.monotonic()

                unix = time.time()

                read_ms = (
                    mono - start
                ) * 1000.0

                if (
                    not ok
                    or frame is None
                ):

                    with self.lock:
                        self.failed += 1

                    self.stop.wait(
                        0.005
                    )

                    continue

                expected = (
                    self.args.height,
                    self.args.width,
                    3,
                )

                if frame.shape != expected:

                    raise RuntimeError(
                        "Unexpected frame "
                        f"shape: {frame.shape}, "
                        f"expected {expected}"
                    )

                with self.lock:

                    self.total += 1

                    self.read_ms += (
                        read_ms
                    )

                    self.last_success = (
                        mono
                    )

                    self.latest = frame

                    active = (
                        self.active
                    )

                    sequence = (
                        self.total
                    )

                if (
                    active is not None
                    and mono
                    >= active.start
                ):

                    active.offer(
                        frame,
                        mono,
                        unix,
                        sequence,
                        read_ms,
                    )

        except Exception as exc:

            self.error = str(exc)

        finally:

            self.cap.release()


# ============================================================
# CLI
# ============================================================

def arguments():

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=
            argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--device",
        help=(
            "Camera device, e.g. "
            "/dev/video2. "
            "Default: auto detect"
        ),
    )

    parser.add_argument(
        "--width",
        type=int,
        default=640,
    )

    parser.add_argument(
        "--height",
        type=int,
        default=480,
    )

    parser.add_argument(
        "--fps",
        type=float,
        default=60,
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "recordings",
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
    )

    parser.add_argument(
        "--buffers",
        type=int,
        default=2,
    )

    parser.add_argument(
        "--queue-size",
        type=int,
        default=120,
    )

    parser.add_argument(
        "--no-preview",
        action="store_true",
    )

    parser.add_argument(
        "--avi",
        action="store_true",
    )

    parser.add_argument(
        "--duration",
        type=float,
        help=(
            "Automatically record "
            "for N seconds"
        ),
    )

    parser.add_argument(
        "--exposure-mode",
        choices=[
            "manual",
            "auto",
            "keep",
        ],
        default="keep",
    )

    parser.add_argument(
        "--exposure",
        type=int,
        default=156,
    )

    parser.add_argument(
        "--gain",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--power-line-frequency",
        type=int,
        choices=[
            0,
            1,
            2,
        ],
        default=None,
    )

    args = parser.parse_args()

    if not (
        1
        <= args.jpeg_quality
        <= 100
    ):

        parser.error(
            "--jpeg-quality "
            "must be 1..100"
        )

    if (
        args.width <= 0
        or args.height <= 0
        or args.fps <= 0
        or args.buffers <= 0
        or args.queue_size <= 0
    ):

        parser.error(
            "Width, height, FPS, "
            "buffers and queue-size "
            "must be positive"
        )

    if (
        args.duration is not None
        and args.duration <= 0
    ):

        parser.error(
            "--duration must "
            "be positive"
        )

    if (
        not args.no_preview
        and not (
            os.environ.get(
                "DISPLAY"
            )
            or os.environ.get(
                "WAYLAND_DISPLAY"
            )
        )
    ):

        parser.error(
            "No display available. "
            "Use --no-preview"
        )

    return args


# ============================================================
# Main
# ============================================================

def main():

    args = arguments()

    cv2.setNumThreads(1)

    args.output = (
        args.output
        .expanduser()
        .resolve()
    )

    device = args.device

    cap = None
    capture = None
    session = None

    saved_controls = {}

    quit_event = (
        threading.Event()
    )

    def signal_handler(
        *_,
    ):

        quit_event.set()

    signal.signal(
        signal.SIGINT,
        signal_handler,
    )

    signal.signal(
        signal.SIGTERM,
        signal_handler,
    )

    exit_code = 0

    try:

        # ----------------------------------------------------
        # Camera detection
        # ----------------------------------------------------

        device = (
            device
            or detect()
        )

        # ----------------------------------------------------
        # Open camera
        # ----------------------------------------------------

        cap, settings = open_camera(
            device,
            args.width,
            args.height,
            args.fps,
            1,
        )

        settings[
            "single_buffer_accepted"
        ] = settings[
            "buffersize_accepted"
        ]

        settings[
            "single_buffer_reported"
        ] = settings[
            "buffersize_reported"
        ]

        if args.buffers != 1:

            settings[
                "buffersize_accepted"
            ] = bool(
                cap.set(
                    cv2.CAP_PROP_BUFFERSIZE,
                    args.buffers,
                )
            )

            settings[
                "buffersize_reported"
            ] = cap.get(
                cv2.CAP_PROP_BUFFERSIZE
            )

        # ----------------------------------------------------
        # V4L controls
        # ----------------------------------------------------

        apply_controls(
            device,
            args,
            saved_controls,
        )

        print(
            "\n"
            "====================================\n"
            "USB CAMERA RECORDER\n"
            "===================================="
        )

        print(
            f"Device     : {device}"
        )

        print(
            f"Format     : MJPG"
        )

        print(
            f"Resolution : "
            f"{args.width}x{args.height}"
        )

        print(
            f"Requested  : "
            f"{args.fps:g} FPS"
        )

        print(
            f"Buffer     : "
            f"{settings['buffersize_reported']}"
        )

        print(
            f"Output     : "
            f"{args.output}"
        )

        print(
            "====================================\n"
        )

        # ----------------------------------------------------
        # Capture thread
        # ----------------------------------------------------

        capture = Capture(
            cap,
            args,
        )

        capture.thread.start()

        # Camera warm-up
        warmup_end = (
            time.monotonic()
            + 2.0
        )

        while (
            time.monotonic()
            < warmup_end
            and not quit_event.wait(
                0.02
            )
        ):

            if capture.error:

                raise RuntimeError(
                    capture.error
                )

        # ----------------------------------------------------
        # Metadata
        # ----------------------------------------------------

        vid, pid = usb_id(
            device
        )

        base = {

            "device":
                device,

            "usb_vid":
                vid,

            "usb_pid":
                pid,

            "width":
                args.width,

            "height":
                args.height,

            "requested_fps":
                args.fps,

            "pixel_format":
                "MJPG",

            "fourcc":
                "MJPG",

            "backend":
                "CAP_V4L2",

            "opencv_version":
                cv2.__version__,

            "negotiated":
                settings,
        }

        # ----------------------------------------------------
        # Window
        # ----------------------------------------------------

        window_name = (
            "USB RGB Camera | "
            "R record | "
            "S stop | "
            "Q quit"
        )

        if not args.no_preview:

            cv2.namedWindow(
                window_name,
                cv2.WINDOW_AUTOSIZE,
            )

        # ----------------------------------------------------
        # Session controls
        # ----------------------------------------------------

        def start_session():

            new_session = Session(
                args,
                dict(
                    base,
                    **control_snapshot(
                        device
                    ),
                ),
            )

            with capture.lock:

                capture.active = (
                    new_session
                )

            return new_session


        def stop_session(
            current,
        ):

            with capture.lock:

                capture.active = None

            current.meta[
                "capture_failed_reads_at_stop"
            ] = capture.failed

            return current.close(
                device
            )

        # ----------------------------------------------------
        # Auto-start for headless / duration
        # ----------------------------------------------------

        if (
            not quit_event.is_set()
            and (
                args.no_preview
                or args.duration
                is not None
            )
        ):

            session = (
                start_session()
            )

        # ----------------------------------------------------
        # Runtime stats
        # ----------------------------------------------------

        last_stats = (
            time.monotonic()
        )

        previous_total = (
            capture.total
        )

        previous_ms = (
            capture.read_ms
        )

        previous_saved = 0

        # ----------------------------------------------------
        # Main loop
        # ----------------------------------------------------

        while not quit_event.is_set():

            now = time.monotonic()

            if capture.error:

                raise RuntimeError(
                    capture.error
                )

            if (
                now
                - capture.last_success
                > 5
            ):

                raise RuntimeError(
                    "No successful camera "
                    "read for five seconds"
                )

            if (
                session
                and session.error
            ):

                raise RuntimeError(
                    "Writer failed: "
                    f"{session.error}"
                )

            if (
                session
                and args.duration
                and (
                    now
                    - session.start
                    >= args.duration
                )
            ):

                break

            with capture.lock:

                frame = capture.latest

                total = capture.total

                latency = (
                    capture.read_ms
                )

                failed = (
                    capture.failed
                )

            # ------------------------------------------------
            # Statistics every 5 sec
            # ------------------------------------------------

            if (
                now - last_stats
                >= 5
            ):

                current_saved = (
                    session.saved
                    if session
                    else 0
                )

                elapsed = (
                    now - last_stats
                )

                capture_fps = (
                    total
                    - previous_total
                ) / elapsed

                saved_fps = max(
                    0,
                    current_saved
                    - previous_saved,
                ) / elapsed

                reads = max(
                    1,
                    total
                    - previous_total,
                )

                average_read = (
                    latency
                    - previous_ms
                ) / reads

                print(
                    f"[STATS] "
                    f"Capture: "
                    f"{capture_fps:.1f} FPS | "
                    f"Recorded: "
                    f"{saved_fps:.1f} FPS | "
                    f"Dropped: "
                    f"{session.dropped if session else 0} | "
                    f"Queue: "
                    f"{session.queue.qsize() if session else 0}"
                    f"/{args.queue_size} | "
                    f"Read: "
                    f"{average_read:.1f} ms | "
                    f"Failed: "
                    f"{failed}",
                    flush=True,
                )

                previous_total = total
                previous_ms = latency
                previous_saved = (
                    current_saved
                )

                last_stats = now

            # ------------------------------------------------
            # Headless
            # ------------------------------------------------

            if args.no_preview:

                quit_event.wait(
                    0.005
                )

                continue

            # ------------------------------------------------
            # GUI
            # ------------------------------------------------

            if frame is not None:

                display = (
                    frame.copy()
                )

                if session:

                    status = (
                        "REC"
                    )

                    color = (
                        0,
                        0,
                        255,
                    )

                else:

                    status = (
                        "R: record | "
                        "S: stop | "
                        "Q: quit"
                    )

                    color = (
                        0,
                        255,
                        0,
                    )

                cv2.putText(
                    display,
                    status,
                    (
                        10,
                        30,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

                if session:

                    cv2.putText(
                        display,
                        (
                            f"Saved: "
                            f"{session.saved}"
                        ),
                        (
                            10,
                            60,
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (
                            0,
                            0,
                            255,
                        ),
                        2,
                    )

                cv2.imshow(
                    window_name,
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

            # R
            if (
                key
                in (
                    ord("r"),
                    ord("R"),
                )
                and session is None
            ):

                session = (
                    start_session()
                )

                previous_saved = 0

            # S
            if (
                key
                in (
                    ord("s"),
                    ord("S"),
                )
                and session
            ):

                summary = (
                    stop_session(
                        session
                    )
                )

                session = None

                previous_saved = 0

                if summary.get(
                    "writer_error"
                ):

                    raise RuntimeError(
                        summary[
                            "writer_error"
                        ]
                    )

            # Window closed
            if (
                cv2.getWindowProperty(
                    window_name,
                    cv2.WND_PROP_VISIBLE,
                )
                < 1
            ):

                break

    except Exception as exc:

        print(
            f"\nERROR: {exc}",
            file=sys.stderr,
            flush=True,
        )

        if session:

            session.meta[
                "recorder_error"
            ] = str(exc)

        exit_code = 1

    finally:

        # ----------------------------------------------------
        # Stop writer
        # ----------------------------------------------------

        if capture:

            with capture.lock:

                capture.active = None

            if session:

                session.meta[
                    "capture_failed_reads_at_stop"
                ] = capture.failed

                summary = session.close(
                    device
                )

                if summary.get(
                    "writer_error"
                ):

                    exit_code = 1

            # ------------------------------------------------
            # Stop capture
            # ------------------------------------------------

            capture.stop.set()

            capture.thread.join(
                timeout=6
            )

            if capture.thread.is_alive():

                print(
                    "ERROR: capture thread "
                    "did not stop",
                    file=sys.stderr,
                )

                exit_code = 1

        elif cap:

            cap.release()

        # ----------------------------------------------------
        # Restore camera settings
        # ----------------------------------------------------

        try:

            if saved_controls:

                restore_controls(
                    device,
                    saved_controls,
                )

        except Exception as exc:

            print(
                "WARNING: could not "
                "restore controls: "
                f"{exc}",
                file=sys.stderr,
            )

        # ----------------------------------------------------
        # GUI cleanup
        # ----------------------------------------------------

        if not args.no_preview:

            cv2.destroyAllWindows()

    return exit_code


if __name__ == "__main__":

    sys.exit(
        main()
    )