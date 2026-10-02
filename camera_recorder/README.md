# USB RGB Camera Recorder

Standalone USB RGB dataset recorder for Ubuntu 22.04, Python 3, OpenCV and V4L2.

- USB VID/PID auto detection (`1bcf:2cd1`) and capture-node verification
- Native MJPG mode validation, V4L2 capture, high FPS capture where supported
- Manual exposure support with readback and restoration on normal exit
- Individual JPEG RGB images, monotonic/Unix timestamps and YAML metadata
- Optional MJPG AVI preview
- Bounded writer queue, newest-frame preview and capture FPS statistics

## Installation

```bash
sudo apt install python3-venv v4l-utils
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
python record_camera.py --help
```

Only `opencv-python` and `PyYAML` are direct Python dependencies. NumPy is installed by OpenCV. No ROS2, model, GPU or conda environment is used. Preview needs a graphical desktop; use `--no-preview` on a headless machine.

## Camera capability check

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/videoX --list-formats-ext
v4l2-ctl -d /dev/videoX --list-ctrls
v4l2-ctl -d /dev/videoX --list-ctrls-menus
```

See [the static test report](test_results/REPORT.md) and raw capability logs. Device numbering can change; `/dev/video0` is the capture node on the tested machine, `/dev/video1` is metadata only. Auto detection follows USB parents and verifies MJPG support. Multiple matching cameras require `--device`.

On this host sudo was unavailable. Small Ubuntu `v4l-utils`/`libv4l` debs were unpacked in ignored `.tools/`; the scripts fall back to this local executable. A fresh clone should install `v4l-utils` using the command above.

## Recording

Tested default: **640×480 MJPG, 60 FPS, two capture buffers, manual exposure 156, gain 255, power-line menu 1 (50 Hz)**. The 12-second benchmark measured 59.997 FPS, and the 10-second JPEG+AVI smoke test saved 600/600 frames with zero drops. This is a throughput-tested exposure candidate, **not a final exposure recommendation**. The current scene remains dark/noisy; improve illumination before collecting a training dataset. `--exposure-mode auto` gives brighter images but the tested scene reduced read FPS to about 20 even with two buffers. `--exposure-mode keep` retains exposure mode/value; gain and power-line options still apply.

`CAP_PROP_BUFFERSIZE=1` was accepted but delivered only ~30 FPS with manual exposure at requested 60 FPS. Two buffers delivered ~60 FPS, so they are the smallest tested sustainable setting. Use `--buffers 1` to reproduce the single-buffer test. The recorder first probes one buffer and records both that result and its final setting in metadata. Queue capacity defaults to 120 BGR frames (~106 MiB at 640×480, plus overhead); use `--queue-size` to lower it. It is a fixed capacity, not a growing history.

Controls changed by the recorder are restored on normal exit, Ctrl+C and SIGTERM. Forced termination/power loss cannot guarantee restoration or metadata finalization. A session with `status: recording` or `incomplete` needs inspection before use.

```bash
python record_camera.py
./run.sh
python record_camera.py \
    --width 640 \
    --height 480 \
    --fps 60 \
    --output ./recordings \
    --jpeg-quality 95
```

R = Start recording; S = Stop recording; Q = Quit. Each R creates a separate session. A bounded recording queue drops newly arriving frames when full; it never blocks camera reads. Preview shows the newest frame. Stats print every five seconds. Stop drains queued frames and finalizes metadata.

```bash
# Headless: automatically starts; Ctrl+C stops and flushes.
./run.sh --no-preview
# Small timed recording, including optional preview AVI:
./run.sh --no-preview --duration 10 --avi
# Explicit native manual exposure value (100 us units on this camera):
./run.sh --exposure 80
# Explicit node override:
./run.sh --device /dev/video0
```

Unsupported native modes are rejected. In particular, this camera does not advertise 640×480 MJPG at 90 or 120 FPS. `--fps` is a request; actual read FPS and saved FPS are measured separately. Scene illumination and exposure can reduce actual FPS.

## Output

```text
recordings/
└── session_YYYYMMDD_HHMMSS/
    ├── rgb/
    │   ├── 00000000.jpg
    │   ├── 00000001.jpg
    │   └── ...
    ├── timestamps.csv
    ├── metadata.yaml
    └── preview.avi          # only with --avi
```

JPEG quality defaults to 95. OpenCV decodes MJPG to BGR in memory and encodes standard color JPEGs; downstream OpenCV readers should convert BGR to RGB when required by their model. Capture JPEG is decoded and re-encoded, not a passthrough of the USB bitstream.

Timestamps are host times immediately after successful `cap.read()`, not sensor exposure timestamps. Monotonic time is appropriate for intervals. Unix time can be adjusted by the system clock. CSV includes source capture sequence so queue drops remain visible. AVI has constant playback FPS and does not preserve timing gaps; CSV and JPEGs are authoritative.

Metadata includes requested/negotiated modes, actual controls and raw control flags, capture/save/drop counts, read latency, duration and finalization status. Unavailable controls and physical values hidden by automatic control are null. Automatic focus, exposure and white balance may change during a session; snapshots do not provide per-frame sensor telemetry.

**Do not commit `recordings/` to GitHub.** `.gitignore` excludes datasets, AVI/video, `.venv`, local tools and caches; only `recordings/.gitkeep` is retained. No GitHub repository or push is required to use this project.

## Repeat the static benchmark

```bash
python scripts/camera_test.py --fps 30 60 --seconds 12
python scripts/camera_test.py --fps 60 --seconds 12 --exposure 156 --gain 255 --buffers 2
```

Each test warms up for two seconds. Measured FPS is successful frames / elapsed wall time. Intervals are between successful read completions; a stall is an interval greater than max(100 ms, three requested frame periods). Read latency includes waiting for the next camera frame and decoding; it is not an end-to-end optical latency measurement. One ignored JPEG per test is saved for visual inspection.

Motion blur: **NOT TESTED - camera currently cannot be moved.**
Exposure candidates **Require moving-camera validation**. When movement is possible, pan past a high-contrast edge/text at the intended operating speed, using the same illumination and fixed gain. Record 5–10 seconds each at exposure 40, 80 and 156; compare edge smear at full resolution, dark pixels/noise, highlights, actual FPS and drops. Choose the longest exposure that meets the blur limit and FPS requirement; add light if shorter exposures are too dark.

## Verification and Git

```bash
python -m unittest discover -s tests -v
python scripts/smoke_test.py   # creates ~34 MB of ignored 10-second test data

git status --short
git add .
git commit -m "Add USB RGB camera recorder"
```

An independent local Git repository was initialized in this directory; no commit, remote, login or push was performed. `git status --short --untracked-files=all` shows `recordings/.gitkeep` as the only dataset-directory candidate. Full raw measurements are under `test_results/`; sample images and test videos are ignored.
