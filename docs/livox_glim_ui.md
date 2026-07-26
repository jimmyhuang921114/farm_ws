# Livox MID-360 + DECXIN + GLIM + Collection UI

## Paths and architecture

The host checkout is `/home/jimmy/farm_ws`. The Docker container mounts it at
`/workspace/farm_ws`; all ROS commands and runtime configuration use the
container path. The expected container is `farm_ws_glim_ui_dev`, running with
Docker host networking, ROS domain 40, and CycloneDDS.

The launcher loads `config/cyclonedds_livox.xml`: loopback is the preferred DDS
control path and `enp5s0` is an optional discovery interface. This keeps the
GLIM supervisor reachable while the LiDAR cable is unplugged.

Data flow:

```text
MID-360 /livox/lidar -> monitor + guard -> /livox/lidar_valid
MID-360 /livox/imu -> /livox/imu_base
  -> GLIM /glim_ros/{odom,map,points} and TF
  -> RViz2 and Collection health/UI

DECXIN 1280x720 /decxin_camera/image_raw
  -> RViz2 and Collection health/UI
```

The collection manager is UI-only. It does not create MCAP or record sensor
data.

## Hardware and addresses

- Connect the MID-360 Ethernet cable to host interface `enp5s0`.
- MID-360 address: `192.168.113.158`.
- Host address on `enp5s0`: `192.168.113.1/24`.
- Connect the DECXIN camera over USB. Expected USB ID: `1bcf:2cd1`.
- Docker must use `--network host`; bridge networking cannot receive the
  configured Livox unicast UDP streams correctly.

Configure the host network deliberately, outside Docker:

```bash
cd /home/jimmy/farm_ws
sudo ./docker/setup_livox_network.sh
```

The helper does not flush addresses, change the default route, or modify the
firewall.

## Build and launch

Build the image and enter the container:

```bash
cd /home/jimmy/farm_ws
./docker/build.sh
./docker/run.sh
```

Inside `farm_ws_glim_ui_dev`, build the workspace if needed:

```bash
cd /workspace/farm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

Run the integrated stack from any container working directory:

```bash
/workspace/farm_ws/scripts/run_livox_glim_ui.sh
```

Headless operation:

```bash
/workspace/farm_ws/scripts/run_livox_glim_ui.sh --headless
```

Other bounded modes:

```bash
# Continue without the DECXIN camera.
/workspace/farm_ws/scripts/run_livox_glim_ui.sh --skip-camera

# Validate sensors and UI without GLIM.
/workspace/farm_ws/scripts/run_livox_glim_ui.sh --no-glim

# Static software/configuration validation only.
/workspace/farm_ws/scripts/run_livox_glim_ui.sh --dry-run
```

The script refuses to report readiness until required topics contain messages.
It stores one PID and log per child under
`/tmp/farm_ws_runtime/<UTC timestamp>/`. To stop normally, press Ctrl+C in the
launching terminal. The trap signals only processes started by that invocation;
it never uses `killall` or a broad `pkill`.

## Calibration status

The permanent GLIM profile is `config/livox_mid360`. The PointCloud2
`timestamp` field is handled through GLIM's per-point-time auto-configuration.
GLIM subscribes to guarded `/livox/lidar_valid`; the original `/livox/lidar`
is unchanged. Per-frame JSONL/CSV evidence and exact anomalous binary records
are written below each run's `cloud_monitor/` directory. The guard threshold
uses 10% of a rolling median after 20 healthy frames, plus validity-ratio and
absolute safety checks; it is not tuned to the observed 96-point failure.
The inherited `T_lidar_imu` is not a confirmed MID-360 calibration. See
`config/livox_mid360/CALIBRATION_REQUIRED.md` before collecting mapping data.

Run a continuous 60-second LiDAR/IMU time comparison while the stack is up:

```bash
ros2 run sensor_bringup livox_time_sync_monitor.py --ros-args \
  -p duration_sec:=60.0 \
  -p output_path:=/tmp/livox_time_sync.json
```

The reported signed delta is LiDAR header stamp minus the nearest IMU header
stamp. Receive-wall offsets are reported separately and must not be confused
with sensor timestamp offset.

No DECXIN calibration file was confirmed on 2026-07-22. `open_camera` therefore
logs `UNCALIBRATED` and does not publish CameraInfo unless a standard
`camera_calibration` YAML is passed as `camera_info_url`. Do not invent camera
intrinsics. A typical calibration command is:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 \
  --ros-args -r image:=/decxin_camera/image_raw \
  -r camera:=/decxin_camera
```

Replace the checkerboard size and square dimension with the actual target.
Then start the camera with
`-p camera_info_url:=file:///workspace/farm_ws/config/<calibration>.yaml`.

## Troubleshooting

### `bind failed`

The Livox SDK could not bind a configured host IP/port. Confirm
`192.168.113.1/24` exists on `enp5s0`, the container uses host networking, and
UDP ports 56101 through 56501 are not already occupied.

### `NO-CARRIER`

The Ethernet interface has no physical link. Check MID-360 power and cable;
the network helper intentionally refuses to configure an interface without
carrier.

### `camera not found`

Run `lsusb | grep -i 1bcf:2cd1` on the host. Recreate the container after
connecting the device so its video nodes and USB bus permissions are present,
or use `--skip-camera` for a LiDAR-only run.

### QoS incompatible

Livox and camera sensor topics use sensor-data/Best Effort QoS. Use
`rviz/livox_glim_ui.rviz`, whose raw LiDAR and Image displays are explicitly
Best Effort.

### `cudaErrorInsufficientDriver`

The NVIDIA kernel driver/runtime is unavailable or mismatched. The committed
MID-360 profile selects GLIM CPU modules; the launcher warns but does not claim
CUDA acceleration.

### GLIM has no odometry

Check that `/livox/lidar_valid`, `/livox/imu_base`, and their timestamps are all
advancing. Inspect the run's `glim.log` for deskew, per-point timestamp, IMU
initialization, and extrinsic warnings. A GLIM process or advertised publisher
alone is not proof of mapping output.

The launcher requires odometry and registered-point messages plus a typed map
publisher. A first `/glim_ros/map` message is not expected until motion creates
enough keyframes for a submap.

If GLIM terminates with `IndexedSlidingWindow: index out of range`, inspect the
preceding point-count and timestamp warnings plus `cloud_monitor/frames.jsonl`.
The 2026-07-22 failure began with simultaneous LiDAR and IMU gaps of about
5690 seconds, a cross-gap per-point timestamp range, then repeated 96-point and
empty clouds. The guard blocks low-quality frames and preserves evidence, but
the upstream exception itself is unchanged and needs a new long soak test.

Continuity gating and GLIM session splitting are documented in
`docs/livox_continuity_sessions.md`. The LiDAR/IMU GLIM profile deliberately
subscribes to `/glim/disabled_image`: current mapping uses LiDAR and IMU, and
feeding 6.2 MB raw images into an unused GLIM queue caused DDS backpressure.
RViz, UI, and the lightweight health callback may still subscribe to
`/decxin_camera/image_raw`.
