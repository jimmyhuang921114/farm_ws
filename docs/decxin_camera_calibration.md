# DECXIN camera calibration at 1280x720

The integrated launcher uses the DECXIN USB camera in `1280x720` MJPEG mode.
No repository calibration was confirmed for this camera, so `open_camera`
reports `UNCALIBRATED` and publishes no `CameraInfo` until an operator supplies
a real calibration file.

Measure the calibration target before running the command. Replace:

- `<INNER_COLUMNS>` with the number of internal checkerboard corners across.
- `<INNER_ROWS>` with the number of internal checkerboard corners down.
- `<SQUARE_METERS>` with the measured side length of one square in metres.

```bash
ros2 run camera_calibration cameracalibrator \
  --size <INNER_COLUMNS>x<INNER_ROWS> \
  --square <SQUARE_METERS> \
  --ros-args \
  -r image:=/decxin_camera/image_raw \
  -r camera:=/decxin_camera
```

Save the resulting YAML, without editing its numeric values, under a deliberate
path such as:

```text
/workspace/farm_ws/config/camera/decxin_1280x720.yaml
```

Start the camera with the calibration URL:

```bash
ros2 run sensor_bringup open_camera --ros-args \
  -p width:=1280 -p height:=720 -p fps:=30.0 -p fourcc:=MJPG \
  -p camera_info_url:=file:///workspace/farm_ws/config/camera/decxin_1280x720.yaml
```

Verify that the image and calibration dimensions agree:

```bash
ros2 topic echo /decxin_camera/image_raw --once \
  --field width --qos-reliability best_effort
ros2 topic echo /decxin_camera/image_raw --once \
  --field height --qos-reliability best_effort
ros2 topic echo /decxin_camera/camera_info --once \
  --qos-reliability best_effort
```

Both messages must report `1280x720`; `k`, `d`, `r`, and `p` must come from the
saved calibration. Recalibrate if the lens, focus, resolution, or camera unit
changes.
