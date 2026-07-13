# lidar_rgb_fusion

ROS2 Humble Python node that projects VLP-16 `/velodyne_points` into a RealSense RGB image, publishes colored LiDAR points, and publishes a debug projection image.

## Build

```bash
cd /workspace/farm_ws/pointcloud_process_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_rgb_fusion
source install/setup.bash
```

If numpy is missing:

```bash
sudo apt install python3-numpy
```

## Launch

```bash
export ROS_DOMAIN_ID=40
ros2 launch lidar_rgb_fusion lidar_rgb_fusion.launch.py
```

If synchronized messages are not received, try increasing `sync_slop` to `0.5`.

## Check Frames

```bash
ros2 topic echo /velodyne_points --once | grep frame_id
ros2 topic echo /camera/camera/color/image_raw --once | grep frame_id
ros2 topic echo /camera/camera/color/camera_info --once | grep frame_id
```

## Check TF

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame velodyne
```

If the TF does not exist, use a temporary static TF only to test the data flow:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  camera_color_optical_frame velodyne
```

This is only for testing the pipeline. It is not a correct extrinsic calibration. Real alignment requires LiDAR-camera extrinsic calibration.

## RViz

- Fixed Frame: `velodyne`
- Add PointCloud2: `/velodyne_colored_points`
- Color Transformer: `RGB8` or `RGB`

## Debug Image

```bash
rqt_image_view
```

Select:

```text
/lidar_rgb_fusion/debug_image
```

If you see `No LiDAR points projected into image`, the TF/extrinsic calibration is likely wrong. It does not necessarily mean the node is broken.
