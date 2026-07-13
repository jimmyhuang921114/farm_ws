# VLP-16 KISS-ICP ROG-Map Bringup

This bringup keeps ROG-Map on a world-frame cloud. Do not feed `/velodyne_points`
directly to ROG-Map unless the ROG-Map input cloud is already in the same world
frame.

Pipeline:

```text
VLP-16 -> /velodyne_points -> KISS-ICP TF/odom -> /cloud_registered -> ROG-Map -> RViz2
```

Default frames/topics:

```text
LiDAR cloud:      /velodyne_points, frame velodyne
World frame:      odom
World cloud:      /cloud_registered, frame odom
KISS odometry:    /kiss/odometry
ROG occupancy:    /rog_map/occ
ROG inflated map: /rog_map/inf_occ
ROG ESDF:         /rog_map/esdf
ROG bounds:       /rog_map/map_bound
```

## Build

Use system CMake on this machine. The `/opt/venv` CMake 3.31 can fail in
PCL/VTK/MPI detection.

```bash
cd /workspace/farm_ws/pointcloud_process_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:$PATH colcon build --packages-select kiss_icp world_cloud_tools rog_map rog_map_vlp16_bringup --symlink-install
source install/setup.bash
```

## Terminal 1: VLP-16

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=40

ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py \
  device_ip:=192.168.1.201 \
  port:=2368 \
  frame_id:=velodyne \
  model:=VLP16
```

Check:

```bash
ros2 topic hz /velodyne_points
ros2 topic echo /velodyne_points --once | grep frame_id
```

## Terminal 2: KISS-ICP

KISS-ICP launch argument for the cloud is `topic`. The odometry topic is
`/kiss/odometry`. For this VLP-16 pipeline, publish `odom -> velodyne` by setting
`lidar_odom_frame:=odom` and `invert_odom_tf:=false`.

```bash
cd /workspace/farm_ws/pointcloud_process_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 launch kiss_icp odometry.launch.py \
  topic:=/velodyne_points \
  lidar_odom_frame:=odom \
  invert_odom_tf:=false \
  visualize:=true
```

Check:

```bash
ros2 topic list | grep -E "kiss|odom|tf|cloud"
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo odom velodyne
```

## Terminal 3: world cloud

KISS-ICP does not publish a complete registered world cloud by default, so use
`world_cloud_tools`.

```bash
cd /workspace/farm_ws/pointcloud_process_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 launch world_cloud_tools world_cloud_republisher.launch.py \
  input_cloud:=/velodyne_points \
  output_cloud:=/cloud_registered \
  target_frame:=odom
```

Check:

```bash
ros2 topic hz /cloud_registered
ros2 topic echo /cloud_registered --once | grep frame_id
```

## Terminal 4: ROG-Map + RViz2

```bash
cd /workspace/farm_ws/pointcloud_process_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 launch rog_map_vlp16_bringup rog_map_vlp16.launch.py \
  cloud_topic:=/cloud_registered \
  odom_topic:=/kiss/odometry \
  frame_id:=odom \
  rviz:=true
```

## Record bag

```bash
mkdir -p /workspace/farm_ws/bags

ros2 bag record \
  /velodyne_points \
  /cloud_registered \
  /kiss/odometry \
  /tf \
  /tf_static \
  -o /workspace/farm_ws/bags/vlp16_rog_map_$(date +%Y%m%d_%H%M%S)
```

## Verification

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /cloud_registered
ros2 topic list | grep -E "rog|map|esdf|occ|inflate"
ros2 run tf2_ros tf2_echo odom velodyne
```

RViz2 should use `Fixed Frame = odom` and show TF, `/velodyne_points`,
`/cloud_registered`, `/rog_map/occ`, `/rog_map/inf_occ`, `/rog_map/esdf`, and
`/rog_map/map_bound`.

## Notes

ROG-Map ROS2 in SUPER is a library, not a standalone executable. This package
adds the thin `rog_map_vlp16_node` wrapper and leaves SUPER's mapping logic
unchanged. One small SUPER header patch makes ROS2 visualization headers use the
configured `rog_map/visualization/frame_id` instead of hardcoded `world`.
