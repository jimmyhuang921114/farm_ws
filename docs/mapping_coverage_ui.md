# GLIM mapping coverage preview

This integration consumes GLIM **mapping odometry**. It does not implement or
claim pure localization.

## ROS interfaces

`preview_tools/coverage_analyzer` subscribes:

- `/glim_ros/odom` (`nav_msgs/msg/Odometry`, sensor-data QoS)

It publishes:

- `/collection/trajectory` (`nav_msgs/msg/Path`, transient local)
- `/coverage/markers` (`visualization_msgs/msg/MarkerArray`, transient local)
- `/coverage/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)
- `/coverage/status_json` (`std_msgs/msg/String`, transient local)
- `/coverage/reset` (`std_srvs/srv/Trigger`)

The reset service clears only the in-memory trajectory and visited-cell
preview. It does not reset GLIM, alter mapping output, or modify session files.

The analyzer defaults to 0.5 m XY cells and accepts a new preview pose after
0.05 m translation. Both values are ROS parameters. Path and marker sizes are
bounded to avoid unbounded UI messages.

## Meaning and limits

Visited area is calculated as:

```text
unique visited XY cells × cell_size_m²
```

It is a path coverage estimate. It does not account for LiDAR field of view,
occlusion, traversability, map correctness, loop-closure quality, or whether
the whole environment was observed.

`/coverage/status_json` uses schema version 1 and contains:

- `mode: "mapping"`
- `pure_localization: false`
- `state`: `WAITING`, `MAPPING`, or `STALE`
- odometry topic, frame, and message age
- accepted pose and bounded path-pose counts
- visited cells, cell size, estimated area, and travelled distance
- rejected non-finite and non-increasing-stamp counters

The RQT panel consumes this JSON directly. A Web UI or rosbridge adapter can
consume the same topic without duplicating coverage calculations. No native
Web backend/frontend is present in this repository, so an HTTP service is not
claimed or started.

## Launch

Build the custom UI packages inside the container:

```bash
cd /workspace/farm_ws
source /opt/ros/humble/setup.bash
./scripts/build_ui.sh
source install/setup.bash
```

Launch collection UI with the analyzer:

```bash
ros2 launch collection_bringup collection_full.launch.py \
  start_coverage:=true \
  mapping_odom_topic:=/glim_ros/odom
```

The Livox one-command workflow starts the analyzer after GLIM odometry and map
publishers have been validated:

```bash
./scripts/run_livox_glim_ui.sh
```

For RViz, use `rviz/collection.rviz` for the VLP-16 path or
`rviz/livox_glim_ui.rviz` for the MID-360 path. Both use `map` as the fixed
frame and display mapping odometry, trajectory, map/registered points, and the
approximate visited cells.
