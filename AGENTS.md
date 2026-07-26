# AGENTS.md

## Scope

This ROS 2 Humble workspace integrates farm sensor bringup, collection health/UI/session metadata, and GLIM mapping. The active custom sensor path is Velodyne VLP-16 + FDILINK AHRS + optional RealSense. Livox and FAST-Calib are separate vendored paths.

## Ownership

Custom packages: `collection_bringup`, `collection_interfaces`, `collection_manager`, `collection_rqt_panel`, `farm_sensor_bringup`, and `sensor_health_monitor`.

Treat `glim`, `glim_ros2` (package `glim_ros`), `Livox-SDK2`, `ws_livox/src/livox_ros_driver2`, `FAST-Calib-ROS2` (package `fast_calib`), `fdilink_ahrs`, and `serial` as third-party/upstream. Do not edit them unless the task explicitly requires an upstream change.

## Safety and compatibility

- Inspect the owning `package.xml`, build metadata, launch/config files, and node source before changing code.
- Preserve package/executable names and ROS message/service definitions. Do not rename topics, services, actions, parameters, or TF frames without explicit instruction.
- Do not launch LiDAR, IMU, camera, calibration, GLIM, recording, or other real-hardware processes automatically.
- Do not change host network interfaces, serial permissions/groups/udev rules, firmware, or device configuration automatically. Setup scripts requiring root must be reviewed and run deliberately by an operator.
- Do not edit generated `build/`, `install/`, `log/`, `__pycache__/`, `src/Livox-SDK2/build/`, `src/ws_livox/{build,install,log}/`, recorded `mapping_sessions/`, calibration output, or generated frame diagrams.
- The manager is intentionally UI-only and does not create MCAP. Do not document or imply real recording unless implementation and verification explicitly change.
- Preserve unrelated working-tree changes; several components may be submodules or untracked local integrations.

## Build and test

Run inside the container path expected by launch/config files:

```bash
cd /workspace/farm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select <changed-package>
colcon test-result --verbose
```

Use `./scripts/build_glim.sh`, `build_ui.sh`, or `build_all.sh` when their package selection matches the task. Livox-SDK2 is a separate native prerequisite for `livox_ros_driver2`; never reuse nested generated workspace artifacts. `scripts/test_ui_only.sh` is a bounded integration test that starts ROS nodes and writes a test session, so run it only when appropriate.

## Verification and documentation

After changes, confirm package discovery and relevant installed executables, inspect `ros2 interface show` for changed interfaces, and statically check launch arguments/remappings/config paths. Run focused tests, then `git diff` and `git status --short`; verify no generated or unrelated files were changed.

Documentation must distinguish custom from third-party code, source from generated artifacts, confirmed behavior from runtime assumptions, and Velodyne from Livox paths. Use relative repository links, exact names from source, container paths for commands, and “Not confirmed from source” where evidence is absent. Hardware claims require explicit runtime evidence and should be dated when recorded.
