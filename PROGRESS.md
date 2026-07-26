# Progress — 2026-07-13

## Physical sensor continuation — authoritative current state

### Selected repositories and packages

- GLIM `d0eeebead1ab8240edf3645682ec12d79fbfa70a`; glim_ros2 `a62811dc3ab73076f4a43fc21005f96cd712903c`.
- FDILINK ROS 2 driver `Atr-0/fdilink_ahrs_ROS2` at `5ea50eae17952f27f606f58ac1b2bfe807e0452c`. Selected after inspecting native ament build, launch/config, parameters, serial parser, and published topics. Humble apt has no FDILINK package.
- Ament serial dependency `RoverRobotics-forks/serial-ros2` at `ae46504ae7d4a199ea9bba0e73a6f083bf172f80`. The initially inspected wjwwood `ros2` branch was catkin and was not used.
- Velodyne binaries 2.5.1: meta, driver, msgs, pointcloud. RealSense camera/description 4.58.2 and librealsense 2.58.2. Exact dpkg revisions are in `VERSIONS.txt`.

### Docker hardware mapping

- Image `farm_ws_glim_ui:humble` rebuilt successfully and container `farm_ws_glim_ui_dev` recreated successfully.
- `docker/run.sh` now uses `--network host`, conditionally maps `/dev/ttyUSB0`, adds actual host dialout/video GIDs, bind-mounts `/dev/bus/usb`, adds cgroup rule `c 189:* rmw`, and maps only Intel `8086` video nodes. No `--privileged`.
- X11 settings preserved. Host RTX 2070 / NVIDIA driver 580.159.03 works with host `nvidia-smi`, but Docker NVIDIA runtime discovery fails; container runs sensor/GLIM CPU validation without `--gpus all` and has no `nvidia-smi`.

### Host hardware and setup results

- Expected Velodyne NIC `enx00e04c521538` is absent; expected host IP `192.168.1.10/24` is not configured. Current host interfaces are `enp5s0=172.31.1.140/24` and `wlo1=192.168.0.148/24`. Ping to `192.168.1.201` failed. Normal-user tcpdump is permission denied and `sudo -n` requires a password.
- Despite failed expected-network preflight, the ROS driver receives real UDP 2368 scans filtered to source `192.168.1.201`. Actual ingress interface remains to be identified with sudo tcpdump; network setup is not claimed complete.
- FDILINK `/dev/ttyUSB0`: CP2102 `10c4:ea60`, serial `0001`, `root:dialout 0660`. Host login user is not yet in dialout; printed exact command is `sudo usermod -aG dialout jimmy`. Container developer user has host dialout GID and read/write succeeds. Verified udev template and install workflow created.
- RealSense D435i: USB `8086:0b3a`, serial `908212070822`, firmware `5.13.0.50`; ROS driver and V4L nodes enumerate it. Physical topology reports USB 2.1 / 480M.
- Setup/check scripts created: Velodyne network setup/read-only check, sensor permission/device checks, and verified FDILINK udev installer. No ROS launch action runs sudo/chmod/network mutation.

### Dependency and build results

- `rosdep install --from-paths src --ignore-src -r -y` satisfied sensor dependencies. Four existing ament-python packages report unresolved `ament_python` because Humble rosdep has no key; `-r` continues and the required build support is installed.
- `colcon build --symlink-install --event-handlers console_direct+ --packages-up-to farm_sensor_bringup` passed; the final audit completed 10 packages. Final focused rebuild of `fdilink_ahrs` and `farm_sensor_bringup` also passed. Remaining warnings are unused variables in upstream FDILINK parser code and an upstream Humble scoped-header notice from `glim_ros`.
- Unified-launch bug found and fixed: all included drivers originally declared generic `config_file`, causing FDILINK to receive `velodyne.yaml`. Arguments are now isolated as `velodyne_config_file`, `fdilink_config_file`, and `realsense_config_file`.
- FDILINK source now retries serial open five times at 1-second intervals and reports port/attempt/error; persistent absence exits nonzero without fabricated messages.

### Independent runtime results

- Velodyne nodes: `/velodyne_driver_node`, `/velodyne_transform_node`.
- Velodyne topics: `/velodyne_packets` (`VelodyneScan`) and `/velodyne_points` (`PointCloud2`), both about 9.915 Hz. Each scan has 76 nonempty packets. PointCloud frame is `velodyne`, 1,824×16 = 29,184 points with `x,y,z,intensity,ring,time`, progressing timestamps.
- FDILINK node: `/ahrs_bringup`. Actual topics: `/imu`, `/mag_pose_2d`, `/euler_angles`, `/magnetic`, `/gps/fix`, `/system_speed`, `/NED_odometry`. `/imu` is 100.0 Hz, frame `imu_link`, and orientation/angular velocity/linear acceleration are finite. Orientation accuracy is not inferred.
- RealSense node `/camera` finds the device. The required comma profile syntax was corrected to installed-driver syntax. Because USB 2.1 exposes only 1280×720×15, config uses that profile; target 1280×720×30 is unavailable. The profile opens as RGB8 but no frames arrive; V4L2 reports a timeout every five seconds. Root and 424×240×6 tests fail identically, excluding container-user permissions. Camera topics are advertised but have zero messages.

### Combined, health, TF, and RViz

- LiDAR+IMU combined uptime exceeded 60 seconds. Concurrent rates: packets 9.915 Hz, points converged to 9.720 Hz during one loaded window, IMU 100.005 Hz. Final unified launch gives IMU 100.003 Hz.
- Raw TF report: `base_link -> velodyne -> imu_link`; identity base/LiDAR and approximate 0,0,0.07 + roll 180°/yaw 90° LiDAR/IMU transforms are explicitly uncalibrated. Camera external TF is absent and semantic projection refuses unconfigured extrinsics. `frames_2026-07-13_12.39.29.{gv,pdf}` records the raw tree.
- Health monitor validates payloads. Observed OK: LiDAR Points (29,184, `velodyne`), IMU (finite, `imu_link`, 100 Hz), Disk (about 806 GB free). Observed WAITING: Camera, CameraInfo, GLIM TF while GLIM off. After GLIM starts, GLIM TF becomes OK at 10 Hz. No sensor is marked OK from process existence alone.
- RViz config loads with software OpenGL 4.5 and survives the 15-second visualization window; no display/plugin/topic configuration error occurred. Timeout shutdown throws `std::system_error`; human visual confirmation remains pending.

### MCAP result

- Path: `mapping_sessions/manual_sensor_test/sensors`.
- Storage: MCAP; duration 19.722657510 s; size 138.1 MiB; 2,367 messages.
- Counts: `/imu` 1,973; `/velodyne_packets` 196; `/velodyne_points` 196; `/tf_static` 2.
- `/tf` had no messages with GLIM disabled, so it is absent from bag metadata. Camera topics were excluded because no valid frames exist. Initial camera policy is raw RGB for fidelity; compressed is the storage-saving alternative, not recorded simultaneously.

### GLIM sensor readiness

- Source inspection confirms `T_lidar_imu` maps IMU coordinates to LiDAR coordinates. Conflicting duplicate and unrelated camera calibration values were removed. Approximate `T_lidar_imu` is `[0,0,0.07,0.7071068,0.7071068,0,0]` and remains uncalibrated.
- ROS inputs are `/velodyne_points` and `/imu`; actual frame IDs are `velodyne` and `imu_link`. CPU odometry/sub/global modules are selected because Docker GPU runtime is absent. GLIM still prints nonfatal CUDA-driver warnings from CUDA-enabled shared libraries.
- GLIM executable `/glim_ros` runs, publishes odometry/aligned/map topics, and produces about 10 Hz TF. Initial disconnected-tree error was fixed by setting GLIM `base_frame_id=base_link` and `publish_imu2lidar=false`. Verified chain: `map -> odom -> base_link -> velodyne -> imu_link`; `tf2_echo map velodyne` succeeds.
- `sensors_with_ui.launch.py start_glim:=true` now starts the verified executable/config. Default remains false. `ui_only` now correctly suppresses physical driver launch.

### Remaining calibration and blockers

- Connect D435i through a true USB 3.x port/cable, verify 1280×720×30 RGB messages and CameraInfo, then repeat UI, RViz, and MCAP with raw RGB.
- Identify actual VLP UDP ingress interface with `sudo tcpdump -ni any udp port 2368`; configure/verify intended `enx00e04c521538` and `192.168.1.10/24` if that adapter is required.
- Add `jimmy` to host dialout and re-login; install/reload the verified FDILINK udev rule.
- Calibrate base-to-LiDAR, LiDAR-to-IMU, and especially LiDAR-to-camera extrinsics. Camera transform remains `REQUIRES_CALIBRATION`.
- Restore Docker NVIDIA runtime for GPU GLIM; current CPU readiness is functional but not the intended performance path.
- Human-confirm RViz/rqt rendered sensor visuals. Repeat a complete MCAP after camera recovery; current partial MCAP is valid for LiDAR/IMU only.

## Historical pre-container audit (superseded by the results above)

### Source

- 初始 `src/glim`、`src/glim_ros2` 都是空目錄，但外層 repository 有正確 gitlink；不是獨立 repository，也沒有既有 source 修改。
- 使用 `git submodule update --init --recursive` 完成 clone；兩者內部無待初始化 submodule。
- GLIM: `d0eeebead1ab8240edf3645682ec12d79fbfa70a`。
- glim_ros2: `a62811dc3ab73076f4a43fc21005f96cd712903c`。
- colcon 真實 package：`glim`、`glim_ros`。固定版本無 launch file；CMake executables 為 `glim_rosnode`、`glim_rosbag`、`validator_node`、`offline_viewer`、`map_editor`。

## Docker / dependencies

- 已建立 CUDA 12.6.3 + Ubuntu 22.04 + ROS 2 Humble Desktop Dockerfile、koide3 PPA、MCAP、rqt/RViz、Cyclone DDS、非 root UID/GID 與 X11 scripts。
- Image 目標：`farm_ws_glim_ui:humble`；container：`farm_ws_glim_ui_dev`。
- Docker build **未完成**：host user 無 `/var/run/docker.sock` 權限；`sudo docker` 又需要互動密碼。這是外部權限 blocker，不是 Dockerfile build 結果。
- 目標 CUDA：12.6；目標 gtsam_points：`libgtsam-points-cuda12.6-dev`，因 image 未 build 尚無實際 dpkg version。
- Host 僅有 CUDA 11.5，`nvidia-smi` 無法連 driver，且沒有 GTSAM/gtsam_points/Iridescence。
- rosdep：container 未執行（image 未 build）；沒有使用 blanket skip-key。

## Builds / runtime

- UI packages 六個全部在 host ROS Humble build 成功。
- 初次錯誤：manifest `jimmy@localhost` 不被 catkin_pkg 接受；改為有效 email。
- 第二次錯誤：bringup 在 dependencies 尚未 build 時被單獨選取；依正確拓撲先 interfaces、再 consumers、最後 bringup。
- GLIM CPU：失敗於第一個 configure dependency `GTSAMConfig.cmake` not found；完整 log `logs/glim_cpu_build.txt`。
- GLIM CUDA：失敗於同一 GTSAM 前置 dependency，尚未進入 CUDA configure；log `logs/glim_cuda_build.txt`。
- rqt：plugin discovery 成功，offscreen 實際載入 `collection_rqt_panel.panel.CollectionPanel` 並存活至 timeout。
- RViz2：`rviz/collection.rviz` offscreen 載入並存活至 timeout；真實 X11 畫面未驗證。
- UI-only：成功。Session `mapping_sessions/20260713_042748_ui_test`，3 秒倒數、10 秒計時、自動 COMPLETED；IMU 抽查為 WAITING/0 Hz/age -1；YAML 正確；events 正確；沒有 `.mcap`。
- runtime 修正：`ROS_LOG_DIR` 改到 writable workspace；`velodyne_msgs` 缺少時 monitor 不 crash而維持 packets WAITING，container rosdep 安裝後會訂閱。

## 尚未測試

- Docker image/container、container rosdep、GLIM CPU/CUDA/Viewer build、GLIM shared-library/executable/config smoke test。
- 實際 X11 GUI forwarding 與人工版面檢查。
- 真實 VLP-16、FDILINK IMU、RGB camera、TF、GLIM mapping 與 rosbag2 MCAP recording（UI-only 明確不製造假資料）。

## Continuation access audit — 2026-07-13

- `id`: `uid=1000(jimmy) gid=1000(jimmy) groups=1000(jimmy),65534(nogroup)`；當前 shell 沒有 gid 999。
- `groups`: `jimmy nogroup`。
- Group database：`docker:x:999:jimmy`，代表帳號設定已加入但 shell 尚未重新載入。
- Docker socket：`srw-rw---- nobody:nogroup /var/run/docker.sock`；一般執行與 managed external approval 都回覆 `permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`。
- `newgrp docker -c ...`：失敗，精確錯誤 `Cannot open audit interface - aborting.`。未使用互動 sudo。
- Docker image build：已執行 `./docker/build.sh`，但在送出 build context／執行第一層之前失敗：`ERROR: permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`；因此沒有 package layer 可診斷。完整輸出在 `logs/docker_build_continuation.txt`。
- Container/workspace mount/ROS Humble in container：未執行。
- NVIDIA：host `nvidia-smi` 回覆 `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver`。GPU runtime unavailable。
- CUDA/dependency versions in container：未取得；Dockerfile 目標是 CUDA 12.6.3 與 `libgtsam-points-cuda12.6-dev`，不可當成已安裝版本。
- rosdep in container：未執行，沒有 skip key。
- GLIM CUDA/CPU：本輪無法在 container 執行；先前 host 兩者都精確阻塞於 `GTSAMConfig.cmake` not found。
- GLIM executables/shared-library validation/smoke tests：因 GLIM 尚未 build，無可驗證 binary；repository 定義清單不能冒充 discovery 結果。
- UI container rebuild/UI-only rerun：daemon blocker，未執行；先前 host UI build/runtime 成功仍有效。
- X11：`DISPLAY=:0`、X0 socket 存在且為 `jimmy:jimmy`，但 `xhost` 回覆 `unable to open display ":0"`；真實視窗未開啟，offscreen 結果仍有效。
- ROS overlay 再確認：`collection_interfaces` 與 `collection_bringup` 可由 `ros2 pkg prefix` 發現；rqt plugin discovery 仍成功；真實 GLIM package names 仍為 `glim`、`glim_ros`。
- 解鎖條件：完整登出/登入（或可用的 host terminal 執行 `newgrp docker`），直到 `id` 顯示 docker gid 且 `docker ps` 成功；另需修復 host NVIDIA driver 才能宣稱 CUDA runtime。

## Continuation retry — 2026-07-13

- Required host commands were rerun. Exact identity remains `uid=1000(jimmy) gid=1000(jimmy) groups=1000(jimmy),65534(nogroup)` and `groups` remains `jimmy nogroup`; the current shell has not picked up Docker gid 999 even though `getent group docker` returns `docker:x:999:jimmy`. A full re-login or a working host-terminal `newgrp docker` is still required.
- `docker version` reports client `29.5.3`, API `1.54`, then fails at the daemon with `permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`. `docker ps` therefore cannot run. `docker info | grep -i runtime -A3` returns no runtime lines because `docker info` itself fails.
- `/var/run/docker.sock` is `srw-rw---- nobody nogroup`. The same daemon denial occurred both in the managed shell and in the approved external retry.
- `chmod +x docker/*.sh` completed. `docker/build.sh` was rerun and failed before sending the context or entering any Dockerfile layer: `ERROR: permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`. Image `farm_ws_glim_ui:humble` was not built; there is no failing apt/CMake layer to diagnose.
- Because the image does not exist, `docker/run.sh`, container inspection, dependency/version inspection, rosdep, GLIM CUDA/CPU container builds, executable discovery, `ldd`, GLIM smoke tests, and container UI retest are not currently possible. No rosdep keys were skipped and no success is claimed for any of these items.
- Host `nvcc --version` is CUDA toolkit `11.5`, build `V11.5.119`. Host runtime remains unavailable: `nvidia-smi` exits 9 with `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make sure that the latest NVIDIA driver is installed and running.` Toolkit presence is not recorded as GPU success.
- Actual source package names were reconfirmed with `colcon list`: `glim` and `glim_ros`. Repository-defined executable names remain `glim_rosnode`, `glim_rosbag`, `validator_node`, `offline_viewer`, and `map_editor`, but none can be reported as installed/discovered until GLIM builds.
- X11 cannot be honestly validated: `DISPLAY=:0` and X sockets exist, but `xhost +local:docker` returns `xhost: unable to open display ":0"`; additionally no container can start. Real GUI result is **not tested**.
- Previously completed host UI-only results remain unchanged and were not recreated: 3-second countdown, 10-second auto-completion, WAITING status, valid metadata, and no fake MCAP.
- Exact resume command in a newly logged-in host terminal: `cd /home/jimmy/farm_ws && id && docker ps && cd docker && ./build.sh`. Proceed to `./run.sh` only after that command builds the image successfully.

## Final continuation audit — 2026-07-13

- Re-ran `id`, `groups`, `docker version`, `docker ps`, `nvidia-smi`, and `docker info | grep -i runtime -A3`. Docker client is 29.5.3, but daemon access still fails with exact error `permission denied while trying to connect to the Docker API at unix:///var/run/docker.sock`; `docker ps` and runtime inspection therefore cannot run. Current identity is `uid=1000(jimmy) gid=999(docker) groups=999(docker),65534(nogroup)` and socket is `nobody:docker` mode `0660`; a full re-login or working host-terminal `newgrp docker` remains required despite the group appearing in this shell.
- Re-ran `nvidia-smi`: exact error is `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make sure that the latest NVIDIA driver is installed and running.` No NVIDIA runtime success is claimed.
- Re-ran `./build.sh`; it fails before build context/layers with the same Docker API permission error. No image/container exists to inspect, so all dependent container steps (rosdep, dependency versions, CUDA/CPU builds, executable discovery, ldd, smoke tests, container UI test) remain blocked and were not falsely reported.
- Corrected README's stale claims of successful image/container/GLIM/X11 validation to match the exact current blockers. Host UI-only/offscreen results remain unchanged and were not recreated.
- Next exact command after opening a newly logged-in host terminal: `cd /home/jimmy/farm_ws && id && docker ps && cd docker && ./build.sh && ./run.sh`.
