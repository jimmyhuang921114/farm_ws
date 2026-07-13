# FAST_LIO Audit

## 1. 執行環境

- 宿主機工作區: `/home/jimmy/farm_ws`
- FAST_LIO 宿主機路徑: `/home/jimmy/farm_ws/pointcloud_process_ws/src/FAST_LIO`
- Docker 工作區: `/workspace/farm_ws`
- 選定 container: `farm_ws_gpu_dev`
- Image: `farm_ai:latest`
- Mount mapping: 未找到精確 `Source=/home/jimmy/farm_ws, Destination=/workspace/farm_ws` 的 mount；`farm_ws_gpu_dev` 與 `farm_ai` 都是 `/home/jimmy -> /workspace`，因此目前工作區實際對應為 `/home/jimmy/farm_ws -> /workspace/farm_ws`。
- 選擇原因: `farm_ws_gpu_dev` 與 `farm_ai` 都有 `/opt/ros/humble/setup.bash` 且可見 `/workspace/farm_ws/pointcloud_process_ws/src/FAST_LIO`；優先選用名稱明確指向工作區開發的 `farm_ws_gpu_dev`。
- ROS distro: `humble`
- ROS package prefix: `/workspace/farm_ws/pointcloud_process_ws/install/fast_lio`
- 其他觀察: `/home/jimmy/farm_ws/pointcloud_process_ws/255.255.255.255.2368:` 存在；依要求未處理。

## 2. Repository 身分

- Remote: `https://github.com/hku-mars/FAST_LIO.git`
- Branch: `ROS2`
- Commit: `a4743b095409588842a5b30ddfa27e29d2f99164`
- Last commit: `a4743b0 Merge pull request #381 from mfassler/ROS2`
- ROS 版本: ROS 2，使用 `rclcpp` 與 `ament_cmake`
- 身分判斷: 官方 `hku-mars/FAST_LIO` repository 的 ROS2 branch，內容為 FAST-LIO2 ROS2 port/fork。README 也標示 ROS2 fork maintainer 與 ROS >= Foxy/Humble。
- 工作目錄狀態: 有既有未提交變更，未由本次修改。
  - `M CMakeLists.txt`
  - `?? CMakeLists.txt.bak_cpp17`
  - `?? config/vlp16_fdi.yaml`
  - `?? launch/vlp16_fdi_mapping.launch.py`
- Submodule: `include/ikd-Tree` -> `https://github.com/hku-mars/ikd-Tree.git`, branch `fast_lio`, commit `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`

## 3. Package 結構

- Package name: `fast_lio`
- Version: `0.0.0`
- Build type: `ament_cmake`
- Main executable: `fastlio_mapping`
- Node name in code: `laser_mapping`
- Launch files:
  - `launch/mapping.launch.py`
  - `launch/vlp16_fdi_mapping.launch.py`
  - `launch/gdb_debug_example.launch`
- Config files:
  - `config/avia.yaml`
  - `config/horizon.yaml`
  - `config/mid360.yaml`
  - `config/ouster64.yaml`
  - `config/velodyne.yaml`
  - `config/vlp16_fdi.yaml`
- RViz configs:
  - `rviz/fastlio.rviz`
  - `rviz_cfg/loam_livox.rviz`
- Custom msg: `msg/Pose6D.msg`
- Main dependencies: `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `visualization_msgs`, `pcl_ros`, `pcl_conversions`, `livox_ros_driver2`, `Eigen3`, `PCL`, `OpenMP`, `PythonLibs`, `ikd-Tree`

## 4. VLP-16 支援狀態

結果: 修改 config 後可使用。

理由:
- 程式碼有 Velodyne 專用分支: `preprocess.lidar_type: 2` 對應 `VELO16`，會訂閱 `sensor_msgs/msg/PointCloud2`。
- 本地 `config/vlp16_fdi.yaml` 已設定 `lid_topic: /velodyne_points`, `lidar_type: 2`, `scan_line: 16`, `scan_rate: 10`。
- 目前 ROS graph 中 `/velodyne_points` 存在，型別為 `sensor_msgs/msg/PointCloud2`，publisher 是 `velodyne_transform_node`。
- 現場 `/velodyne_points` fields 已確認為 `x`, `y`, `z`, `intensity`, `ring`, `time`，其中 `time` datatype `7` 為 FLOAT32，符合 FAST_LIO Velodyne point struct。
- 仍需調整/確認: `vlp16_fdi.yaml` 的 `timestamp_unit: 0` 假設 `time` 是秒；目前點雲欄位名稱與型別符合，但未驗證 `time` 數值單位是否真為秒。

## 5. IMU 支援狀態

- FAST_LIO config topic:
  - `config/velodyne.yaml`: `/imu/data`
  - `config/vlp16_fdi.yaml`: `/imu`
- fdilink/vlp16 bringup 實際預設 topic: `/imu`
- 目前 ROS graph: `/imu` 存在，型別 `sensor_msgs/msg/Imu`，publisher 是 `fdilink_ahrs_driver`
- 目前 live `/imu` header frame: `gyro_link`
- 本地 fdilink launch 預設 frame: `imu_link`，但 `ahrs_driver.cpp` 參數預設是 `gyro_link`；目前 runtime 是 `gyro_link`。
- FAST_LIO 使用欄位: 已由程式碼確認只讀 `angular_velocity` 與 `linear_acceleration`，不讀 `orientation`，不讀三組 covariance。
- 單位: fdilink data struct 註解為 gyro `rad/s`、accelerometer `m/s^2`；FAST_LIO 以 `G_m_s2=9.81` 初始化/正規化加速度，要求加速度包含重力。
- 頻率: 本地 FAST_LIO 程式碼與 README 未明確確認建議 IMU 頻率。
- Covariance: 目前 live `/imu` 三組 covariance 皆為 0；FAST_LIO 不讀它們，因此 orientation covariance 0 或 0.10 都不影響 FAST_LIO 演算法。
- ENU/NED: fdilink `device_type_ == 1` 會對 quaternion、gyro、acc 做符號/姿態轉換；是否完全符合 FAST_LIO 需要以靜止重力方向與實測運動驗證，目前只能確認 fdilink 有做 ROS 標準座標轉換意圖。

## 6. LiDAR 點雲欄位需求

| Field | 必須 | 本地 FAST_LIO 使用方式 | VLP-16 目前狀態 |
| ----- | -: | ---------------- | ----------- |
| `x` | 是 | PCL Velodyne point struct 讀取座標 | 存在，FLOAT32 |
| `y` | 是 | PCL Velodyne point struct 讀取座標 | 存在，FLOAT32 |
| `z` | 是 | PCL Velodyne point struct 讀取座標 | 存在，FLOAT32 |
| `intensity` | 是 | 複製到 FAST_LIO `PointType.intensity` | 存在，FLOAT32 |
| `ring` | 是 | 用於 scan line 分層與無 time 時估算 offset time | 存在，UINT16 |
| `time` | 強烈需要 | `time * time_unit_scale` 存入 `curvature`，單位轉成 ms；deskew 依賴每點 offset time | 存在，FLOAT32 |
| `timestamp` | 否 | Velodyne 分支未讀取此名稱 | 未使用 |
| `t` | 否 | Ouster 分支才讀 `t` | 未使用 |

Velodyne 分支只接受欄位名稱 `time`，型別為 `float`。若 `time` 缺失，`pcl::fromROSMsg` 會印出 field mismatch 類警告，`time` 可能變成 0；FAST_LIO 會改用 `ring` 與 yaw/scan_rate 估算每點 offset time，不是完全停用 deskew，但精度依賴 scan model 與點排序。

## 7. Topic 表

| 方向 | Topic | Message type | Frame |
| -- | ----- | ------------ | ----- |
| Input | `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | live 未 echo header；bringup 預設 `velodyne` |
| Input | `/imu` | `sensor_msgs/msg/Imu` | live `gyro_link`; source launch 預設 `imu_link` |
| Input alternative | `/imu/data` | `sensor_msgs/msg/Imu` | 只在 `config/velodyne.yaml` 預設 |
| Output | `/Odometry` | `nav_msgs/msg/Odometry` | header `camera_init`, child `body` |
| Output | `/path` | `nav_msgs/msg/Path` | `camera_init` |
| Output | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | `camera_init` |
| Output | `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | `body` |
| Output | `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | `camera_init` |
| Output | `/Laser_map` | `sensor_msgs/msg/PointCloud2` | `camera_init` |
| Output | TF | `tf2` | `camera_init` -> `body` |
| File | `FAST_LIO/PCD/scans.pcd` or `map_file_path` | PCD | global/map points |
| Debug file | `FAST_LIO/Log/*.txt`, `fast_lio_time_log.csv` | text/csv | N/A |

FAST_LIO QoS:
- LiDAR PointCloud2 subscription uses `rclcpp::SensorDataQoS()`.
- IMU subscription uses queue depth `10`.
- Publishers use queue depth `20`.

## 8. TF 架構

FAST_LIO:
- 發布 TF: 是。
- Parent: hard-coded `camera_init`
- Child: hard-coded `body`
- Odometry header/child: `camera_init` / `body`
- Output cloud frames: world/map clouds are `camera_init`; body cloud is `body`
- Frame 名稱不可由 YAML 參數修改。

KISS-ICP:
- `kiss_icp_vlp16.launch.py` 預設 topic `/velodyne_points`, `base_frame=velodyne`, `lidar_odom_frame=kiss_odom`, `publish_odom_tf=true`, `invert_odom_tf=true`。
- 在 `invert_odom_tf=true` 時發布 TF `velodyne` -> `kiss_odom`；若 false 則 `kiss_odom` -> `velodyne`。
- Odometry topic 是 `kiss/odometry`，header `lidar_odom_frame`，child 是 `base_frame` 或點雲 frame。

可能衝突:
- FAST_LIO 發布 `camera_init -> body`，KISS-ICP 預設發布 `velodyne -> kiss_odom`。名稱不完全相同，但若另有 static TF `body/base_link -> velodyne`，同時使用兩套 odometry TF 會造成 TF tree 語意混亂。
- 若將 KISS-ICP 改成 `lidar_odom_frame=camera_init` 或 `base_frame=body`，可能造成同一 child frame 有多個 parent。
- 建議同時比較時關閉其中一套 odometry TF，或讓兩套使用完全不同 odom frame。RViz Fixed Frame 可選 FAST_LIO 的 `camera_init` 或 KISS-ICP 的 `kiss_odom`，不要混用。

建議 TF:

```text
camera_init
└── body
    └── velodyne
    └── imu_link / gyro_link
```

FAST_LIO 內部外參用於演算法把 LiDAR point 轉到 IMU body；ROS static TF 只用於 RViz/TF tree 和其他節點理解感測器安裝關係。兩者可以都需要，但角色不同；內部外參不會自動發布成 `body -> velodyne` static TF。

## 9. 外參

- 參數位置:
  - `config/velodyne.yaml`: `mapping.extrinsic_T`, `mapping.extrinsic_R`
  - `config/vlp16_fdi.yaml`: `mapping.extrinsic_T`, `mapping.extrinsic_R`
- 程式碼讀取: `laserMapping.cpp` 將 YAML 讀入 `Lidar_T_wrt_IMU`, `Lidar_R_wrt_IMU`，再傳入 `ImuProcess::set_extrinsic`。
- README 定義: LiDAR pose in IMU body frame，也就是 LiDAR 相對 IMU body 的位置與旋轉。
- 程式碼方向確認:
  - `pointBodyToWorld`: `p_body = offset_R_L_I * p_lidar + offset_T_L_I`
  - `RGBpointBodyLidarToIMU`: 同樣把 LiDAR point 轉到 IMU/body
  - 因此 `extrinsic_R/T` 是 LiDAR frame 到 IMU/body frame 的轉換。
- Translation 單位: meter。
- Rotation 排列: YAML 9 個數讀成 3x3 matrix，row-major 寫法。
- Online estimation: `mapping.extrinsic_est_en` 支援；`vlp16_fdi.yaml` 設為 `false`。
- 目前 `vlp16_fdi.yaml` 數值:
  - `extrinsic_T: [0.0, 0.0, 0.07]`
  - `extrinsic_R: [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0]`
- 是否可信: 尚未可信。這是既有本地 config，未看到校正來源；需要量測或用 LI-Init/實測校正。
- 注意: `vlp16_imu_bringup` static TF 參數名是 `lidar_to_imu_*`，發布方向是 `velodyne -> imu_frame`；它與 FAST_LIO 內部外參方向語意接近，但 static TF 的 RPY 目前預設全 0，和 `vlp16_fdi.yaml` 的 rotation matrix 不一致。

## 10. 現有 Config 適用性

最接近 VLP-16 的 config: `config/vlp16_fdi.yaml`。

目前值:
- `lid_topic: /velodyne_points`
- `imu_topic: /imu`
- `lidar_type: 2`
- `scan_line: 16`
- `scan_rate: 10`
- `timestamp_unit: 0`
- `blind: 0.3`
- `det_range: 30.0`
- `time_sync_en: false`
- `time_offset_lidar_to_imu: 0.0`
- `pcd_save_en: false`

仍需修改/確認:
- 確認 `/velodyne_points.time` 單位是否為秒；若不是，修正 `timestamp_unit`。
- 統一 IMU frame: live 是 `gyro_link`，bringup/FAST_LIO 建議架構多處使用 `imu_link`。
- 外參 rotation/translation 需要校正，不應只沿用未驗證值。
- 確認 LiDAR 與 IMU 時鐘同步；`time_sync_en=false` 表示依賴外部同步或 header time 已一致。

## 11. Build 狀態

- `colcon list` 在容器內可辨識: `fast_lio src/FAST_LIO (ros.ament_cmake)`
- `ros2 pkg list` 已包含: `fast_lio`
- 已安裝: `/workspace/farm_ws/pointcloud_process_ws/install/fast_lio`
- Build directory 存在: `/home/jimmy/farm_ws/pointcloud_process_ws/build/fast_lio`
- 最近一次 FAST_LIO build log: `/home/jimmy/farm_ws/pointcloud_process_ws/log/build_2026-07-09_07-51-53/fast_lio/`
- 該 log 看到 CMake/PCL/Boost/Python policy warnings 與 Boost bind deprecated note，未在已檢查片段看到 FAST_LIO 編譯失敗。
- `log/latest_build` 指向 `build_2026-07-10_13-00-37`，該次只含 `kiss_icp`，不是 FAST_LIO 最近 build。
- Dependency: `livox_ros_driver2` 已從 workspace install 被 CMake 找到。
- 可能 stale: 目前 source 有未提交 CMake/config/launch 變更；install 是否完全對應目前 source 需重新 build 才能保證，但本次依要求未 build。

## 12. 啟動指令草案

### Docker 內編譯草案

```bash
docker exec -it farm_ws_gpu_dev bash -lc '
set +u
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws/pointcloud_process_ws
colcon build --symlink-install --packages-select fast_lio
source install/setup.bash
ros2 pkg prefix fast_lio
'
```

### Docker 內啟動 VLP-16 與 IMU 草案

```bash
docker exec -it farm_ws_gpu_dev bash -lc '
set +u
source /opt/ros/humble/setup.bash
source /workspace/farm_ws/pointcloud_process_ws/install/setup.bash
ros2 launch vlp16_imu_bringup vlp16_imu_bringup.launch.py \
  lidar_frame:=velodyne \
  imu_frame:=imu_link \
  imu_topic:=/imu \
  organize_cloud:=false
'
```

### Docker 內啟動 FAST_LIO 草案

使用 source tree 絕對 config 的本地 VLP-16/FDI launch:

```bash
docker exec -it farm_ws_gpu_dev bash -lc '
set +u
source /opt/ros/humble/setup.bash
source /workspace/farm_ws/pointcloud_process_ws/install/setup.bash
ros2 launch fast_lio vlp16_fdi_mapping.launch.py \
  config_file:=/workspace/farm_ws/pointcloud_process_ws/src/FAST_LIO/config/vlp16_fdi.yaml
'
```

或使用通用 launch:

```bash
docker exec -it farm_ws_gpu_dev bash -lc '
set +u
source /opt/ros/humble/setup.bash
source /workspace/farm_ws/pointcloud_process_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/workspace/farm_ws/pointcloud_process_ws/src/FAST_LIO/config \
  config_file:=vlp16_fdi.yaml \
  rviz:=false
'
```

### RViz2 草案

```bash
docker exec -it farm_ws_gpu_dev bash -lc '
set +u
source /opt/ros/humble/setup.bash
source /workspace/farm_ws/pointcloud_process_ws/install/setup.bash
rviz2 -d /workspace/farm_ws/pointcloud_process_ws/src/FAST_LIO/rviz/fastlio.rviz
'
```

RViz Fixed Frame 建議先用 `camera_init` 檢查 FAST_LIO；若只看原始 VLP-16，使用 `velodyne`。

## 13. 阻塞問題

Critical:
- LiDAR-IMU 外參尚未可信；FAST_LIO 內部 `extrinsic_R/T` 與 bringup static TF 預設不一致。
- 時間同步尚未驗證；FAST_LIO README 明確要求 IMU/LiDAR synchronized，`time_sync_en=false` 代表目前仰賴外部或 driver timestamp。

High:
- `/velodyne_points.time` 欄位存在但單位尚未確認；`vlp16_fdi.yaml timestamp_unit=0` 必須與實際單位一致。
- IMU frame live 是 `gyro_link`，但 bringup launch 預設與建議 TF 使用 `imu_link`，需要統一。

Medium:
- `FAST_LIO` source 有未提交變更，install 可能不是目前 source 的完全重建結果。
- 同時啟動 FAST_LIO 與 KISS-ICP 時需避免兩套 odometry TF 混入同一 TF tree。

Low:
- Build log 有多個 CMake policy warnings，不是目前阻塞。
- `pointcloud_process_ws/255.255.255.255.2368:` 異常目錄/檔名存在，未影響本次 FAST_LIO audit。

## 14. 下一步建議

1. 確認 `/velodyne_points` 的 `time` 數值單位，必要時修正 `config/vlp16_fdi.yaml` 的 `timestamp_unit`。
2. 統一 IMU topic/frame，建議 `/imu` 與單一 frame name，例如 `imu_link` 或明確接受 `gyro_link`。
3. 確認 LiDAR 與 IMU header timestamp 是否同時基準；先不要依賴 `time_sync_en` 軟同步。
4. 校正 LiDAR 到 IMU/body 的 `extrinsic_T` 與 `extrinsic_R`，並同步更新 ROS static TF 的語意。
5. 重新編譯 `fast_lio`，確認 install 對應目前 source/config/launch。
6. 靜止測試: 檢查 IMU 重力方向、FAST_LIO 初始化、點雲是否穩定。
7. 低速移動測試: 檢查 `/Odometry`、`/cloud_registered`、漂移與地圖扭曲。
8. RViz 與 TF 驗證: 固定 `camera_init`，檢查 `camera_init -> body`、`body -> velodyne/imu` 是否一致；若同時跑 KISS-ICP，先關閉其中一套 odometry TF 或分離 odom frame。
