# Farm 感測器與 GLIM 工作區

這是以 ROS 2 Humble 建立的農場感測器工作區。目前建議使用的整合流程是：

```text
Livox MID-360 ─┬─ PointCloud2 ─→ 資料檢查 ─→ GLIM 建圖 ─→ RViz
               └─ IMU ─────────→ 座標轉換 ──┘
DECXIN Camera ───────────────────────────────────────→ RQT / 健康監控
Collection Manager ─→ 工作階段資訊、事件標記 ────────→ RQT
GLIM Odometry ──────→ 路徑與覆蓋率分析 ─────────────→ RQT / RViz
```

> 目前的 MID-360 與 IMU 外參只是尚未校正的預設值，不可視為正式校正結果。詳見
> [`config/livox_mid360/CALIBRATION_REQUIRED.md`](config/livox_mid360/CALIBRATION_REQUIRED.md)。

## 最快開啟方式

所有主要指令都以 Docker 容器內的 `/workspace/farm_ws` 為基準。主機端專案實際放在哪裡不影響容器內路徑。

### 第一次安裝

在主機執行：

```bash
git clone --recurse-submodules https://github.com/jimmyhuang921114/farm_ws.git
cd farm_ws

# 建立 Ubuntu 22.04、ROS 2 Humble、CUDA 與相關套件的映像
./docker/build.sh
```

如果專案已經 clone，但 `src/glim` 或 `src/glim_ros2` 是空目錄：

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

### 啟動硬體與介面

1. 將 MID-360 接到主機並開機。
2. 在主機設定 LiDAR 網路：

```bash
cd /path/to/farm_ws
sudo ./docker/setup_livox_network.sh
```

3. 開啟容器：

```bash
./docker/run.sh
```

4. 第一次進入容器時建置工作區：

```bash
cd /workspace/farm_ws
./scripts/build_all.sh
source install/setup.bash
```

建制LiDAR Workspace
```
cd livox_ros_driver2
source  /opt/ros/humble/setup.bash
./build.sh humble

#first time colcon build the workspace
colcon build \
  --symlink-install \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble
```


-------
```
4. 第一次進入容器時建置工作區：

cd /workspace/farm_ws/src/gtsam_points

mkdir -p build
cd build

cmake ..
make -j$(nproc)

sudo make install
sudo ldconfig

```


5. 啟動 MID-360、相機、GLIM、RQT 與 RViz：

```bash
./scripts/run_livox_glim_ui.sh
```

若要保存可重複使用的原始資料與 GLIM 結果，使用：

```bash
./scripts/run_livox_glim_ui.sh --record
```

啟動後在 Collection 面板取消 **UI only**，再執行 Preflight → Start。每次工作階段會保存於
`mapping_sessions/<時間>_<名稱>/rosbag/`，內容包含 MID-360 原始/有效點雲、IMU、TF、相機與 GLIM
輸出 topic；影像以 JPEG quality 85 保存，rosbag2 使用 Zstandard 檔案壓縮，GLIM 的 dump 與執行記錄則保存於
`data/runtime/`。這些資料不會提交到 GitHub。

看到 `Integrated stack is ready` 即代表主要節點已通過啟動檢查。程式會持續在前景執行；按 `Ctrl+C` 可依序停止本次啟動的節點。

### 平常再次開啟

映像與工作區建置完成後，通常只需要：

```bash
# 主機端
cd /path/to/farm_ws
sudo ./docker/setup_livox_network.sh
./docker/run.sh

# 容器內
./scripts/run_livox_glim_ui.sh
```

離開 `docker/run.sh` 開啟的 shell 後，腳本會停止並移除開發容器；工作區內容仍保留在主機掛載的專案目錄。

## 啟動選項

整合啟動腳本支援：

```bash
./scripts/run_livox_glim_ui.sh --help
./scripts/run_livox_glim_ui.sh --dry-run      # 只檢查軟體、設定與 executable
./scripts/run_livox_glim_ui.sh --skip-camera  # 未接 DECXIN 相機時使用
./scripts/run_livox_glim_ui.sh --headless     # 不開 RQT、RViz
./scripts/run_livox_glim_ui.sh --no-glim      # 只啟動感測器與 Collection 服務
```

若只想查看 UI、不啟動實體感測器：

```bash
./scripts/run_ui_only.sh
```

也可分別開啟介面：

```bash
./scripts/run_rqt_only.sh
./scripts/run_rviz_only.sh
```

這些腳本都必須在容器內執行，且需要先完成 colcon build。GUI 模式需要主機已有 X11 與正確的 `DISPLAY`；無圖形環境時請使用 `--headless`。

## 目前檔案架構

```text
farm_ws/
├── config/
│   ├── livox_mid360/          MID-360 使用的 GLIM 設定與校正警告
│   ├── sensors/               Velodyne、FDILINK、RealSense 舊流程設定
│   ├── udev/                  FDILINK 裝置規則
│   └── cyclonedds_livox.xml   Livox 網路使用的 CycloneDDS 設定
├── data/sessions/             目前保留在專案中的工作階段資料
├── docker/                    Dockerfile、容器啟動與 LiDAR 網路腳本
├── rviz/                      Collection 與 Livox/GLIM 的 RViz 版面
├── scripts/                   建置、啟動、硬體檢查與工作階段檢查工具
├── src/
│   ├── collection_interfaces/ Collection 自訂 msg 與 srv
│   ├── collection_manager/    工作階段、計時、事件與選用 rosbag2 管理
│   ├── collection_rqt_panel/  操作面板
│   ├── collection_bringup/    Collection core/UI launch 檔
│   ├── sensor_bringup/        MID-360 點雲檢查、IMU 轉換、相機與 GLIM supervisor
│   ├── sensor_health_monitor/ 感測器 topic 健康狀態
│   ├── preview_tools/         GLIM 路徑與覆蓋率預覽
│   ├── farm_sensor_bringup/   Velodyne/FDILINK/RealSense 舊整合流程
│   ├── camera/                尚未實作節點的 ROS 套件骨架
│   ├── glim/                  第三方 GLIM core submodule
│   ├── glim_ros2/             第三方 GLIM ROS 2 submodule（套件名 glim_ros）
│   ├── Livox-SDK2/            第三方 Livox 原生 SDK
│   ├── ws_livox/src/
│   │   └── livox_ros_driver2/ 第三方 Livox ROS 2 driver
│   ├── FAST-Calib-ROS2/       第三方 LiDAR/相機外參校正工具
│   ├── fdilink_ahrs/          第三方 FDILINK IMU driver
│   └── serial/                FDILINK 使用的 serial library
├── build/                     colcon 建置產物，不需手動修改
├── install/                   colcon 安裝空間；啟動前要 source
├── log/                       colcon 記錄
├── .gitmodules                GLIM submodule 來源
└── VERSIONS.txt               第三方套件版本紀錄
```

`build/`、`install/`、`log/`、`logs/` 與 `mapping_sessions/` 都是執行時產物，已由 `.gitignore` 排除。

要重新查看或重新建圖：

```bash
./scripts/replay_session.sh mapping_sessions/<時間>_<名稱>
./scripts/replay_session.sh mapping_sessions/<時間>_<名稱> --rerun-glim
```

## 主要 ROS 套件

| 套件 | 類型 | 用途 |
|---|---|---|
| `collection_interfaces` | 自製 | Collection 狀態、事件、感測器狀態與服務介面 |
| `collection_manager` | 自製 | 建立 session、倒數、標記事件；可由參數啟用 rosbag2 |
| `collection_rqt_panel` | 自製 | 顯示健康狀態、GLIM 狀態、覆蓋率與 Collection 控制 |
| `collection_bringup` | 自製 | 啟動 Collection core、RQT 與 RViz |
| `sensor_bringup` | 自製 | MID-360/IMU/DECXIN 資料前處理與 GLIM 監督 |
| `sensor_health_monitor` | 自製 | 檢查點雲、IMU、相機與 TF 是否持續更新 |
| `preview_tools` | 自製 | 將 `/glim_ros/odom` 轉成軌跡與覆蓋率資訊 |
| `farm_sensor_bringup` | 自製／舊流程 | Velodyne、FDILINK、RealSense 的 launch 整合 |
| `glim`、`glim_ros` | 第三方 | LiDAR-inertial mapping |
| `livox_ros_driver2`、`livox_sdk2` | 第三方 | MID-360 通訊與 ROS 2 driver |
| `fast_calib` | 第三方 | 離線 LiDAR/相機標定 |

## 常用工具

| 指令 | 用途 | 執行位置 |
|---|---|---|
| `./docker/build.sh` | 建立開發映像 | 主機 |
| `sudo ./docker/setup_livox_network.sh` | 設定 MID-360 網路 | 主機 |
| `./docker/run.sh` | 建立容器並進入 shell | 主機 |
| `./scripts/build_all.sh` | 安裝 rosdep 並建置全部套件 | 容器 |
| `./scripts/build_glim.sh` | 只建置到 `glim_ros` | 容器 |
| `./scripts/build_ui.sh` | 只建置 Collection UI 相關套件 | 容器 |
| `./scripts/check_glim.sh` | 輸出 GLIM/CUDA/函式庫環境資訊 | 容器 |
| `./scripts/run_livox_glim_ui.sh` | 啟動目前主要整合流程 | 容器 |
| `./scripts/review_sessions.sh [路徑]` | 檢查 rosbag 與 GLIM 軌跡 | 容器 |

## 網路與裝置設定

目前 Livox 主流程使用固定值：

| 項目 | 值 |
|---|---|
| 主機網卡 | `enp5s0` |
| 主機 LiDAR 網段位址 | `192.168.113.1/24` |
| MID-360 位址 | `192.168.113.158` |
| DECXIN USB ID | `1bcf:2cd1` |
| ROS Domain ID | `40` |
| RMW | `rmw_cyclonedds_cpp` |

若實際網卡或 LiDAR IP 不同，需要同步修改：

- [`docker/setup_livox_network.sh`](docker/setup_livox_network.sh)
- [`scripts/run_livox_glim_ui.sh`](scripts/run_livox_glim_ui.sh)
- [`src/ws_livox/src/livox_ros_driver2/config/MID360_config.json`](src/ws_livox/src/livox_ros_driver2/config/MID360_config.json)

主機可先用以下指令確認：

```bash
ip -br address show enp5s0
ping -I enp5s0 -c 3 192.168.113.158
lsusb | grep -i '1bcf:2cd1'
```

## Topic 與資料流

主要 topic 如下：

| Topic | 內容 |
|---|---|
| `/livox/lidar` | Livox driver 原始 PointCloud2 |
| `/livox/lidar_valid` | 經連續性與 IMU 條件檢查後送入 GLIM 的點雲 |
| `/livox/imu` | MID-360 原始 IMU |
| `/livox/imu_base` | 轉換到 `base_link` 且套用加速度比例後的 IMU |
| `/decxin_camera/image_raw` | DECXIN 影像 |
| `/glim_ros/odom` | GLIM odometry |
| `/glim_ros/points`、`/glim_ros/map` | GLIM 配準點雲與地圖 |
| `/coverage/status_json` | 行走距離、姿態數量與近似覆蓋率 |
| `/collection/state` | Collection 工作階段狀態 |
| `/collection/sensor_status` | 感測器健康資訊 |

目前 `config/livox_mid360/config_ros.json` 將 GLIM 的 image topic 設為 `/glim/disabled_image`，因此 DECXIN 影像只供顯示與監控，尚未加入 GLIM 計算。

## 輸出與記錄

- Collection 預設在 `/workspace/farm_ws/mapping_sessions/<時間>_<名稱>/` 建立 `metadata.yaml`、`session.yaml`、marker 與報告目錄。
- `run_livox_glim_ui.sh` 預設不錄製；使用 `--record` 後，取消 RQT 的 **UI only** 才會建立 rosbag2。
- 錄製時保留原始與有效點雲，影像另存為 JPEG compressed topic；TF 與 pose 會一起保存，重播時 rosbag2 會自動解壓縮。
- 啟動檢查、各節點 stdout/stderr 與效能資訊放在容器的 `/tmp/farm_ws_runtime/<UTC 時間>/`。容器移除後這些 `/tmp` 記錄不會保留。
- `data/sessions/` 是目前專案中已有的工作階段資料；它和程式預設新建的 `mapping_sessions/` 用途不同。

## 常見問題

### `src/glim` 是空的

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

### 找不到 ROS 套件或 executable

```bash
cd /workspace/farm_ws
source /opt/ros/humble/setup.bash
./scripts/build_all.sh
source install/setup.bash
colcon list
```

### LiDAR 無法連線

確認 MID-360 已供電、網線 carrier 正常，再於主機重新執行：

```bash
sudo ./docker/setup_livox_network.sh
```

### 相機未連接

使用：

```bash
./scripts/run_livox_glim_ui.sh --skip-camera
```

### 沒有圖形畫面

確認主機已在圖形桌面工作階段並設定 `DISPLAY`。遠端或無桌面環境可改用：

```bash
./scripts/run_livox_glim_ui.sh --headless
```

## 版本與環境

- 主機／容器基底：Ubuntu 22.04
- ROS：ROS 2 Humble
- CUDA image：12.6.3
- GLIM 目前選用 CPU odometry、sub-mapping 與 global-mapping 設定；Docker image 仍包含 CUDA 建置環境。
- 更完整的第三方版本紀錄請見 [`VERSIONS.txt`](VERSIONS.txt)。
