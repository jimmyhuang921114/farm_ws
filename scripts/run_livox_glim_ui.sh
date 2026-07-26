#!/usr/bin/env bash
set -Eeuo pipefail

readonly ROOT="/workspace/farm_ws"
readonly EXPECTED_CONTAINER="farm_ws_glim_ui_dev"
readonly LIDAR_INTERFACE="enp5s0"
readonly HOST_CIDR="192.168.113.1/24"
readonly LIDAR_IP="192.168.113.158"
readonly CAMERA_USB_ID="1bcf:2cd1"
readonly LIVOX_CONFIG="${ROOT}/src/ws_livox/src/livox_ros_driver2/config/MID360_config.json"
readonly GLIM_CONFIG="${ROOT}/config/livox_mid360"
readonly CYCLONEDDS_CONFIG="${ROOT}/config/cyclonedds_livox.xml"
readonly RVIZ_CONFIG="${ROOT}/rviz/livox_glim_ui.rviz"
readonly WAIT_SECONDS=30

skip_camera=false
headless=false
no_glim=false
dry_run=false
runtime_dir=""
declare -a child_pids=()
declare -a child_names=()

usage() {
  cat <<'EOF'
Usage: run_livox_glim_ui.sh [OPTIONS]

Start Livox MID-360, DECXIN camera, IMU transform, GLIM, Collection UI,
and RViz2 after validating each required upstream data stream.

Options:
  --skip-camera  Continue without the DECXIN camera.
  --headless     Do not start RQT or RViz2.
  --no-glim      Do not start GLIM or wait for mapping outputs.
  --dry-run      Validate software/configuration without touching hardware.
  -h, --help     Show this help.
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

warn() {
  echo "WARN: $*" >&2
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if ((${#child_pids[@]})); then
    echo "Stopping processes started by this run..."
    for ((index=${#child_pids[@]} - 1; index >= 0; index--)); do
      pid="${child_pids[index]}"
      name="${child_names[index]}"
      if kill -0 -- "-${pid}" 2>/dev/null; then
        echo "  TERM ${name} process group (${pid})"
        kill -TERM -- "-${pid}" 2>/dev/null || true
      fi
    done
    for _ in {1..25}; do
      any_running=false
      for pid in "${child_pids[@]}"; do
        kill -0 -- "-${pid}" 2>/dev/null && any_running=true
      done
      [[ "${any_running}" == false ]] && break
      sleep 0.2
    done
    for pid in "${child_pids[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        echo "  KILL remaining owned process group (${pid})"
        kill -KILL -- "-${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    done
  fi

  if [[ -n "${runtime_dir}" ]]; then
    echo "Runtime logs: ${runtime_dir}"
  fi
  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

while (($#)); do
  case "$1" in
    --skip-camera) skip_camera=true ;;
    --headless) headless=true ;;
    --no-glim) no_glim=true ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1 (use --help)" ;;
  esac
  shift
done

[[ -e /.dockerenv ]] || die "this script must run inside ${EXPECTED_CONTAINER}"
[[ "${FARM_WS_CONTAINER_NAME:-}" == "${EXPECTED_CONTAINER}" ]] ||
  die "FARM_WS_CONTAINER_NAME must be ${EXPECTED_CONTAINER}; recreate it with docker/run.sh"
[[ -d "${ROOT}" ]] || die "workspace is not mounted at ${ROOT}"
cd "${ROOT}"

[[ -r /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble setup is missing"
# ROS setup scripts are not nounset-clean.
set +u
source /opt/ros/humble/setup.bash
[[ -r install/setup.bash ]] || die "${ROOT}/install/setup.bash is missing; build first"
source install/setup.bash
set -u

export ROS_DOMAIN_ID=40
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
export ROS2CLI_DISABLE_DAEMON=1

for command in ip ping ss lsusb ros2 python3 setsid timeout; do
  command -v "${command}" >/dev/null || die "required command is missing: ${command}"
done
for path in "${LIVOX_CONFIG}" "${GLIM_CONFIG}/config.json" \
  "${CYCLONEDDS_CONFIG}" \
  "${GLIM_CONFIG}/config_ros.json" "${RVIZ_CONFIG}"; do
  [[ -r "${path}" ]] || die "required file is missing: ${path}"
done

python3 - "${GLIM_CONFIG}/config_ros.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    config = json.load(stream)["glim_ros"]

expected = {
    "points_topic": "/livox/lidar_valid",
    "imu_topic": "/livox/imu_base",
    "image_topic": "/glim/disabled_image",
    "lidar_frame_id": "livox_frame",
    "imu_frame_id": "base_link",
}
for key, value in expected.items():
    if config.get(key) != value:
        raise SystemExit(f"{key}: expected {value!r}, got {config.get(key)!r}")
PY

for package_executable in \
  "livox_ros_driver2 livox_ros_driver2_node" \
  "sensor_bringup open_camera" \
  "sensor_bringup imu_transform" \
  "sensor_bringup livox_cloud_guard" \
  "sensor_bringup livox_cloud_monitor.py" \
  "sensor_bringup livox_time_sync_monitor.py" \
  "sensor_bringup glim_session_supervisor.py" \
  "tf2_ros static_transform_publisher" \
  "collection_manager collection_manager" \
  "sensor_health_monitor sensor_health_monitor" \
  "preview_tools coverage_analyzer"; do
  read -r package executable <<<"${package_executable}"
  ros2 pkg executables "${package}" | awk '{print $2}' | grep -Fxq "${executable}" ||
    die "missing executable: ${package}/${executable}"
done
if [[ "${no_glim}" == false ]]; then
  ros2 pkg executables glim_ros | awk '{print $2}' | grep -Fxq glim_rosnode ||
    die "missing executable: glim_ros/glim_rosnode"
fi

if [[ "${dry_run}" == true ]]; then
  echo "Dry run passed: environment, executables, MID-360 profile, and RViz config are valid."
  exit 0
fi

runtime_dir="/tmp/farm_ws_runtime/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "${runtime_dir}"
export ROS_LOG_DIR="${runtime_dir}/ros"
mkdir -p "${ROS_LOG_DIR}"
printf '%s\n' "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" \
  "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  "CYCLONEDDS_URI=${CYCLONEDDS_URI}" >"${runtime_dir}/environment.txt"

node_exists() {
  timeout 8s ros2 node list --no-daemon 2>/dev/null | grep -Fxq "$1"
}

assert_node_absent() {
  node_exists "$1" && die "ROS node already exists; refusing duplicate launch: $1"
  return 0
}

start_process() {
  local name=$1
  shift
  echo "Starting ${name}..."
  setsid "$@" >"${runtime_dir}/${name}.log" 2>&1 &
  local pid=$!
  child_names+=("${name}")
  child_pids+=("${pid}")
  printf '%s\n' "${pid}" >"${runtime_dir}/${name}.pid"
  sleep 1
  kill -0 "${pid}" 2>/dev/null || {
    tail -80 "${runtime_dir}/${name}.log" >&2 || true
    die "${name} exited during startup"
  }
}

start_process runtime_metrics "${ROOT}/scripts/runtime_metrics.sh" 5

wait_for_echo() {
  local topic=$1
  local description=$2
  shift 2
  local deadline=$((SECONDS + WAIT_SECONDS))
  while ((SECONDS < deadline)); do
    if timeout 3s ros2 topic echo "${topic}" --once --no-daemon \
      "$@" \
      >"${runtime_dir}/probe_${description}.txt" 2>/dev/null; then
      echo "Validated ${description} on ${topic}."
      return 0
    fi
    sleep 1
  done
  die "timed out waiting for ${description} on ${topic}"
}

wait_for_nonempty_cloud() {
  local topic=$1
  local probe_name=$2
  local deadline=$((SECONDS + WAIT_SECONDS))
  while ((SECONDS < deadline)); do
    if timeout 3s ros2 topic echo "${topic}" --once --field width --no-daemon \
      >"${runtime_dir}/probe_${probe_name}_width.txt" 2>/dev/null &&
      awk '$1 + 0 > 0 {found=1} END {exit !found}' \
        "${runtime_dir}/probe_${probe_name}_width.txt"; then
      echo "Validated non-empty PointCloud2 on ${topic}."
      return 0
    fi
    sleep 1
  done
  die "timed out waiting for a non-empty PointCloud2 on ${topic}"
}

wait_for_typed_publisher() {
  local topic=$1
  local expected_type=$2
  local description=$3
  local deadline=$((SECONDS + WAIT_SECONDS))
  while ((SECONDS < deadline)); do
    info="$(timeout 3s ros2 topic info "${topic}" --no-daemon 2>/dev/null || true)"
    if grep -Fq "Type: ${expected_type}" <<<"${info}" &&
      grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${info}"; then
      printf '%s\n' "${info}" >"${runtime_dir}/probe_${description}.txt"
      echo "Validated ${description} publisher on ${topic}."
      return 0
    fi
    sleep 1
  done
  die "timed out waiting for ${description} publisher on ${topic}"
}

wait_for_true() {
  local topic=$1
  local description=$2
  local deadline=$((SECONDS + WAIT_SECONDS * 2))
  while ((SECONDS < deadline)); do
    if timeout 3s ros2 topic echo "${topic}" --once --field data --no-daemon \
      2>/dev/null | grep -Fixq true; then
      echo "Validated ${description} on ${topic}."
      return 0
    fi
    sleep 1
  done
  die "timed out waiting for ${description} on ${topic}"
}

ip link show dev "${LIDAR_INTERFACE}" >/dev/null 2>&1 ||
  die "host interface ${LIDAR_INTERFACE} is not visible in the host-network container"
[[ "$(cat "/sys/class/net/${LIDAR_INTERFACE}/carrier" 2>/dev/null || echo 0)" == "1" ]] ||
  die "${LIDAR_INTERFACE} has NO-CARRIER; connect MID-360 and run docker/setup_livox_network.sh on the host"
ip -o -4 address show dev "${LIDAR_INTERFACE}" | awk '{print $4}' |
  grep -Fxq "${HOST_CIDR}" ||
  die "${HOST_CIDR} is missing on ${LIDAR_INTERFACE}; run docker/setup_livox_network.sh on the host"
ping -I "${LIDAR_INTERFACE}" -c 2 -W 1 "${LIDAR_IP}" >/dev/null ||
  die "MID-360 ${LIDAR_IP} is unreachable through ${LIDAR_INTERFACE}"

if [[ "${skip_camera}" == false ]]; then
  lsusb | grep -iq "${CAMERA_USB_ID}" ||
    die "DECXIN camera ${CAMERA_USB_ID} not found (or use --skip-camera)"
fi

occupied_ports="$(ss -H -lunp | grep -E ':(56101|56201|56301|56401|56501)([[:space:]]|$)' || true)"
[[ -z "${occupied_ports}" ]] || die "Livox host UDP port is already occupied: ${occupied_ports}"

if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA GPU/runtime available."
else
  warn "NVIDIA GPU/runtime unavailable; GLIM must use the CPU modules in ${GLIM_CONFIG}."
fi

assert_node_absent /livox_lidar_publisher
start_process livox_driver \
  ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 \
  -p frame_id:=livox_frame -p user_config_path:="${LIVOX_CONFIG}"

deadline=$((SECONDS + WAIT_SECONDS))
while ((SECONDS < deadline)); do
  type="$(timeout 3s ros2 topic type /livox/lidar 2>/dev/null || true)"
  [[ "${type}" == "sensor_msgs/msg/PointCloud2" ]] && break
  sleep 1
done
[[ "${type:-}" == "sensor_msgs/msg/PointCloud2" ]] ||
  die "/livox/lidar is not sensor_msgs/msg/PointCloud2"
wait_for_nonempty_cloud /livox/lidar raw_lidar
wait_for_echo /livox/imu imu

assert_node_absent /livox_cloud_monitor
start_process livox_cloud_monitor \
  ros2 run sensor_bringup livox_cloud_monitor.py --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_directory:="${runtime_dir}/cloud_monitor"

# This is the inverse pose corresponding to the repository's existing,
# uncalibrated T_lidar_imu. For this particular 180-degree rotation and
# translation its numeric tuple is self-inverse. Publishing base -> lidar keeps
# one TF tree after GLIM starts publishing map -> odom -> base_link.
warn "using uncalibrated repository T_lidar_imu; do not treat it as a calibration result"
assert_node_absent /livox_imu_static_tf
start_process livox_imu_static_tf \
  ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.07 \
  --qx 0.7071068 --qy 0.7071068 --qz 0.0 --qw 0.0 \
  --frame-id base_link --child-frame-id livox_frame --ros-args \
  -r __node:=livox_imu_static_tf

assert_node_absent /livox_imu_transform
start_process imu_transform \
  ros2 run sensor_bringup imu_transform --ros-args \
  -r __node:=livox_imu_transform \
  -p input_topic:=/livox/imu -p output_topic:=/livox/imu_base \
  -p target_frame:=base_link -p accel_scale:=9.80665 \
  -p use_latest_tf:=true -p force_orientation_unavailable:=true
wait_for_echo /livox/imu_base imu_base

assert_node_absent /livox_cloud_guard
start_process livox_cloud_guard \
  ros2 run sensor_bringup livox_cloud_guard --ros-args \
  -p input_topic:=/livox/lidar -p output_topic:=/livox/lidar_valid \
  -p imu_topic:=/livox/imu_base
start_process continuity_diagnostics \
  ros2 topic echo /livox/continuity/diagnostics --no-daemon
wait_for_nonempty_cloud /livox/lidar_valid guarded_lidar

if [[ "${skip_camera}" == false ]]; then
  assert_node_absent /decxin_camera
  start_process decxin_camera ros2 run sensor_bringup open_camera --ros-args \
    -p width:=1280 -p height:=720 -p fps:=30.0 -p fourcc:=MJPG
  wait_for_echo /decxin_camera/image_raw camera_image
fi

if [[ "${no_glim}" == false ]]; then
  assert_node_absent /glim_session_supervisor
  start_process glim_session_supervisor \
    ros2 run sensor_bringup glim_session_supervisor.py --ros-args \
    -p config_path:="${GLIM_CONFIG}" \
    -p runtime_directory:="${runtime_dir}"
  wait_for_true /glim/session_supervisor/ready glim_session_ready
  wait_for_echo /glim_ros/odom glim_odometry
  wait_for_echo /glim_ros/points glim_registered_points
  wait_for_typed_publisher /glim_ros/map sensor_msgs/msg/PointCloud2 glim_map
  warn "GLIM map publisher is ready; the first map message requires enough motion to form a submap"
  assert_node_absent /coverage_analyzer
  start_process coverage_analyzer \
    ros2 run preview_tools coverage_analyzer --ros-args \
    -p odom_topic:=/glim_ros/odom
  wait_for_echo /coverage/status_json coverage_status
fi

assert_node_absent /collection_manager
start_process collection_manager ros2 run collection_manager collection_manager
assert_node_absent /sensor_health_monitor
health_args=(ros2 run sensor_health_monitor sensor_health_monitor --ros-args
  -p lidar_points_topic:=/livox/lidar
  -p imu_topic:=/livox/imu_base
  -p camera_image_topic:=/decxin_camera/image_raw
  -p camera_info_topic:=/decxin_camera/camera_info)
start_process sensor_health_monitor "${health_args[@]}"

deadline=$((SECONDS + WAIT_SECONDS))
while ((SECONDS < deadline)); do
  if timeout 3s ros2 service type /collection/preflight 2>/dev/null |
    grep -Fxq collection_interfaces/srv/PreflightCollection; then
    break
  fi
  sleep 1
done
((SECONDS < deadline)) || die "Collection UI services did not become ready"

if [[ "${headless}" == false ]]; then
  [[ -n "${DISPLAY:-}" ]] || die "DISPLAY is unset; use --headless"
  assert_node_absent /rviz
  start_process rqt rqt --standalone collection_rqt_panel.panel.CollectionPanel
  rviz_args=(rviz2 -d "${RVIZ_CONFIG}")
  [[ "${no_glim}" == true ]] && rviz_args+=(-f livox_frame)
  start_process rviz "${rviz_args[@]}"
fi

echo "Integrated stack is ready. Runtime directory: ${runtime_dir}"
echo "Press Ctrl+C for an orderly shutdown."
while true; do
  sleep 2
  for index in "${!child_pids[@]}"; do
    if ! kill -0 "${child_pids[index]}" 2>/dev/null; then
      die "${child_names[index]} exited unexpectedly; inspect ${runtime_dir}/${child_names[index]}.log"
    fi
  done
done
