#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly CAMERA_USB_ID="1bcf:2cd1"
readonly LIVOX_CONFIG_TEMPLATE="${ROOT}/src/ws_livox/src/livox_ros_driver2/config/MID360_config.json"
readonly GLIM_CONFIG="${ROOT}/config/livox_mid360"
readonly CYCLONEDDS_CONFIG_TEMPLATE="${ROOT}/config/cyclonedds_livox.xml"
readonly RVIZ_CONFIG="${ROOT}/rviz/livox_glim_ui.rviz"
readonly WAIT_SECONDS=30

skip_camera=false
headless=false
no_glim=false
dry_run=false
record=false
runtime_dir=""
livox_interface=""
livox_interface_source=""
livox_host_ip=""
livox_host_ip_source=""
livox_ip=""
livox_ip_source=""
livox_runtime_config=""
cyclonedds_runtime_config=""

declare -a child_pids=()
declare -a child_names=()

usage() {
  cat <<'EOF'
Usage: run_livox_glim_ui.sh [OPTIONS]

Start Livox MID-360, DECXIN camera, IMU transform, GLIM,
Collection UI, and RViz2 after validating each required
upstream data stream.

Options:
  --skip-camera  Continue without the DECXIN camera.
  --headless     Do not start RQT or RViz2.
  --no-glim      Do not start GLIM or wait for mapping outputs.
  --record       Enable session recording for later replay/reconstruction.
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
    --skip-camera)
      skip_camera=true
      ;;
    --headless)
      headless=true
      ;;
    --no-glim)
      no_glim=true
      ;;
    --dry-run)
      dry_run=true
      ;;
    --record)
      record=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1 (use --help)"
      ;;
  esac
  shift
done

# ============================================================
# Docker check
# ============================================================

# 不再綁定 farm_ws_glim_ui_dev 這個 container 名稱。
# 只要求這支 script 必須在 Docker 裡執行。
[[ -e /.dockerenv ]] || die "this script must run inside Docker"

[[ -d "${ROOT}" ]] || die "workspace is not mounted at ${ROOT}"

cd "${ROOT}"

# ============================================================
# ROS setup
# ============================================================

[[ -r /opt/ros/humble/setup.bash ]] ||
  die "ROS 2 Humble setup is missing"

# ROS setup scripts are not nounset-clean.
set +u
source /opt/ros/humble/setup.bash

[[ -r install/setup.bash ]] ||
  die "${ROOT}/install/setup.bash is missing; build first"

source install/setup.bash
set -u

export ROS_DOMAIN_ID=40
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS2CLI_DISABLE_DAEMON=1

# ============================================================
# Required commands
# ============================================================

for command in ip ping ss lsusb ros2 python3 sed setsid timeout; do
  command -v "${command}" >/dev/null ||
    die "required command is missing: ${command}"
done

# ============================================================
# Required files
# ============================================================

for path in \
  "${LIVOX_CONFIG_TEMPLATE}" \
  "${GLIM_CONFIG}/config.json" \
  "${CYCLONEDDS_CONFIG_TEMPLATE}" \
  "${GLIM_CONFIG}/config_ros.json" \
  "${RVIZ_CONFIG}"; do

  [[ -r "${path}" ]] ||
    die "required file is missing: ${path}"
done

# ============================================================
# Validate GLIM ROS configuration
# ============================================================

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
        raise SystemExit(
            f"{key}: expected {value!r}, got {config.get(key)!r}"
        )
PY

# ============================================================
# Validate ROS executables
# ============================================================

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

  ros2 pkg executables "${package}" |
    awk '{print $2}' |
    grep -Fxq "${executable}" ||
    die "missing executable: ${package}/${executable}"
done

if [[ "${no_glim}" == false ]]; then
  ros2 pkg executables glim_ros |
    awk '{print $2}' |
    grep -Fxq glim_rosnode ||
    die "missing executable: glim_ros/glim_rosnode"
fi

# ============================================================
# Runtime directory
# ============================================================

runtime_dir="${ROOT}/data/runtime/$(date -u +%Y%m%dT%H%M%SZ)"

mkdir -p "${runtime_dir}"

export ROS_LOG_DIR="${runtime_dir}/ros"
mkdir -p "${ROS_LOG_DIR}"

# ============================================================
# Helpers
# ============================================================

valid_ipv4() {
  [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]
}

iface_allowed() {
  local iface=$1

  [[ "${iface}" != lo ]] || return 1
  [[ "${iface}" != docker0 ]] || return 1
  [[ "${iface}" != br-* ]] || return 1
  [[ "${iface}" != veth* ]] || return 1

  return 0
}

iface_carrier() {
  local iface=$1
  cat "/sys/class/net/${iface}/carrier" 2>/dev/null || echo 0
}

first_iface_ipv4() {
  local iface=$1

  ip -o -4 address show dev "${iface}" scope global |
    awk '{print $4}' |
    cut -d/ -f1 |
    head -n1
}

config_lidar_ip() {
  python3 - "${LIVOX_CONFIG_TEMPLATE}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    config = json.load(stream)

for entry in config.get("MID360", {}).get("host_net_info", []):
    for ip in entry.get("lidar_ip", []):
        if ip:
            print(ip)
            raise SystemExit(0)

for entry in config.get("lidar_configs", []):
    ip = entry.get("ip")
    if ip:
        print(ip)
        raise SystemExit(0)
PY
}

iface_for_ip_route() {
  local target_ip=$1

  ip -o route get "${target_ip}" 2>/dev/null |
    awk '
      {
        for (i = 1; i <= NF; i++) {
          if ($i == "dev" && (i + 1) <= NF) {
            print $(i + 1)
            exit
          }
        }
      }
    '
}

discover_lidar_ip_from_neighbors() {
  local iface=$1
  local host_ip=$2
  local ignored_ip=${3:-}

  ip neigh show dev "${iface}" 2>/dev/null |
    awk -v host_ip="${host_ip}" -v ignored_ip="${ignored_ip}" '
      $1 != host_ip && $1 != ignored_ip &&
      $1 ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/ &&
      $0 !~ /FAILED|INCOMPLETE/ {
        print $1
        exit
      }
    '
}

neighbor_has_ip() {
  local iface=$1
  local target_ip=$2

  ip neigh show dev "${iface}" to "${target_ip}" 2>/dev/null |
    awk '$0 !~ /FAILED|INCOMPLETE/ {found=1} END {exit !found}'
}

select_livox_interface() {
  local configured_ip=$1
  local iface=""

  if [[ -n "${LIVOX_INTERFACE:-}" ]]; then
    ip link show dev "${LIVOX_INTERFACE}" >/dev/null 2>&1 ||
      die "LIVOX_INTERFACE=${LIVOX_INTERFACE} is not visible in the container"

    iface_allowed "${LIVOX_INTERFACE}" ||
      die "LIVOX_INTERFACE=${LIVOX_INTERFACE} is excluded"

    livox_interface="${LIVOX_INTERFACE}"
    livox_interface_source="env"
    return 0
  fi

  if [[ -n "${LIVOX_HOST_IP:-}" ]]; then
    iface="$(
      ip -o -4 address show scope global |
        awk -v host_ip="${LIVOX_HOST_IP}" '$4 ~ "^" host_ip "/" {print $2; exit}'
    )"

    if [[ -n "${iface}" ]] && iface_allowed "${iface}"; then
      livox_interface="${iface}"
      livox_interface_source="host-ip"
      return 0
    fi
  fi

  if [[ -n "${configured_ip}" ]]; then
    iface="$(iface_for_ip_route "${configured_ip}")"

    if [[ -n "${iface}" ]] && iface_allowed "${iface}"; then
      livox_interface="${iface}"
      livox_interface_source="route"
      return 0
    fi
  fi

  while read -r iface; do
    iface_allowed "${iface}" || continue
    [[ "$(iface_carrier "${iface}")" == "1" ]] || continue
    [[ -n "$(first_iface_ipv4 "${iface}")" ]] || continue

    livox_interface="${iface}"
    livox_interface_source="auto"
    return 0
  done < <(ip -o link show | awk -F': ' '{print $2}' | cut -d@ -f1)

  die "could not auto-select a Livox Ethernet interface; set LIVOX_INTERFACE"
}

select_livox_host_ip() {
  if [[ -n "${LIVOX_HOST_IP:-}" ]]; then
    valid_ipv4 "${LIVOX_HOST_IP}" ||
      die "LIVOX_HOST_IP=${LIVOX_HOST_IP} is not a valid IPv4 address"

    ip -o -4 address show dev "${livox_interface}" |
      awk '{print $4}' |
      cut -d/ -f1 |
      grep -Fxq "${LIVOX_HOST_IP}" ||
      die "LIVOX_HOST_IP=${LIVOX_HOST_IP} is not assigned to ${livox_interface}"

    livox_host_ip="${LIVOX_HOST_IP}"
    livox_host_ip_source="env"
    return 0
  fi

  livox_host_ip="$(first_iface_ipv4 "${livox_interface}")"
  [[ -n "${livox_host_ip}" ]] ||
    die "${livox_interface} has no global IPv4 address; set LIVOX_HOST_IP or configure the host interface"

  livox_host_ip_source="interface"
}

select_livox_ip() {
  local configured_ip=$1
  local discovered_ip=""

  if [[ -n "${LIVOX_IP:-}" ]]; then
    valid_ipv4 "${LIVOX_IP}" ||
      die "LIVOX_IP=${LIVOX_IP} is not a valid IPv4 address"

    livox_ip="${LIVOX_IP}"
    livox_ip_source="env"
    return 0
  fi

  if [[ -n "${configured_ip}" ]]; then
    discovered_ip="$(
      discover_lidar_ip_from_neighbors \
        "${livox_interface}" \
        "${livox_host_ip}" \
        "${configured_ip}"
    )"

    if [[ -n "${discovered_ip}" ]] &&
       ! neighbor_has_ip "${livox_interface}" "${configured_ip}"; then

      livox_ip="${discovered_ip}"
      livox_ip_source="auto-discovery"
      return 0
    fi

    livox_ip="${configured_ip}"
    livox_ip_source="config"
    return 0
  fi

  discovered_ip="$(discover_lidar_ip_from_neighbors "${livox_interface}" "${livox_host_ip}")"

  [[ -n "${discovered_ip}" ]] ||
    die "could not discover a MID-360 IP from ${livox_interface}; set LIVOX_IP or update ${LIVOX_CONFIG_TEMPLATE}"

  livox_ip="${discovered_ip}"
  livox_ip_source="auto-discovery"
}

write_runtime_livox_config() {
  livox_runtime_config="${runtime_dir}/MID360_runtime_config.json"

  python3 - "${LIVOX_CONFIG_TEMPLATE}" "${livox_runtime_config}" "${livox_host_ip}" "${livox_ip}" <<'PY'
import json
import sys

source, target, host_ip, lidar_ip = sys.argv[1:5]

with open(source, encoding="utf-8") as stream:
    config = json.load(stream)

mid360 = config.setdefault("MID360", {})
host_infos = mid360.setdefault("host_net_info", [{}])
if not host_infos:
    host_infos.append({})

for entry in host_infos:
    entry["host_ip"] = host_ip
    entry["lidar_ip"] = [lidar_ip]

lidar_configs = config.setdefault("lidar_configs", [{}])
if not lidar_configs:
    lidar_configs.append({})

for entry in lidar_configs:
    entry["ip"] = lidar_ip

with open(target, "w", encoding="utf-8") as stream:
    json.dump(config, stream, indent=2)
    stream.write("\n")
PY
}

write_runtime_cyclonedds_config() {
  cyclonedds_runtime_config="${runtime_dir}/cyclonedds_livox.xml"

  sed "s/@LIVOX_INTERFACE@/${livox_interface}/g" \
    "${CYCLONEDDS_CONFIG_TEMPLATE}" \
    >"${cyclonedds_runtime_config}"

  export CYCLONEDDS_URI="file://${cyclonedds_runtime_config}"
}

print_livox_network_configuration() {
  cat <<EOF
Detected MID-360
Livox network configuration
---------------------------
Interface : ${livox_interface} (${livox_interface_source})
Host IP   : ${livox_host_ip} (${livox_host_ip_source})
LiDAR IP  : ${livox_ip} (${livox_ip_source})
Config    : ${livox_runtime_config}
CycloneDDS: ${cyclonedds_runtime_config}
EOF
}

node_exists() {
  timeout 8s ros2 node list --no-daemon 2>/dev/null |
    grep -Fxq "$1"
}

assert_node_absent() {
  node_exists "$1" &&
    die "ROS node already exists; refusing duplicate launch: $1"

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

wait_for_echo() {
  local topic=$1
  local description=$2
  shift 2

  local deadline=$((SECONDS + WAIT_SECONDS))

  while ((SECONDS < deadline)); do
    if timeout 3s \
      ros2 topic echo "${topic}" \
      --once \
      --no-daemon \
      "$@" \
      >"${runtime_dir}/probe_${description}.txt" \
      2>/dev/null; then

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
    if timeout 3s \
      ros2 topic echo "${topic}" \
      --once \
      --field width \
      --no-daemon \
      >"${runtime_dir}/probe_${probe_name}_width.txt" \
      2>/dev/null &&
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
    info="$(
      timeout 3s ros2 topic info "${topic}" --no-daemon 2>/dev/null || true
    )"

    if grep -Fq "Type: ${expected_type}" <<<"${info}" &&
       grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${info}"; then

      printf '%s\n' "${info}" \
        >"${runtime_dir}/probe_${description}.txt"

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
    if timeout 3s \
      ros2 topic echo "${topic}" \
      --once \
      --field data \
      --no-daemon \
      2>/dev/null |
      grep -Fixq true; then

      echo "Validated ${description} on ${topic}."
      return 0
    fi

    sleep 1
  done

  die "timed out waiting for ${description} on ${topic}"
}

# ============================================================
# Network / Livox configuration
# ============================================================

configured_livox_ip="$(config_lidar_ip)"

select_livox_interface "${configured_livox_ip}"
select_livox_host_ip
select_livox_ip "${configured_livox_ip}"
write_runtime_livox_config
write_runtime_cyclonedds_config
print_livox_network_configuration

printf '%s\n' \
  "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" \
  "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  "CYCLONEDDS_URI=${CYCLONEDDS_URI}" \
  "LIVOX_INTERFACE=${livox_interface}" \
  "LIVOX_INTERFACE_SOURCE=${livox_interface_source}" \
  "LIVOX_HOST_IP=${livox_host_ip}" \
  "LIVOX_HOST_IP_SOURCE=${livox_host_ip_source}" \
  "LIVOX_IP=${livox_ip}" \
  "LIVOX_IP_SOURCE=${livox_ip_source}" \
  "LIVOX_CONFIG=${livox_runtime_config}" \
  >"${runtime_dir}/environment.txt"

if [[ "${dry_run}" == true ]]; then
  echo "Dry run passed: environment, executables, runtime Livox config, and RViz config are valid."
  exit 0
fi

[[ "$(iface_carrier "${livox_interface}")" == "1" ]] ||
  die "${livox_interface} has NO-CARRIER; connect MID-360 or select the correct interface with LIVOX_INTERFACE"

if ! ping -I "${livox_interface}" -c 1 -W 1 "${livox_ip}" >/dev/null 2>&1; then
  warn "MID-360 ${livox_ip} did not answer ICMP ping through ${livox_interface}; continuing because Livox UDP discovery/data may still work"
fi

# ============================================================
# Runtime metrics
# ============================================================

start_process runtime_metrics \
  "${ROOT}/scripts/runtime_metrics.sh" 5

# ============================================================
# Camera validation
# ============================================================

if [[ "${skip_camera}" == false ]]; then
  lsusb |
    grep -iq "${CAMERA_USB_ID}" ||
    die "DECXIN camera ${CAMERA_USB_ID} not found (or use --skip-camera)"

  [[ -e /dev/video0 || -e /dev/video1 ]] ||
    die "DECXIN USB exists but no /dev/video* device is available inside Docker"
fi

# ============================================================
# UDP port check
# ============================================================

occupied_ports="$(
  ss -H -lunp |
    grep -E ':(56101|56201|56301|56401|56501)([[:space:]]|$)' ||
    true
)"

[[ -z "${occupied_ports}" ]] ||
  die "Livox host UDP port is already occupied: ${occupied_ports}"

# ============================================================
# GPU
# ============================================================

if command -v nvidia-smi >/dev/null &&
   nvidia-smi >/dev/null 2>&1; then

  echo "NVIDIA GPU/runtime available."
else
  warn "NVIDIA GPU/runtime unavailable; GLIM must use the CPU modules in ${GLIM_CONFIG}."
fi

# ============================================================
# Livox driver
# ============================================================

assert_node_absent /livox_lidar_publisher

start_process livox_driver \
  "${ROOT}/build/livox_ros_driver2/livox_ros_driver2_node" \
  --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:="${livox_runtime_config}"

deadline=$((SECONDS + WAIT_SECONDS))

while ((SECONDS < deadline)); do
  type="$(
    timeout 3s ros2 topic type /livox/lidar 2>/dev/null || true
  )"

  [[ "${type}" == "sensor_msgs/msg/PointCloud2" ]] && break
  sleep 1
done

[[ "${type:-}" == "sensor_msgs/msg/PointCloud2" ]] ||
  die "/livox/lidar is not sensor_msgs/msg/PointCloud2"

wait_for_nonempty_cloud /livox/lidar raw_lidar
wait_for_echo /livox/imu imu

# ============================================================
# Livox cloud monitor
# ============================================================

assert_node_absent /livox_cloud_monitor

start_process livox_cloud_monitor \
  ros2 run sensor_bringup livox_cloud_monitor.py \
  --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_directory:="${runtime_dir}/cloud_monitor"

# ============================================================
# Static TF
# ============================================================

warn "using uncalibrated repository T_lidar_imu; do not treat it as a calibration result"

assert_node_absent /livox_imu_static_tf

start_process livox_imu_static_tf \
  ros2 run tf2_ros static_transform_publisher \
  --x 0.0 \
  --y 0.0 \
  --z 0.07 \
  --qx 0.7071068 \
  --qy 0.7071068 \
  --qz 0.0 \
  --qw 0.0 \
  --frame-id base_link \
  --child-frame-id livox_frame \
  --ros-args \
  -r __node:=livox_imu_static_tf

# ============================================================
# IMU transform
# ============================================================

assert_node_absent /livox_imu_transform

start_process imu_transform \
  ros2 run sensor_bringup imu_transform \
  --ros-args \
  -r __node:=livox_imu_transform \
  -p input_topic:=/livox/imu \
  -p output_topic:=/livox/imu_base \
  -p target_frame:=base_link \
  -p accel_scale:=9.80665 \
  -p use_latest_tf:=true \
  -p force_orientation_unavailable:=true

wait_for_echo /livox/imu_base imu_base

# ============================================================
# Livox cloud guard
# ============================================================

assert_node_absent /livox_cloud_guard

start_process livox_cloud_guard \
  ros2 run sensor_bringup livox_cloud_guard \
  --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/livox/lidar_valid \
  -p imu_topic:=/livox/imu_base \
  -p minimum_valid_ratio:=0.05

start_process continuity_diagnostics \
  ros2 topic echo \
  /livox/continuity/diagnostics \
  --no-daemon

wait_for_nonempty_cloud /livox/lidar_valid guarded_lidar

# ============================================================
# DECXIN camera
# ============================================================

if [[ "${skip_camera}" == false ]]; then
  assert_node_absent /decxin_camera

  start_process decxin_camera \
    ros2 run sensor_bringup open_camera \
    --ros-args \
    -p vendor_id:=1bcf \
    -p product_id:=2cd1 \
    -p image_topic:=/decxin_camera/image_raw \
    -p frame_id:=decxin_camera_link \
    -p width:=1280 \
    -p height:=720 \
    -p fps:=30.0 \
    -p fourcc:=MJPG \
    -p reconnect_interval:=1.0 \
    -p max_read_failures:=10

  wait_for_echo \
    /decxin_camera/image_raw \
    camera_image
fi

# ============================================================
# GLIM
# ============================================================

if [[ "${no_glim}" == false ]]; then
  assert_node_absent /glim_session_supervisor

  start_process glim_session_supervisor \
    ros2 run sensor_bringup glim_session_supervisor.py \
    --ros-args \
    -p config_path:="${GLIM_CONFIG}" \
    -p runtime_directory:="${runtime_dir}"

  wait_for_true \
    /glim/session_supervisor/ready \
    glim_session_ready

  wait_for_echo \
    /glim_ros/odom \
    glim_odometry

  wait_for_echo \
    /glim_ros/points \
    glim_registered_points

  wait_for_typed_publisher \
    /glim_ros/map \
    sensor_msgs/msg/PointCloud2 \
    glim_map

  warn "GLIM map publisher is ready; the first map message requires enough motion to form a submap"

  assert_node_absent /coverage_analyzer

  start_process coverage_analyzer \
    ros2 run preview_tools coverage_analyzer \
    --ros-args \
    -p odom_topic:=/glim_ros/odom

  wait_for_echo \
    /coverage/status_json \
    coverage_status
fi

# ============================================================
# Collection Manager
# ============================================================

assert_node_absent /collection_manager

start_process collection_manager \
  ros2 run collection_manager collection_manager \
  --ros-args \
  -p recording_enabled:="${record}" \
  -p record_topics:="/livox/lidar /livox/lidar_valid /livox/imu /livox/imu_base /tf /tf_static /decxin_camera/image_compressed /decxin_camera/camera_info /glim_ros/odom /glim_ros/points /glim_ros/map" \
  -p compression_mode:=file \
  -p compression_format:=zstd \
  -p session_root:="${ROOT}/mapping_sessions"

if [[ "${record}" == true ]]; then
  export FARM_COLLECTION_UI_ONLY_DEFAULT=false
else
  export FARM_COLLECTION_UI_ONLY_DEFAULT=true
fi

# ============================================================
# Sensor Health Monitor
# ============================================================

assert_node_absent /sensor_health_monitor

health_args=(
  ros2 run sensor_health_monitor sensor_health_monitor
  --ros-args
  -p lidar_points_topic:=/livox/lidar
  -p imu_topic:=/livox/imu_base
  -p camera_image_topic:=/decxin_camera/image_raw
  -p camera_info_topic:=/decxin_camera/camera_info
)

start_process sensor_health_monitor \
  "${health_args[@]}"

# ============================================================
# Wait for collection services
# ============================================================

deadline=$((SECONDS + WAIT_SECONDS))

while ((SECONDS < deadline)); do
  if timeout 3s \
    ros2 service type /collection/preflight \
    2>/dev/null |
    grep -Fxq collection_interfaces/srv/PreflightCollection; then

    break
  fi

  sleep 1
done

((SECONDS < deadline)) ||
  die "Collection UI services did not become ready"

# ============================================================
# UI
# ============================================================

if [[ "${headless}" == false ]]; then
  [[ -n "${DISPLAY:-}" ]] ||
    die "DISPLAY is unset; use --headless"

  assert_node_absent /rviz

  start_process rqt \
    rqt \
    --standalone \
    collection_rqt_panel.panel.CollectionPanel

  rviz_args=(
    rviz2
    -d "${RVIZ_CONFIG}"
  )

  [[ "${no_glim}" == true ]] &&
    rviz_args+=(-f livox_frame)

  start_process rviz \
    "${rviz_args[@]}"
fi

# ============================================================
# Ready
# ============================================================

echo
echo "Integrated stack is ready."
echo "Runtime directory: ${runtime_dir}"
echo
if [[ "${record}" == true ]]; then
  echo "Recording is ENABLED: uncheck UI only in the Collection panel to save a reusable session."
else
  echo "Recording is disabled. Use --record to enable reusable session capture."
fi
echo "Collection Manager and RQT Collection Panel are ready."
echo
echo "Press Ctrl+C for an orderly shutdown."

while true; do
  sleep 2

  for index in "${!child_pids[@]}"; do
    if ! kill -0 "${child_pids[index]}" 2>/dev/null; then
      die "${child_names[index]} exited unexpectedly; inspect ${runtime_dir}/${child_names[index]}.log"
    fi
  done
done
