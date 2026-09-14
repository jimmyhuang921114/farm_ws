#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
set -u
export PATH="/usr/bin:/bin:${PATH}"

LIVOX_CONFIG="${WORKSPACE_DIR}/src/ws_livox/src/livox_ros_driver2/config/MID360_config.json"
LIVOX_NODE="${WORKSPACE_DIR}/build/livox_ros_driver2/livox_ros_driver2_node"
CAMERA_NODE="${WORKSPACE_DIR}/src/sensor_bringup/sensor_bringup/open_camera.py"
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/build/livox_ros_driver2:${WORKSPACE_DIR}/build/livox_sdk2/sdk_core:${WORKSPACE_DIR}/install/livox_sdk2/lib:${WORKSPACE_DIR}/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH:-}"

if [[ ! -x "${LIVOX_NODE}" ]]; then
  echo "Livox executable not found: ${LIVOX_NODE}" >&2
  exit 1
fi
if [[ ! -x "${CAMERA_NODE}" ]]; then
  echo "Camera executable not found: ${CAMERA_NODE}" >&2
  exit 1
fi
if [[ ! -f "${LIVOX_CONFIG}" ]]; then
  echo "Livox config not found: ${LIVOX_CONFIG}" >&2
  exit 1
fi

livox_pid=""
camera_pid=""

cleanup() {
  trap - INT TERM EXIT
  for pid in "${camera_pid}" "${livox_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${camera_pid}" "${livox_pid}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup INT TERM EXIT

echo "Starting Livox MID-360"
"${LIVOX_NODE}" \
  --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:="${LIVOX_CONFIG}" &
livox_pid=$!

sleep 1
if ! kill -0 "${livox_pid}" 2>/dev/null; then
  echo "Livox process exited during startup" >&2
  exit 1
fi

echo "Starting project USB camera"
"${CAMERA_NODE}" \
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
  -p max_read_failures:=10 &
camera_pid=$!

echo "Sensors running. Press Ctrl+C to stop both."
while kill -0 "${livox_pid}" 2>/dev/null && kill -0 "${camera_pid}" 2>/dev/null; do
  sleep 1
done

if ! kill -0 "${livox_pid}" 2>/dev/null; then
  echo "Livox process exited" >&2
fi
if ! kill -0 "${camera_pid}" 2>/dev/null; then
  echo "Camera process exited" >&2
fi
exit 1
