#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${ROOT}/config/livox_mid360"

usage() {
  echo "Usage: $0 SESSION_PATH [--rerun-glim]"
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
SESSION_PATH="$1"
shift
RERUN_GLIM=false
if [[ "${1:-}" == "--rerun-glim" ]]; then
  RERUN_GLIM=true
  shift
fi
[[ $# -eq 0 ]] || { usage >&2; exit 2; }

BAG_PATH="${SESSION_PATH}/rosbag"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || {
  echo "Rosbag not found: ${BAG_PATH}" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
[[ -f "${ROOT}/install/setup.bash" ]] && source "${ROOT}/install/setup.bash"
set -u

glim_pid=""
cleanup() {
  if [[ -n "${glim_pid}" ]] && kill -0 "${glim_pid}" 2>/dev/null; then
    kill -TERM "-${glim_pid}" 2>/dev/null || true
    wait "${glim_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${RERUN_GLIM}" == true ]]; then
  dump_path="${SESSION_PATH}/reconstruction_$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "${dump_path}"
  setsid ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:="${CONFIG_PATH}" \
    -p dump_path:="${dump_path}" &
  glim_pid=$!
  sleep 5
  echo "Re-running GLIM; output: ${dump_path}"
fi

echo "Playing: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock
