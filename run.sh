#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="farm_ws_glim_ui:humble"
CONTAINER_NAME="farm_ws_glim_ui_dev"
cleanup_enabled=false
cleanup_done=false

container_exists() {
  docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1
}

container_running() {
  [[ "$(docker container inspect --format '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null)" == "true" ]]
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ "${cleanup_enabled}" == true && "${cleanup_done}" == false ]]; then
    cleanup_done=true
    echo "[INFO] Shell exited; stopping container..."
    if container_exists; then
      docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
      docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    fi
    echo "[INFO] Container removed."
  fi

  exit "${status}"
}

handle_signal() {
  local signal_status="$1"
  exit "${signal_status}"
}

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "[ERROR] Docker image ${IMAGE_NAME} does not exist." >&2
  echo "[INFO] Run ${SCRIPT_DIR}/build.sh first." >&2
  exit 1
fi

trap cleanup EXIT
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM
cleanup_enabled=true

if container_running; then
  echo "[INFO] Container ${CONTAINER_NAME} is already running."
else
  if container_exists; then
    echo "[INFO] Removing stopped container ${CONTAINER_NAME}..."
    docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  if [[ -n "${DISPLAY:-}" ]]; then
    xhost +si:localuser:"$(id -un)" >/dev/null 2>&1 || true
  fi

  gpu_args=()
  if nvidia-smi >/dev/null 2>&1 && docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q 'nvidia'; then
    gpu_args=(--gpus all)
  else
    echo "NVIDIA runtime unavailable; starting CPU-validation container." >&2
  fi

  device_args=()
  group_args=()
  usb_args=()
  if [[ -e /dev/ttyUSB0 ]]; then
    device_args=(--device=/dev/ttyUSB0:/dev/ttyUSB0)
  else
    echo "FDILINK IMU not connected; starting without /dev/ttyUSB0." >&2
  fi
  if dialout_gid="$(getent group dialout | cut -d: -f3)" && [[ -n "${dialout_gid}" ]]; then
    group_args+=(--group-add "${dialout_gid}")
  else
    echo "Host dialout group not found; serial access may require host configuration." >&2
  fi
  if video_gid="$(getent group video | cut -d: -f3)" && [[ -n "${video_gid}" ]]; then
    group_args+=(--group-add "${video_gid}")
  fi
  if [[ -d /dev/bus/usb ]]; then
    usb_args=(-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw')
  else
    echo "USB bus is unavailable; starting without RealSense USB mapping." >&2
  fi
  for video_device in /dev/video*; do
    [[ -e "${video_device}" ]] || continue
    if udevadm info -q property -n "${video_device}" 2>/dev/null | grep -q '^ID_VENDOR_ID=8086$'; then
      device_args+=(--device="${video_device}:${video_device}")
    fi
  done

  echo "[INFO] Starting container..."
  docker run -d --name "${CONTAINER_NAME}" "${gpu_args[@]}" --shm-size=4g \
    --network host "${device_args[@]}" "${group_args[@]}" "${usb_args[@]}" \
    -e DISPLAY -e ROS_DOMAIN_ID=40 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${ROOT}:/workspace/farm_ws" "${IMAGE_NAME}" sleep infinity >/dev/null
fi

echo "[INFO] Opening interactive shell..."
shell_status=0
docker exec -it "${CONTAINER_NAME}" bash -lc '
  source /opt/ros/humble/setup.bash
  if [[ -f /workspace/farm_ws/install/setup.bash ]]; then
    source /workspace/farm_ws/install/setup.bash
  fi
  exec bash -i
' || shell_status=$?

exit "${shell_status}"
