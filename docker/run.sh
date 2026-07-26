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
  [[ "$(
    docker container inspect \
      --format '{{.State.Running}}' \
      "${CONTAINER_NAME}" 2>/dev/null
  )" == "true" ]]
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

  # ------------------------------------------------------------
  # X11
  # ------------------------------------------------------------
  if [[ -n "${DISPLAY:-}" ]]; then
    xhost +si:localuser:"$(id -un)" >/dev/null 2>&1 || true
    xhost +si:localuser:root >/dev/null 2>&1 || true
  fi

  # ------------------------------------------------------------
  # NVIDIA GPU
  # ------------------------------------------------------------
  gpu_args=()

  if \
    nvidia-smi >/dev/null 2>&1 &&
    docker info --format '{{json .Runtimes}}' 2>/dev/null |
      grep -q '"nvidia"'
  then
    gpu_args=(--gpus all)
  else
    echo \
      "[WARN] NVIDIA runtime unavailable; starting CPU-validation container." \
      >&2
  fi

  # ------------------------------------------------------------
  # Supplementary groups
  # ------------------------------------------------------------
  group_args=()

  dialout_gid="$(
    getent group dialout 2>/dev/null |
      cut -d: -f3 || true
  )"

  if [[ -n "${dialout_gid}" ]]; then
    group_args+=(--group-add "${dialout_gid}")
  else
    echo \
      "[WARN] Host dialout group not found; serial access may fail." \
      >&2
  fi

  video_gid="$(
    getent group video 2>/dev/null |
      cut -d: -f3 || true
  )"

  if [[ -n "${video_gid}" ]]; then
    group_args+=(--group-add "${video_gid}")
  else
    echo \
      "[WARN] Host video group not found; camera access may fail." \
      >&2
  fi

  # ------------------------------------------------------------
  # Serial devices
  # ------------------------------------------------------------
  device_args=()

  if [[ -e /dev/ttyUSB0 ]]; then
    echo "[INFO] Mapping serial device /dev/ttyUSB0"

    device_args+=(
      --device=/dev/ttyUSB0:/dev/ttyUSB0
    )
  else
    echo \
      "[WARN] FDILINK IMU not connected; starting without /dev/ttyUSB0." \
      >&2
  fi

  # ------------------------------------------------------------
  # Current V4L2 devices
  #
  # 8086       = Intel / RealSense
  # 1bcf:2cd1  = DECXIN CAMERA
  # ------------------------------------------------------------
  for video_device in /dev/video*; do
    [[ -e "${video_device}" ]] || continue

    properties="$(
      udevadm info \
        --query=property \
        --name="${video_device}" 2>/dev/null || true
    )"

    vendor_id="$(
      printf '%s\n' "${properties}" |
        sed -n 's/^ID_VENDOR_ID=//p' |
        head -n 1
    )"

    product_id="$(
      printf '%s\n' "${properties}" |
        sed -n 's/^ID_MODEL_ID=//p' |
        head -n 1
    )"

    case "${vendor_id}:${product_id}" in
      8086:*)
        echo \
          "[INFO] Mapping Intel camera: ${video_device} " \
          "(${vendor_id}:${product_id})"

        device_args+=(
          --device="${video_device}:${video_device}"
        )
        ;;

      1bcf:2cd1)
        echo \
          "[INFO] Mapping DECXIN camera: ${video_device} " \
          "(${vendor_id}:${product_id})"

        device_args+=(
          --device="${video_device}:${video_device}"
        )
        ;;

      *)
        echo \
          "[INFO] Ignoring unrelated video device: ${video_device} " \
          "(${vendor_id:-unknown}:${product_id:-unknown})"
        ;;
    esac
  done

  # ------------------------------------------------------------
  # Host /dev mirror
  #
  # The camera node can scan /host/dev/video* after a USB hotplug.
  # This avoids replacing the container's own /dev and /dev/shm.
  # ------------------------------------------------------------
  host_dev_args=(
    --volume /dev:/host/dev:rw
    --device-cgroup-rule='c 81:* rmw'
  )

  # ------------------------------------------------------------
  # Host udev database
  # ------------------------------------------------------------
  udev_args=()

  if [[ -d /run/udev ]]; then
    udev_args=(
      --volume /run/udev:/run/udev:ro
    )
  else
    echo \
      "[WARN] /run/udev is unavailable; VID/PID lookup may be limited." \
      >&2
  fi

  # ------------------------------------------------------------
  # USB bus for RealSense and other USB devices
  # ------------------------------------------------------------
  usb_args=()

  if [[ -d /dev/bus/usb ]]; then
    usb_args=(
      --volume /dev/bus/usb:/dev/bus/usb:rw
      --device-cgroup-rule='c 189:* rmw'
    )
  else
    echo \
      "[WARN] USB bus unavailable; starting without USB bus mapping." \
      >&2
  fi

  # ------------------------------------------------------------
  # Start container
  # ------------------------------------------------------------
  echo "[INFO] Starting container..."

  docker run -d \
    --name "${CONTAINER_NAME}" \
    --init \
    "${gpu_args[@]}" \
    --shm-size=4g \
    --network host \
    "${device_args[@]}" \
    "${group_args[@]}" \
    "${host_dev_args[@]}" \
    "${udev_args[@]}" \
    "${usb_args[@]}" \
    -e DISPLAY \
    -e ROS_DOMAIN_ID=40 \
    -e FARM_WS_CONTAINER_NAME=farm_ws_glim_ui_dev \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${ROOT}:/workspace/farm_ws" \
    "${IMAGE_NAME}" \
    sleep infinity >/dev/null
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
