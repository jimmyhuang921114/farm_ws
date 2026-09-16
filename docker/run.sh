#!/usr/bin/env bash

set -e

IMAGE_NAME="camera_yolo_tracker:humble"
CONTAINER_NAME="camera_yolo_tracker"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"


echo "============================================================"
echo "Camera YOLO Tracker"
echo "============================================================"
echo "Image     : ${IMAGE_NAME}"
echo "Container : ${CONTAINER_NAME}"
echo "Workspace : ${SCRIPT_DIR}"
echo "============================================================"


# ============================================================
# Check image
# ============================================================

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "[ERROR] Docker image does not exist:"
    echo "        ${IMAGE_NAME}"
    echo
    echo "Run:"
    echo "    ./build.sh"
    exit 1
fi


# ============================================================
# If container is already running -> exec into it
# ============================================================

if docker ps \
    --format '{{.Names}}' \
    | grep -qx "${CONTAINER_NAME}"; then

    echo "[INFO] Container is already running."
    echo "[INFO] Entering existing container..."

    exec docker exec \
        -it \
        --user work \
        "${CONTAINER_NAME}" \
        bash
fi


# ============================================================
# Remove stale stopped container
# ============================================================

if docker ps -a \
    --format '{{.Names}}' \
    | grep -qx "${CONTAINER_NAME}"; then

    echo "[INFO] Removing stopped container..."
    docker rm "${CONTAINER_NAME}" >/dev/null
fi


# ============================================================
# X11
# ============================================================

if [ -n "${DISPLAY:-}" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
fi


# ============================================================
# Create and enter container
# ============================================================

echo "[INFO] Creating new container..."

exec docker run \
    --rm \
    -it \
    --name "${CONTAINER_NAME}" \
    --hostname camera-tracker \
    --net=host \
    --ipc=host \
    --gpus all \
    --privileged \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${PARENT_DIR}:/workspace" \
    -v /dev:/dev \
    "${IMAGE_NAME}" \
    bash