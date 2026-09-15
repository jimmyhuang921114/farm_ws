#!/usr/bin/env bash

set -e

IMAGE_NAME="camera_yolo_tracker:humble"

USER_UID="$(id -u)"
USER_GID="$(id -g)"

echo "============================================================"
echo "Building Camera YOLO Tracker"
echo "============================================================"
echo "Image : ${IMAGE_NAME}"
echo "UID   : ${USER_UID}"
echo "GID   : ${USER_GID}"
echo "============================================================"

docker build \
    --build-arg USERNAME=work \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    -t "${IMAGE_NAME}" \
    .

echo
echo "[INFO] Build completed."
echo "[INFO] Image: ${IMAGE_NAME}"