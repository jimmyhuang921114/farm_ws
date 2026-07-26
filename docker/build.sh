#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="farm_ws_glim_ui:humble"

build_args=()
if [[ "${1:-}" == "--no-cache" ]]; then
  build_args+=(--no-cache)
  shift
fi
build_args+=("$@")

echo "[INFO] Building Docker image ${IMAGE_NAME}..."
if docker build \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  -t "${IMAGE_NAME}" \
  "${build_args[@]}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${ROOT}"; then
  echo "[INFO] Docker image ${IMAGE_NAME} built successfully."
else
  status=$?
  echo "[ERROR] Docker image build failed (exit status ${status})." >&2
  exit "${status}"
fi
