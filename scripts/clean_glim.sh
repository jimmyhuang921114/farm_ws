#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws
mapfile -t packages < <(colcon list | awk '$1=="glim" || $1=="glim_ros" {print $1}')
[[ ${#packages[@]} -eq 2 ]] || { echo 'Expected real packages glim and glim_ros' >&2; exit 1; }
for p in "${packages[@]}"; do rm -rf "build/$p" "install/$p"; done
