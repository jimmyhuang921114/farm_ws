#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws
mkdir -p logs
exec > >(tee logs/glim_environment.txt) 2>&1
find src/glim src/glim_ros2 -name package.xml -print
grep -R '<name>' src/glim src/glim_ros2 --include=package.xml
git -C src/glim status --short; git -C src/glim_ros2 status --short
git -C src/glim submodule status --recursive; git -C src/glim_ros2 submodule status --recursive
cmake --version; gcc --version; g++ --version; nvcc --version || true
ldconfig -p | grep -Ei 'gtsam|gtsam_points|iridescence' || true
pkg-config --list-all | grep -Ei 'gtsam|iridescence' || true
