#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws
git -C src/glim submodule update --init --recursive
git -C src/glim_ros2 submodule update --init --recursive
colcon list | tee /tmp/farm_colcon_list
grep -Ei '(^|[[:space:]])(glim|glim_ros)([[:space:]]|$)' /tmp/farm_colcon_list
mkdir -p logs
set +e
colcon build --symlink-install --event-handlers console_direct+ --packages-up-to glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_MARCH_NATIVE=OFF 2>&1 | tee logs/glim_cuda_build.txt
s=${PIPESTATUS[0]};set -e
if ((s)); then echo "CUDA build failed. Read logs/glim_cuda_build.txt; CPU fallback is intentionally not automatic." >&2; exit "$s"; fi
