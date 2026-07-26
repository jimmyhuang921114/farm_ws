#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws
./scripts/install_deps.sh
colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_MARCH_NATIVE=OFF
