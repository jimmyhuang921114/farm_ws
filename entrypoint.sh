#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [[ -f /workspace/farm_ws/install/setup.bash ]]; then
  source /workspace/farm_ws/install/setup.bash
fi
exec "$@"
