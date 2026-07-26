#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
cd /workspace/farm_ws
colcon build --symlink-install --event-handlers console_direct+ --packages-select collection_interfaces collection_manager sensor_health_monitor collection_rqt_panel collection_bringup preview_tools
