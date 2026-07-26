#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash; source /workspace/farm_ws/install/setup.bash
export ROS_LOG_DIR=/workspace/farm_ws/logs/ros; mkdir -p "$ROS_LOG_DIR"
rqt --standalone collection_rqt_panel.panel.CollectionPanel
