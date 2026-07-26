#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/farm_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/workspace/farm_ws/logs/ros
mkdir -p "$ROS_LOG_DIR"
ros2 launch collection_bringup collection_full.launch.py ui_only:=true start_rqt:=true start_rviz:=true start_image_view:=false start_glim:=false start_drivers:=false
