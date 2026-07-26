#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash; source /workspace/farm_ws/install/setup.bash
set -u
cd /workspace/farm_ws
export ROS_LOG_DIR=/workspace/farm_ws/logs/ros; mkdir -p "$ROS_LOG_DIR"
before=$(find mapping_sessions -maxdepth 1 -type d -name '*_ui_test' | wc -l)
ros2 launch collection_bringup collection_core.launch.py >logs/ui_only_launch.txt 2>&1 & pid=$!
trap 'kill $pid 2>/dev/null || true' EXIT
for _ in {1..30}; do ros2 service list | grep -q /collection/preflight && break; sleep 0.2; done
ros2 service call /collection/preflight collection_interfaces/srv/PreflightCollection "{session_name: ui_test, location: lab, duration_sec: 10.0, profile: validation, note: automated, ui_only: true}"
ros2 service call /collection/start collection_interfaces/srv/StartCollection '{}'
sleep 14
path=$(find mapping_sessions -maxdepth 1 -type d -name '*_ui_test' | sort | tail -1)
[[ -n "$path" && $(find mapping_sessions -maxdepth 1 -type d -name '*_ui_test' | wc -l) -gt $before ]]
grep -q 'mode: ui_only' "$path/session.yaml"; grep -q 'contains_mcap: false' "$path/session.yaml"
grep -q 'Session completed' "$path/markers/events.json"
if find "$path" -name '*.mcap' -print -quit | grep -q .; then echo 'Unexpected fake MCAP' >&2; exit 1; fi
echo "UI_ONLY_SESSION=$path"
