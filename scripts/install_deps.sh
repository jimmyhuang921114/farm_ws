#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace/farm_ws
mkdir -p logs
rosdep update 2>&1 | tee logs/rosdep_update.txt || true
set +e
rosdep install --from-paths src --ignore-src -r -y 2>&1 | tee logs/rosdep_install.txt
s=${PIPESTATUS[0]}; set -e
if ((s)); then echo "rosdep failed; inspect unresolved keys above. No blanket skip-key was used." >&2; exit "$s"; fi
