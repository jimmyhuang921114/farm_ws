#!/usr/bin/env bash
set -o pipefail

ROOT="${1:-/workspace/farm_ws/mapping_sessions}"

# ROS setup files may reference unset variables.
set +u
source /opt/ros/humble/setup.bash

if [ -f /workspace/farm_ws/install/setup.bash ]; then
    source /workspace/farm_ws/install/setup.bash
fi
set -u

echo "Session root: $ROOT"

for session in "$ROOT"/*; do
    [ -d "$session" ] || continue

    echo
    echo "============================================================"
    echo "SESSION: $(basename "$session")"
    echo "PATH:    $session"
    echo "============================================================"

    mcap_count="$(find "$session" -type f -name '*.mcap' | wc -l)"
    traj_count="$(find "$session" -type f -name 'traj_lidar.txt' | wc -l)"

    echo "MCAP files:        $mcap_count"
    echo "GLIM trajectories: $traj_count"

    if [ -f "$session/session.yaml" ]; then
        echo
        echo "--- session.yaml ---"
        sed -n '1,120p' "$session/session.yaml"
    fi

    found_bag=0

    while IFS= read -r metadata; do
        [ -n "$metadata" ] || continue

        found_bag=1
        bag_dir="$(dirname "$metadata")"

        echo
        echo "--- ros2 bag info: $bag_dir ---"

        if ! ros2 bag info "$bag_dir"; then
            echo "[ERROR] Could not read bag: $bag_dir"
        fi
    done < <(find "$session" -type f -name metadata.yaml | sort)

    if [ "$found_bag" -eq 0 ]; then
        echo
        echo "No rosbag metadata.yaml found."
    fi

    found_traj=0

    while IFS= read -r trajectory; do
        [ -n "$trajectory" ] || continue

        found_traj=1

        echo
        echo "--- GLIM trajectory ---"
        echo "$trajectory"
        echo "Lines: $(wc -l < "$trajectory")"
        echo "First two lines:"
        head -2 "$trajectory"
        echo "Last two lines:"
        tail -2 "$trajectory"
    done < <(find "$session" -type f -name traj_lidar.txt | sort)

    if [ "$found_traj" -eq 0 ]; then
        echo
        echo "No traj_lidar.txt found."
    fi
done
