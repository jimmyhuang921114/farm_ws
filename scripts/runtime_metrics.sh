#!/usr/bin/env bash
set -Eeuo pipefail

interval="${1:-5}"
printf '%s\n' 'wall_time,pid,elapsed,cpu_percent,mem_percent,rss_kib,command'
while true; do
  wall_time="$(date +%s.%N)"
  ps -eo pid=,etimes=,%cpu=,%mem=,rss=,args= |
    awk -v now="${wall_time}" '
      /livox_ros_driver2_node|livox_cloud_guard|livox_cloud_monitor|imu_transform|open_camera|glim_rosnode|glim_session_supervisor|collection_manager|sensor_health_monitor/ &&
      $0 !~ /runtime_metrics|awk -v now/ {
        cmd=$6
        for (i=7; i<=NF; i++) cmd=cmd " " $i
        gsub(/,/, ";", cmd)
        print now "," $1 "," $2 "," $3 "," $4 "," $5 "," cmd
      }'
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used \
      --format=csv,noheader,nounits 2>/dev/null |
      awk '{gsub(/, /, ","); print "gpu," $0}' || true
  fi
  sleep "${interval}"
done
