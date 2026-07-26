#!/usr/bin/env bash
set -euo pipefail

INTERFACE="${VELODYNE_INTERFACE:-enx00e04c521538}"
HOST_IP="${VELODYNE_HOST_IP:-192.168.1.10/24}"
SENSOR_IP="${VELODYNE_SENSOR_IP:-192.168.1.201}"
UDP_PORT="${VELODYNE_UDP_PORT:-2368}"

if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
  echo "ERROR: Velodyne interface '$INTERFACE' does not exist." >&2
  echo "Available interfaces:" >&2
  ip -brief link >&2
  exit 3
fi
ip addr show dev "$INTERFACE"
if ! ip -o -4 addr show dev "$INTERFACE" | awk '{print $4}' | grep -Fxq "$HOST_IP"; then
  echo "ERROR: expected host address $HOST_IP is not configured." >&2
  echo "Run: sudo ./scripts/setup_velodyne_network.sh" >&2
  exit 4
fi
if ! ping -I "$INTERFACE" -c 3 -W 1 "$SENSOR_IP"; then
  echo "ERROR: VLP-16 at $SENSOR_IP did not answer ping." >&2
  exit 5
fi
if command -v tcpdump >/dev/null 2>&1; then
  echo "Optional packet check: sudo timeout 10s tcpdump -ni $INTERFACE 'udp port $UDP_PORT' -c 20"
else
  echo "Install tcpdump for an optional UDP $UDP_PORT packet check."
fi
