#!/usr/bin/env bash
set -euo pipefail

INTERFACE="${VELODYNE_INTERFACE:-enx00e04c521538}"
HOST_IP="${VELODYNE_HOST_IP:-192.168.1.10/24}"
SENSOR_IP="${VELODYNE_SENSOR_IP:-192.168.1.201}"
UDP_PORT="${VELODYNE_UDP_PORT:-2368}"

if [[ $EUID -ne 0 ]]; then
  echo "Network configuration requires root. Run: sudo $0" >&2
  exit 2
fi
if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
  echo "ERROR: Velodyne interface '$INTERFACE' does not exist." >&2
  exit 3
fi

ip link set dev "$INTERFACE" up
expected_ip="${HOST_IP%/*}"
expected_prefix="${HOST_IP#*/}"
while read -r address; do
  [[ -z "$address" || "$address" == "$HOST_IP" ]] && continue
  if [[ "${address%/*}" == "$expected_ip" ]]; then
    ip addr del "$address" dev "$INTERFACE"
  fi
done < <(ip -o -4 addr show dev "$INTERFACE" | awk '{print $4}')
if ! ip -o -4 addr show dev "$INTERFACE" | awk '{print $4}' | grep -Fxq "$expected_ip/$expected_prefix"; then
  ip addr add "$HOST_IP" dev "$INTERFACE"
fi

ip addr show dev "$INTERFACE"
if ! ping -I "$INTERFACE" -c 3 -W 1 "$SENSOR_IP"; then
  echo "ERROR: VLP-16 at $SENSOR_IP is unavailable from $INTERFACE." >&2
  exit 4
fi
if command -v tcpdump >/dev/null 2>&1; then
  echo "Checking for UDP $UDP_PORT packets (10 second timeout)..."
  if ! timeout 10s tcpdump -ni "$INTERFACE" "udp port $UDP_PORT" -c 1; then
    echo "ERROR: no UDP $UDP_PORT traffic observed on $INTERFACE." >&2
    exit 5
  fi
else
  echo "tcpdump is not installed; ping succeeded but UDP traffic was not verified." >&2
fi
