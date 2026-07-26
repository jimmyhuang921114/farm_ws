#!/usr/bin/env bash
set -Eeuo pipefail

readonly INTERFACE="enp5s0"
readonly HOST_CIDR="192.168.113.1/24"
readonly LIDAR_IP="192.168.113.158"

if [[ $# -ne 0 ]]; then
  echo "Usage: sudo $0" >&2
  echo "This helper is intentionally fixed to ${INTERFACE} and ${HOST_CIDR}." >&2
  exit 2
fi

if [[ -e /.dockerenv ]]; then
  echo "ERROR: run this helper on the host, not inside Docker." >&2
  exit 3
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "Network configuration requires root. Run: sudo $0" >&2
  exit 4
fi

if ! ip link show dev "${INTERFACE}" >/dev/null 2>&1; then
  echo "ERROR: required interface ${INTERFACE} does not exist." >&2
  ip -br link >&2
  exit 5
fi

carrier="$(cat "/sys/class/net/${INTERFACE}/carrier" 2>/dev/null || echo 0)"
if [[ "${carrier}" != "1" ]]; then
  echo "ERROR: ${INTERFACE} has NO-CARRIER. Connect and power the MID-360 first." >&2
  ip -br link show dev "${INTERFACE}" >&2
  exit 6
fi

# `replace` is scoped to this one address. It does not flush other addresses,
# change the default route, or touch firewall rules.
ip link set dev "${INTERFACE}" up
ip address replace "${HOST_CIDR}" dev "${INTERFACE}"

echo "Configured ${HOST_CIDR} on ${INTERFACE}."
echo "Addresses:"
ip -br address show dev "${INTERFACE}"
echo "Routes:"
ip route show
echo "MID-360 connectivity:"
ping -I "${INTERFACE}" -c 3 -W 1 "${LIDAR_IP}"

