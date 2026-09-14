#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LIVOX_CONFIG="${ROOT}/src/ws_livox/src/livox_ros_driver2/config/MID360_config.json"

usage() {
  cat <<'EOF' >&2
Usage: sudo docker/setup_livox_network.sh

Optional overrides:
  LIVOX_INTERFACE=ethX
  LIVOX_HOST_IP=x.x.x.x
  LIVOX_HOST_CIDR=x.x.x.x/24
  LIVOX_IP=x.x.x.x
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

valid_ipv4() {
  [[ "$1" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]
}

iface_allowed() {
  local iface=$1

  [[ "${iface}" != lo ]] || return 1
  [[ "${iface}" != docker0 ]] || return 1
  [[ "${iface}" != br-* ]] || return 1
  [[ "${iface}" != veth* ]] || return 1

  return 0
}

first_iface_cidr() {
  local iface=$1

  ip -o -4 address show dev "${iface}" scope global |
    awk '{print $4}' |
    head -n1
}

first_iface_ipv4() {
  local iface=$1

  first_iface_cidr "${iface}" |
    cut -d/ -f1
}

config_lidar_ip() {
  [[ -r "${LIVOX_CONFIG}" ]] || return 0

  python3 - "${LIVOX_CONFIG}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    config = json.load(stream)

for entry in config.get("MID360", {}).get("host_net_info", []):
    for ip in entry.get("lidar_ip", []):
        if ip:
            print(ip)
            raise SystemExit(0)

for entry in config.get("lidar_configs", []):
    ip = entry.get("ip")
    if ip:
        print(ip)
        raise SystemExit(0)
PY
}

iface_for_ip_route() {
  local target_ip=$1

  ip -o route get "${target_ip}" 2>/dev/null |
    awk '
      {
        for (i = 1; i <= NF; i++) {
          if ($i == "dev" && (i + 1) <= NF) {
            print $(i + 1)
            exit
          }
        }
      }
    '
}

derive_host_cidr() {
  local lidar_ip=$1

  awk -F. '{printf "%s.%s.%s.1/24\n", $1, $2, $3}' <<<"${lidar_ip}"
}

if [[ $# -ne 0 ]]; then
  usage
  exit 2
fi

if [[ -e /.dockerenv ]]; then
  die "run this helper on the host, not inside Docker"
fi

if [[ ${EUID} -ne 0 ]]; then
  die "network configuration requires root. Run: sudo $0"
fi

command -v ip >/dev/null ||
  die "required command is missing: ip"

command -v python3 >/dev/null ||
  die "required command is missing: python3"

lidar_ip="${LIVOX_IP:-$(config_lidar_ip)}"

if [[ -n "${lidar_ip}" ]]; then
  valid_ipv4 "${lidar_ip}" ||
    die "LIVOX_IP/config LiDAR IP is not a valid IPv4 address: ${lidar_ip}"
fi

interface="${LIVOX_INTERFACE:-}"

if [[ -z "${interface}" && -n "${lidar_ip}" ]]; then
  interface="$(iface_for_ip_route "${lidar_ip}")"
fi

if [[ -z "${interface}" ]]; then
  while read -r candidate; do
    iface_allowed "${candidate}" || continue
    [[ "$(cat "/sys/class/net/${candidate}/carrier" 2>/dev/null || echo 0)" == "1" ]] || continue

    interface="${candidate}"
    break
  done < <(ip -o link show | awk -F': ' '{print $2}' | cut -d@ -f1)
fi

[[ -n "${interface}" ]] ||
  die "could not find a usable Ethernet interface; set LIVOX_INTERFACE"

ip link show dev "${interface}" >/dev/null 2>&1 ||
  die "interface does not exist: ${interface}"

iface_allowed "${interface}" ||
  die "refusing excluded interface: ${interface}"

carrier="$(cat "/sys/class/net/${interface}/carrier" 2>/dev/null || echo 0)"
if [[ "${carrier}" != "1" ]]; then
  die "${interface} has NO-CARRIER. Connect and power the MID-360 first"
fi

current_cidr="$(first_iface_cidr "${interface}")"

if [[ -n "${LIVOX_HOST_CIDR:-}" ]]; then
  host_cidr="${LIVOX_HOST_CIDR}"
elif [[ -n "${LIVOX_HOST_IP:-}" ]]; then
  host_cidr="${LIVOX_HOST_IP}/24"
elif [[ -n "${current_cidr}" ]]; then
  host_cidr="${current_cidr}"
elif [[ -n "${lidar_ip}" ]]; then
  host_cidr="$(derive_host_cidr "${lidar_ip}")"
else
  die "no host IPv4 is configured and no LiDAR IP is known; set LIVOX_HOST_CIDR"
fi

host_ip="${host_cidr%/*}"
valid_ipv4 "${host_ip}" ||
  die "host IPv4 is not valid: ${host_ip}"

ip link set dev "${interface}" up

if [[ "${current_cidr}" != "${host_cidr}" ]]; then
  ip address replace "${host_cidr}" dev "${interface}"
  echo "Configured ${host_cidr} on ${interface}."
else
  echo "Keeping existing ${host_cidr} on ${interface}."
fi

echo "Livox host network"
echo "------------------"
echo "Interface : ${interface}"
echo "Host CIDR : ${host_cidr}"
echo "LiDAR IP  : ${lidar_ip:-unknown}"
echo
echo "Addresses:"
ip -br address show dev "${interface}"
echo "Routes:"
ip route show dev "${interface}" || true

if [[ -n "${lidar_ip}" ]] && command -v ping >/dev/null; then
  echo "MID-360 ICMP diagnostic:"
  ping -I "${interface}" -c 1 -W 1 "${lidar_ip}" || true
fi
