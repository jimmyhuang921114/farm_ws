#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE="$ROOT/config/udev/99-fdilink-ahrs.rules"
DEST=/etc/udev/rules.d/99-fdilink-ahrs.rules

if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 2
fi
if grep -Eq 'REPLACE|PLACEHOLDER|0000' "$RULE"; then
  echo "ERROR: refusing to install a placeholder VID/PID rule." >&2
  exit 3
fi
if ! grep -q 'idVendor}=="10c4"' "$RULE" || ! grep -q 'idProduct}=="ea60"' "$RULE"; then
  echo "ERROR: rule does not contain the verified 10c4:ea60 device IDs." >&2
  exit 4
fi
install -o root -g root -m 0644 "$RULE" "$DEST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "Installed $DEST. Reconnect the FDILINK IMU."
if [[ -e /dev/ttyUSB0 ]]; then
  ls -l /dev/ttyUSB0
  stat -c 'owner=%U group=%G mode=%a' /dev/ttyUSB0
else
  echo "/dev/ttyUSB0 is not currently present; verify after reconnecting."
fi
