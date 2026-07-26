#!/usr/bin/env bash
set -euo pipefail

user_name="${SENSOR_USER:-${SUDO_USER:-${USER:-jimmy}}}"
if [[ -e /dev/ttyUSB0 ]]; then
  ls -l /dev/ttyUSB0
else
  echo "FDILINK is disconnected: /dev/ttyUSB0 is absent." >&2
fi
if ! id -nG "$user_name" | tr ' ' '\n' | grep -Fxq dialout; then
  echo "Add $user_name to dialout, then log out and back in:"
  echo "  sudo usermod -aG dialout $user_name"
fi
echo "Install the verified device rule with:"
echo "  sudo ./scripts/install_fdilink_udev_rule.sh"
echo "No permissions were changed by this script."
