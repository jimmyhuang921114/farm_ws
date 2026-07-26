#!/usr/bin/env bash
set -uo pipefail

status=0
echo "FDILINK serial device:"
if [[ -e /dev/ttyUSB0 ]]; then
  ls -l /dev/ttyUSB0
  udevadm info -q property -n /dev/ttyUSB0 | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_USB_DRIVER)=' || true
  if [[ ! -r /dev/ttyUSB0 || ! -w /dev/ttyUSB0 ]]; then
    echo "ERROR: current user cannot read/write /dev/ttyUSB0." >&2
    status=1
  fi
else
  echo "ERROR: /dev/ttyUSB0 is not present." >&2
  status=1
fi

echo "User groups: $(groups)"
if ! id -nG | tr ' ' '\n' | grep -Fxq dialout; then
  echo "User is not in dialout. Run: sudo usermod -aG dialout ${USER:-jimmy}" >&2
  status=1
fi

echo "Intel RealSense USB device:"
if lsusb | grep -iE '8086:0b3a|RealSense' ; then
  :
else
  echo "ERROR: Intel RealSense D435i is not visible on the USB bus." >&2
  status=1
fi
exit "$status"
