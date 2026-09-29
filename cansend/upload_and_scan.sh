#!/usr/bin/env bash
# Build + flash can/can.ino to the NUCLEO-F207ZG, start the serial<->can0 bridge,
# run `damiao scan`, then print the scan result and the STM32/bridge log.
#
# Usage:  ./upload_and_scan.sh            (build, flash, scan)
#         ./upload_and_scan.sh --no-flash (scan only)
#
# can0 must exist (vcan):  sudo modprobe vcan; sudo ip link add dev can0 type vcan; sudo ip link set up can0
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HOME/canbuild"
PORT="${PORT:-/dev/ttyACM0}"
OCD="$HOME/.arduino15/packages/STMicroelectronics/tools/xpack-openocd/0.12.0-6"
BRIDGE_LOG="$BUILD/bridge.log"

mkdir -p "$BUILD"

if [[ "${1:-}" != "--no-flash" ]]; then
  echo "=== Building ==="
  (cd "$HERE/can" && arduino --board STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_F207ZG \
      --pref build.path="$BUILD" --verify can.ino 2>&1 | grep -E "Sketch uses|error|Error" )

  echo "=== Flashing ==="
  "$OCD/bin/openocd" -s "$OCD/openocd/scripts" -f interface/stlink.cfg -f target/stm32f2x.cfg \
      -c "program $BUILD/can.ino.elf verify reset exit" 2>&1 | grep -E "Verified|Error|error"
fi

if ! ip link show can0 >/dev/null 2>&1; then
  echo "ERROR: can0 does not exist. Run:"
  echo "  sudo modprobe vcan; sudo ip link add dev can0 type vcan; sudo ip link set up can0"
  exit 1
fi

# Only one program can own the serial port.
pkill -f serial_can_bridge.py 2>/dev/null && sleep 0.5 || true

echo "=== Starting bridge ($PORT <-> can0) ==="
python3 -u "$HERE/serial_can_bridge.py" --port "$PORT" > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null || true' EXIT
sleep 3   # let the STM32 finish booting after the reset

echo "=== damiao scan ==="
damiao scan || true
sleep 1

echo
echo "=== STM32 / bridge log ==="
cat "$BRIDGE_LOG"
