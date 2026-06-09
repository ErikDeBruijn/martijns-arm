#!/usr/bin/env bash
# Mac-side: maakt de USB serial van de ESP32 beschikbaar als TCP poort
# zodat de ROS2 bridge node (in de Docker container) erbij kan.
#
# Houd dit venster open. Stop met Ctrl+C.

set -euo pipefail

PORT_DEV="${PORT_DEV:-/dev/cu.usbmodem1101}"
TCP_PORT="${TCP_PORT:-9999}"

if [ ! -e "$PORT_DEV" ]; then
  echo "✗ ESP32 niet gevonden op $PORT_DEV"
  echo "  Beschikbare poorten:"; ls /dev/cu.usbmodem* 2>/dev/null || echo "  (geen)"
  exit 1
fi

echo "→ Bridge $PORT_DEV (115200 8N1) ↔ TCP :$TCP_PORT"
echo "  (ROS bridge node in container leest van host.docker.internal:$TCP_PORT)"
exec socat -d \
  TCP-LISTEN:"$TCP_PORT",reuseaddr,fork \
  FILE:"$PORT_DEV",b115200,raw,echo=0,cs8
