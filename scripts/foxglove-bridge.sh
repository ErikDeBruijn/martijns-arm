#!/usr/bin/env bash
# Start de foxglove_bridge in de draaiende ros2-jazzy container.
#
# De bridge publiceert alle ROS2 topics op ROS_DOMAIN_ID=42 via een WebSocket
# op poort 8765. Verbind vanuit Foxglove Studio (Mac app) met:
#   ws://localhost:8765
#
# Draait in de voorgrond — stop met Ctrl+C.

set -euo pipefail

CONTAINER="ros2-jazzy"
PORT="8765"

# Controleer of de container draait, anders heeft starten geen zin.
if ! docker ps --filter "name=^${CONTAINER}$" --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Container '${CONTAINER}' draait niet. Start hem eerst met scripts/up.sh" >&2
  exit 1
fi

echo "Foxglove bridge starten in ${CONTAINER} op poort ${PORT}..."
echo "Verbind vanuit Foxglove Studio met: ws://localhost:${PORT}"
echo "Stop met Ctrl+C."
echo

# -i zodat Ctrl+C netjes doorkomt; geen -t want we hebben geen TTY nodig.
exec docker exec -i "${CONTAINER}" bash -lc "
  source /opt/ros/jazzy/setup.bash
  exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=${PORT}
"
