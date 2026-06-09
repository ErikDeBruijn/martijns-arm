#!/usr/bin/env bash
# Container-side: start de ROS2 brug-node die het PB-formaat parset
# en publiceert als /joint_states + per-joint Float32 topics.
#
# Vereist: docker container ros2-jazzy moet draaien, en op de Mac
# moet ./scripts/serial-tcp-bridge.sh draaien.

set -euo pipefail

docker exec -it ros2-jazzy bash -ic '
  cd /home/ros/ros2_ws
  python3 src/arm_serial_bridge/arm_serial_bridge/bridge_node.py \
    --host host.docker.internal --port 9999
'
