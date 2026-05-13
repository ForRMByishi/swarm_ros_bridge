#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
for c in "$NODE_A" "$NODE_B" "$NODE_C"; do
  echo "========== compile in $c =========="
  docker exec "$c" bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    cd /root/swarm_ros_bridge_ws
    rm -rf build devel
    catkin_make
  '
done
