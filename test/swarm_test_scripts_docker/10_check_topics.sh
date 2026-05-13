#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
for item in "$NODE_A:$NODE_A_IP" "$NODE_B:$NODE_B_IP" "$NODE_C:$NODE_C_IP"; do
  node="${item%%:*}"
  ip="${item##*:}"
  echo "========== topics in $node =========="
  docker exec "$node" bash -lc "source /opt/ros/noetic/setup.bash; source /root/swarm_ros_bridge_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_IP=$ip; rostopic list"
done
