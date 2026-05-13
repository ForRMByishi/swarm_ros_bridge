#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
docker exec -it "$NODE_C" bash -lc "source /opt/ros/noetic/setup.bash; source /root/swarm_ros_bridge_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_IP=$NODE_C_IP; python3 /tmp/bw_sub.py"
