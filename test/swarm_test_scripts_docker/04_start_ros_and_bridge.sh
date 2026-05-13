#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"

for c in "$NODE_A" "$NODE_B" "$NODE_C"; do
  docker exec "$c" bash -lc 'pkill -f rostopic || true; pkill -f bw_pub.py || true; pkill -f bw_sub.py || true; pkill -f bridge_node || true; pkill -f roscore || true; pkill -f rosmaster || true; pkill -f rosout || true' || true
done
sleep 1

start_roscore() {
  local node="$1" ip="$2"
  docker exec -d "$node" bash -lc "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_IP=$ip; roscore > /tmp/roscore.log 2>&1"
}
start_roscore "$NODE_A" "$NODE_A_IP"
start_roscore "$NODE_B" "$NODE_B_IP"
start_roscore "$NODE_C" "$NODE_C_IP"
sleep 3

start_bridge() {
  local node="$1" ip="$2" launch="$3"
  docker exec -d "$node" bash -lc "source /opt/ros/noetic/setup.bash; source /root/swarm_ros_bridge_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_IP=$ip; roslaunch swarm_ros_bridge $launch > /tmp/bridge.log 2>&1"
}
start_bridge "$NODE_A" "$NODE_A_IP" bw_node_a.launch
start_bridge "$NODE_B" "$NODE_B_IP" bw_node_b.launch
start_bridge "$NODE_C" "$NODE_C_IP" bw_node_c.launch
sleep 2

for c in "$NODE_A" "$NODE_B" "$NODE_C"; do
  echo "========== $c bridge log =========="
  docker exec "$c" bash -lc 'tail -n 80 /tmp/bridge.log || true'
done
