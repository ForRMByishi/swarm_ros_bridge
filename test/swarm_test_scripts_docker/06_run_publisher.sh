#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
echo "PUB_RATE_HZ=$PUB_RATE_HZ, PAYLOAD_BYTES=$PAYLOAD_BYTES"
python3 - <<PY
rate = float("$PUB_RATE_HZ")
payload = int("$PAYLOAD_BYTES")
print(f"理论输入 payload 带宽约为：{rate * payload * 8 / 1e6:.3f} Mbps")
PY
docker exec -it "$NODE_A" bash -lc "source /opt/ros/noetic/setup.bash; source /root/swarm_ros_bridge_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_IP=$NODE_A_IP; python3 /tmp/bw_pub.py $PUB_RATE_HZ $PAYLOAD_BYTES"
