#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"

if [ ! -f "$BRIDGE_SRC/package.xml" ] || [ ! -f "$BRIDGE_SRC/CMakeLists.txt" ]; then
  echo "[error] $BRIDGE_SRC 不是 swarm_ros_bridge 仓库根目录"
  exit 1
fi
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "[error] 找不到 Docker 镜像：$IMAGE_NAME"
  exit 1
fi
if ! docker network inspect "$NET_NAME" >/dev/null 2>&1; then
  docker network create --subnet="$NET_SUBNET" "$NET_NAME"
fi

docker rm -f "$NODE_A" "$NODE_B" "$NODE_C" >/dev/null 2>&1 || true

start_node() {
  local name="$1" ip="$2"
  docker run -dit \
    --name "$name" \
    --hostname "$name" \
    --network "$NET_NAME" \
    --ip "$ip" \
    -v "$BRIDGE_SRC":/root/swarm_ros_bridge_ws/src/swarm_ros_bridge \
    "$IMAGE_NAME" bash >/dev/null
  echo "[ok] started $name at $ip"
}

start_node "$NODE_A" "$NODE_A_IP"
start_node "$NODE_B" "$NODE_B_IP"
start_node "$NODE_C" "$NODE_C_IP"

for c in "$NODE_A" "$NODE_B" "$NODE_C"; do
  echo "========== $c source =========="
  docker exec "$c" bash -lc 'ls -la /root/swarm_ros_bridge_ws/src/swarm_ros_bridge | head'
done

echo "[ok] containers ready"
