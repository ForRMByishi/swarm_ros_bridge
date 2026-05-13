export BRIDGE_SRC=/home/lixueyang/code/swarm_ros_bridge
export IMAGE_NAME=swarm_ros_bridge:noetic

docker rm -f node_ctrl 2>/dev/null || true

docker run -dit \
  --name node_ctrl \
  --hostname node_ctrl \
  --network swarmtest \
  --ip 172.28.0.4 \
  -v "$BRIDGE_SRC":/root/swarm_ros_bridge_ws/src/swarm_ros_bridge \
  "$IMAGE_NAME" \
  bash
