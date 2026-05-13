#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
mkdir -p "$BRIDGE_SRC/config" "$BRIDGE_SRC/launch"

cat > "$BRIDGE_SRC/config/bw_node_a.yaml" <<EOF
IP:
  self: '*'
  node_a: $NODE_A_IP
  node_b: $NODE_B_IP

### optional remote control server for simulated node-level network fault
control_enabled: true # disabled by default to avoid port conflicts on one host
control_node_id: node_a # optional logical node id used by trigger_fault.py --target
control_bind_ip: '*' # usually '*' for WiFi LAN tests
control_bind_port: 3999 # each bridge process on the same host needs a different port
control_token: "" # optional shared token; empty means no token check

send_topics:
  - topic_name: /string
    msg_type: std_msgs/String
    max_freq: 100000
    srcIP: self
    srcPort: $BRIDGE_PORT
    fault_policy: buffer
EOF
if [ "$MAX_BANDWIDTH_KBPS" != "0" ]; then
cat >> "$BRIDGE_SRC/config/bw_node_a.yaml" <<EOF
    max_bandwidth_kbps: $MAX_BANDWIDTH_KBPS
EOF
fi
cat >> "$BRIDGE_SRC/config/bw_node_a.yaml" <<'EOF'

recv_topics: []
EOF

cat > "$BRIDGE_SRC/config/bw_node_b.yaml" <<EOF
IP:
  self: '*'
  node_a: $NODE_A_IP
  node_b: $NODE_B_IP

### optional remote control server for simulated node-level network fault
control_enabled: true # disabled by default to avoid port conflicts on one host
control_node_id: node_a # optional logical node id used by trigger_fault.py --target
control_bind_ip: '*' # usually '*' for WiFi LAN tests
control_bind_port: 3999 # each bridge process on the same host needs a different port
control_token: "" # optional shared token; empty means no token check

send_topics: []

recv_topics:
  - topic_name: /string_recv
    msg_type: std_msgs/String
    max_freq: 0
    srcIP: node_a
    srcPort: $BRIDGE_PORT
    fault_policy: buffer
EOF

cat > "$BRIDGE_SRC/config/bw_node_c.yaml" <<EOF
IP:
  self: '*'
  node_a: $NODE_A_IP
  node_c: $NODE_C_IP

send_topics: []

### optional remote control server for simulated node-level network fault
control_enabled: true # disabled by default to avoid port conflicts on one host
control_node_id: node_a # optional logical node id used by trigger_fault.py --target
control_bind_ip: '*' # usually '*' for WiFi LAN tests
control_bind_port: 3999 # each bridge process on the same host needs a different port
control_token: "" # optional shared token; empty means no token check

recv_topics:
  - topic_name: /string_recv
    msg_type: std_msgs/String
    max_freq: 0
    srcIP: node_a
    srcPort: $BRIDGE_PORT
    fault_policy: buffer
EOF

cat > "$BRIDGE_SRC/launch/bw_node_a.launch" <<'EOF'
<launch>
  <node pkg="swarm_ros_bridge" type="bridge_node" name="swarm_bridge" output="screen">
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/bw_node_a.yaml" />
  </node>
</launch>
EOF
cat > "$BRIDGE_SRC/launch/bw_node_b.launch" <<'EOF'
<launch>
  <node pkg="swarm_ros_bridge" type="bridge_node" name="swarm_bridge" output="screen">
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/bw_node_b.yaml" />
  </node>
</launch>
EOF
cat > "$BRIDGE_SRC/launch/bw_node_c.launch" <<'EOF'
<launch>
  <node pkg="swarm_ros_bridge" type="bridge_node" name="swarm_bridge" output="screen">
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/bw_node_c.yaml" />
  </node>
</launch>
EOF

cat > /tmp/bw_pub.py <<'PYEOF'
#!/usr/bin/env python3
import sys, time, rospy
from std_msgs.msg import String

def main():
    rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else 1000.0
    payload_bytes = int(sys.argv[2]) if len(sys.argv) > 2 else 2048
    rospy.init_node('bw_pub', anonymous=True)
    pub = rospy.Publisher('/string', String, queue_size=1000)
    rate = rospy.Rate(rate_hz)
    seq = 0
    time.sleep(1.0)
    while not rospy.is_shutdown():
        prefix = f'{seq:012d},{time.time_ns()},'
        data = prefix + ('x' * max(0, payload_bytes - len(prefix)))
        pub.publish(String(data=data))
        seq += 1
        rate.sleep()
if __name__ == '__main__':
    main()
PYEOF
chmod +x /tmp/bw_pub.py
docker cp /tmp/bw_pub.py "$NODE_A":/tmp/bw_pub.py

cat > /tmp/bw_sub.py <<'PYEOF'
#!/usr/bin/env python3
import time, rospy
from std_msgs.msg import String
bytes_sum = 0
msg_sum = 0
last_seq = None
lost_sum = 0
first_seq = None
last_seen_seq = None

def cb(msg):
    global bytes_sum, msg_sum, last_seq, lost_sum, first_seq, last_seen_seq
    data = msg.data
    bytes_sum += len(data.encode('utf-8'))
    msg_sum += 1
    try:
        seq = int(data.split(',', 1)[0])
        if first_seq is None:
            first_seq = seq
        if last_seq is not None and seq > last_seq + 1:
            lost_sum += seq - last_seq - 1
        last_seq = seq
        last_seen_seq = seq
    except Exception:
        pass

def main():
    global bytes_sum, msg_sum, lost_sum, first_seq, last_seen_seq
    rospy.init_node('bw_sub', anonymous=True)
    rospy.Subscriber('/string_recv', String, cb, queue_size=10000)
    last_t = time.time()
    while not rospy.is_shutdown():
        time.sleep(1.0)
        now = time.time()
        dt = now - last_t
        mbps = bytes_sum * 8.0 / dt / 1e6
        mps = msg_sum / dt
        seq_range = 'none' if last_seen_seq is None else f'{first_seq}->{last_seen_seq}'
        print(f'recv_payload={mbps:.3f} Mbps, recv_rate={mps:.1f} msg/s, lost_seq={lost_sum}, seq_range={seq_range}', flush=True)
        bytes_sum = msg_sum = lost_sum = 0
        first_seq = last_seen_seq = None
        last_t = now
if __name__ == '__main__':
    main()
PYEOF
chmod +x /tmp/bw_sub.py
docker cp /tmp/bw_sub.py "$NODE_B":/tmp/bw_sub.py
docker cp /tmp/bw_sub.py "$NODE_C":/tmp/bw_sub.py

echo "[ok] wrote configs, launch files, and pub/sub scripts"
