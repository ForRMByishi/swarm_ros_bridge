#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
"$SCRIPT_DIR/01_start_containers.sh"
"$SCRIPT_DIR/02_compile.sh"
"$SCRIPT_DIR/03_write_test_files.sh"
"$SCRIPT_DIR/04_start_ros_and_bridge.sh"
echo
echo "[topology]"
echo "  $NODE_A($NODE_A_IP): publish /string -> ZMQ tcp://*:$BRIDGE_PORT"
echo "  $NODE_B($NODE_B_IP): subscribe node_a:$BRIDGE_PORT -> /string_recv"
echo "  $NODE_C($NODE_C_IP): subscribe node_a:$BRIDGE_PORT -> /string_recv"
echo
echo "[next]"
echo "终端 1：$SCRIPT_DIR/05_run_subscriber_b.sh"
echo "终端 2：$SCRIPT_DIR/05_run_subscriber_c.sh"
echo "终端 3：$SCRIPT_DIR/06_run_publisher.sh"
