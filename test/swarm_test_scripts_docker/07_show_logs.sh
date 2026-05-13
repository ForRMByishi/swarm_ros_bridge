#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
for c in "$NODE_A" "$NODE_B" "$NODE_C"; do
  echo "========== $c roscore.log =========="
  docker exec "$c" bash -lc 'tail -n 80 /tmp/roscore.log || true'
  echo "========== $c bridge.log =========="
  docker exec "$c" bash -lc 'tail -n 120 /tmp/bridge.log || true'
done
