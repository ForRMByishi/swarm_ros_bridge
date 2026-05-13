#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/00_env.sh"
docker rm -f "$NODE_A" "$NODE_B" "$NODE_C" >/dev/null 2>&1 || true
echo "[ok] removed $NODE_A, $NODE_B and $NODE_C"
