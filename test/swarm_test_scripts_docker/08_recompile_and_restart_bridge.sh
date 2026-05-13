#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/02_compile.sh"
"$SCRIPT_DIR/04_start_ros_and_bridge.sh"
