#!/usr/bin/env bash
# Build the ROS 2 workspace for the SITL simulation.
#
# Usage:
#   ./build_ws.sh            # Normal build
#   ./build_ws.sh --clean    # Clean build (remove build/install/log first)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"

if [[ "${1:-}" == "--clean" ]]; then
    echo "Cleaning workspace..."
    rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

cd "$WS_DIR"
echo "Building workspace at $WS_DIR ..."
colcon build --symlink-install
echo "Build complete. Source with: source $WS_DIR/install/local_setup.bash"
