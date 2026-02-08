#!/usr/bin/env bash
# Launch the full SITL stack (Gazebo + ROS 2 bridge + PID controller).
#
# Usage:
#   ./run_sitl.sh                     # GUI mode, empty_flat world
#   ./run_sitl.sh --headless          # No GUI (CI / Docker / no display)
#   ./run_sitl.sh --world outdoor_obstacles
#   ./run_sitl.sh --hover 2.0         # Hover at 2 m instead of default 1 m
#
# Prerequisites:
#   - ROS 2 Jazzy installed at /opt/ros/jazzy
#   - Workspace built: cd simulation/ros2_ws && colcon build --symlink-install
#   - For GUI mode on WSL2: WSLg or an X server (VcXsrv / X410)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$SIM_DIR/ros2_ws"

# ── Defaults ──────────────────────────────────────────────────────────────
HEADLESS=false
WORLD="empty_flat"
HOVER_HEIGHT="1.0"

# ── Parse arguments ───────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless|-s)  HEADLESS=true; shift ;;
        --world|-w)     WORLD="$2"; shift 2 ;;
        --hover|-z)     HOVER_HEIGHT="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--headless] [--world NAME] [--hover HEIGHT]"
            exit 0 ;;
        *)  echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Check workspace ──────────────────────────────────────────────────────
if [[ ! -d "$WS_DIR/install" ]]; then
    echo "Workspace not built. Building now..."
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    cd "$WS_DIR"
    colcon build --symlink-install
    cd -
fi

# ── Source environments ──────────────────────────────────────────────────
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/local_setup.bash"

# ── Launch ───────────────────────────────────────────────────────────────
if [[ "$HEADLESS" == "true" ]]; then
    echo "Launching headless SITL (world=$WORLD, hover=$HOVER_HEIGHT m)..."
    ros2 launch uav_sim sitl_headless.launch.py \
        world:="$WORLD" \
        hover_height:="$HOVER_HEIGHT"
else
    echo "Launching SITL with GUI (world=$WORLD, hover=$HOVER_HEIGHT m)..."
    ros2 launch uav_sim sitl.launch.py \
        world:="$WORLD" \
        hover_height:="$HOVER_HEIGHT"
fi
