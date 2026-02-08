#!/usr/bin/env bash
# Launch the RL policy in Gazebo with GUI for visual evaluation.
#
# Usage:
#   ./run_rl_gazebo.sh --model /path/to/model.zip
#   ./run_rl_gazebo.sh --model training/models/hover_final.zip
#   ./run_rl_gazebo.sh --model training/models/hover_final.zip --hover 2.0
#   ./run_rl_gazebo.sh --model training/models/hover_final.zip --world outdoor_obstacles
#
# Prerequisites:
#   - ROS 2 Jazzy installed at /opt/ros/jazzy
#   - Workspace built: cd simulation/ros2_ws && colcon build --symlink-install
#   - A trained SB3 model (.zip file)
#   - For GUI on WSL2: WSLg or an X server

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$SIM_DIR/ros2_ws"
PROJECT_DIR="$(dirname "$SIM_DIR")"

# ── Defaults ──────────────────────────────────────────────────────────────
MODEL_PATH=""
WORLD="empty_flat"
HOVER_HEIGHT="1.0"

# ── Parse arguments ───────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --model|-m)     MODEL_PATH="$2"; shift 2 ;;
        --world|-w)     WORLD="$2"; shift 2 ;;
        --hover|-z)     HOVER_HEIGHT="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 --model /path/to/model.zip [--world NAME] [--hover HEIGHT]"
            exit 0 ;;
        *)  echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [[ -z "$MODEL_PATH" ]]; then
    echo "Error: --model is required."
    echo "Usage: $0 --model /path/to/model.zip [--world NAME] [--hover HEIGHT]"
    exit 1
fi

# Resolve relative model path to absolute
if [[ ! "$MODEL_PATH" = /* ]]; then
    MODEL_PATH="$PROJECT_DIR/$MODEL_PATH"
fi

# Check model file exists (with or without .zip extension)
if [[ ! -f "$MODEL_PATH" && ! -f "${MODEL_PATH}.zip" ]]; then
    echo "Error: Model not found at $MODEL_PATH (or ${MODEL_PATH}.zip)"
    exit 1
fi

# ── Check workspace ──────────────────────────────────────────────────────
if [[ ! -d "$WS_DIR/install" ]]; then
    echo "Workspace not built. Building now..."
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    cd "$WS_DIR"
    colcon build --symlink-install
    cd - > /dev/null
fi

# ── Source environments ──────────────────────────────────────────────────
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/local_setup.bash"

# ── Launch ───────────────────────────────────────────────────────────────
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  RL Policy → Gazebo (GUI mode)                             ║"
echo "║  Model:  $MODEL_PATH"
echo "║  World:  $WORLD"
echo "║  Hover:  ${HOVER_HEIGHT}m"
echo "╚══════════════════════════════════════════════════════════════╝"

ros2 launch uav_sim sitl_rl.launch.py \
    model_path:="$MODEL_PATH" \
    world:="$WORLD" \
    hover_height:="$HOVER_HEIGHT"
