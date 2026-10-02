#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
MODE=${MODE:-simulation}
exec ros2 launch lunabot_bringup lunabot_goal.launch.py \
  mode:="$MODE" model_path:="${MODEL_PATH:-}" \
  model_config:="${MODEL_CONFIG:-}" model_checksum:="${MODEL_CHECKSUM:-}" \
  bag_path:="${BAG_PATH:-}" use_rviz:="${USE_RVIZ:-true}" "$@"
