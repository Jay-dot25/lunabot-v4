#!/usr/bin/env bash
# LunaBot Phase 1 (historical project label: Launch A, now Launch 1).
set -euo pipefail
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "BLOCKED: ROS 2 is not sourced and /opt/ros/humble/setup.bash is absent." >&2
    echo "This Launch 1 profile targets ROS 2 Humble + Ignition Fortress; see README.md." >&2
    exit 2
  fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "BLOCKED: ros2 is not available in PATH (ROS_DISTRO=${ROS_DISTRO})." >&2
  exit 2
fi
if ! command -v ign >/dev/null 2>&1; then
  echo "BLOCKED: Ignition Fortress CLI (ign gazebo) is not available in PATH." >&2
  exit 2
fi

exec ros2 launch lunabot_gazebo launch_1.launch.py "$@"
