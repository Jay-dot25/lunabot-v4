#!/usr/bin/env bash
# Launch the verified, standalone Phase 1 workspace without changing the
# existing Phase A-L launchers. Build the workspace once using its README.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$REPO_DIR/lunabot_phase1"
HUMBLE_SETUP="/opt/ros/humble/setup.bash"

usage() {
  cat <<'EOF'
Usage: launch-1 [ros2-launch-arguments...]

Starts the standalone clean Phase 1 Gazebo Fortress workspace in
lunabot_phase1/. This launcher is separate from launch-a through launch-l.

Build the workspace once before launching; see lunabot_phase1/README.md.
Stop the complete simulation with Ctrl+C in this terminal.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

fail() {
  local message="$1"
  local code="${2:-2}"
  printf 'launch-1: ERROR: %s\n' "$message" >&2
  exit "$code"
}

if [[ ! -f "$WORKSPACE_DIR/src/lunabot_phase1_bringup/package.xml" ]]; then
  fail "clean Phase 1 workspace not found at $WORKSPACE_DIR"
fi
if [[ ! -f "$HUMBLE_SETUP" ]]; then
  fail "ROS 2 Humble setup not found at $HUMBLE_SETUP"
fi
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO:-}" != "humble" ]]; then
  fail "ROS_DISTRO is '${ROS_DISTRO:-unset}', not 'humble'. Open a clean Humble terminal."
fi
if [[ ! -r "$WORKSPACE_DIR/install/setup.bash" ]]; then
  cat >&2 <<EOF
launch-1: ERROR: the Phase 1 workspace has not been built.
Build it once with:
  cd "$WORKSPACE_DIR"
  source "$HUMBLE_SETUP"
  colcon build --symlink-install --event-handlers console_direct+
Then run launch-1 again.
EOF
  exit 2
fi

# ROS setup files may reference unset variables; disable nounset only while
# sourcing, then restore the launcher's strict shell mode.
set +u
source "$HUMBLE_SETUP"
source "$WORKSPACE_DIR/install/setup.bash"
set -u

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  fail "sourcing the workspace did not select ROS 2 Humble"
fi
command -v ros2 >/dev/null 2>&1 || fail "ros2 CLI is unavailable after sourcing Humble"
command -v ign >/dev/null 2>&1 || fail "Gazebo Fortress 'ign' executable is unavailable"
ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1 \
  || fail "ros_ign_bridge is not installed in the sourced Humble environment"

check_workspace_package() {
  local package="$1"
  local prefix
  prefix="$(ros2 pkg prefix "$package" 2>/dev/null)" \
    || fail "$package is not built in this workspace; rebuild lunabot_phase1"
  case "$prefix" in
    "$WORKSPACE_DIR/install"|"$WORKSPACE_DIR/install/"*) ;;
    *) fail "$package resolved outside this checkout: $prefix" ;;
  esac
}

for package in \
  lunabot_phase1_bringup \
  lunabot_phase1_description \
  lunabot_phase1_simulation \
  lunabot_phase1_tools; do
  check_workspace_package "$package"
done

BRINGUP_PREFIX="$(ros2 pkg prefix lunabot_phase1_bringup)"
if [[ ! -f "$BRINGUP_PREFIX/share/lunabot_phase1_bringup/launch/phase1.launch.py" ]]; then
  fail "installed Phase 1 launch file is missing; rebuild lunabot_phase1"
fi

printf 'LunaBot clean Phase 1 (launch-1)\n'
printf 'Workspace: %s\n' "$WORKSPACE_DIR"
printf 'ROS: %s | Gazebo: Fortress\n' "$ROS_DISTRO"
printf 'Starting integrated launch; press Ctrl+C here to stop.\n\n'

cd "$WORKSPACE_DIR"
exec ros2 launch lunabot_phase1_bringup phase1.launch.py "$@"
