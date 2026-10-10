#!/usr/bin/env bash
# Verify live ROS interfaces after phase1.launch.py is running.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available. Source /opt/ros/humble/setup.bash first." >&2
  exit 2
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ERROR: ROS_DISTRO is unset. Source ROS 2 Humble and this workspace." >&2
  exit 2
fi
if [[ "${ROS_DISTRO}" != "humble" ]]; then
  echo "ERROR: this restart workspace targets ROS 2 Humble; ROS_DISTRO=${ROS_DISTRO}." >&2
  exit 2
fi

wait_for_once() {
  local topic="$1"
  shift
  printf 'Waiting for a live message on %s ... ' "$topic"
  if timeout 20s ros2 topic echo "$topic" --once "$@" >/dev/null 2>&1; then
    echo "OK"
  else
    echo "FAILED"
    echo "No message received from ${topic} within 20 seconds." >&2
    return 1
  fi
}

assert_type() {
  local topic="$1"
  local expected="$2"
  local actual
  actual="$(ros2 topic type "$topic" 2>/dev/null | head -n 1 || true)"
  if [[ "$actual" != "$expected" ]]; then
    echo "ERROR: ${topic} has type '${actual:-<not visible>}' (expected ${expected})." >&2
    return 1
  fi
  printf 'Type %-28s %s\n' "$topic" "$actual"
}

echo "LunaBot Phase 1 live-interface smoke test"
ros2 node list | grep -q '/command_guard' || {
  echo "ERROR: command_guard is not running." >&2
  exit 1
}
ros2 node list | grep -q '/obstacle_monitor' || {
  echo "ERROR: obstacle_monitor is not running." >&2
  exit 1
}
for bridge_node in lunabot_phase1_bridge lunabot_phase1_camera_bridge lunabot_phase1_lidar_bridge; do
  ros2 node list | grep -q "/${bridge_node}" || {
    echo "ERROR: ${bridge_node} is not running." >&2
    exit 1
  }
done

assert_type /cmd_vel geometry_msgs/msg/Twist
assert_type /cmd_vel_sim geometry_msgs/msg/Twist
assert_type /odom nav_msgs/msg/Odometry
assert_type /camera/image_raw sensor_msgs/msg/Image
assert_type /camera/camera_info sensor_msgs/msg/CameraInfo
assert_type /scan sensor_msgs/msg/LaserScan
assert_type /obstacle_monitor/status std_msgs/msg/String
assert_type /obstacle_monitor/closest_range std_msgs/msg/Float32

# Use best-effort subscriptions for sensor streams; message bodies are discarded
# so image payloads and 720-beam scans do not flood the terminal.
wait_for_once /cmd_vel_sim
wait_for_once /clock
wait_for_once /camera/image_raw --qos-reliability best_effort
wait_for_once /camera/camera_info --qos-reliability best_effort
wait_for_once /scan --qos-reliability best_effort
wait_for_once /odom
wait_for_once /tf
wait_for_once /tf_static --qos-durability transient_local
wait_for_once /obstacle_monitor/status
wait_for_once /obstacle_monitor/closest_range

echo
echo "ROS interfaces are publishing. This smoke test does not prove wheel motion or obstacle-dependent range changes."
echo "Complete the forward/reverse/turn, command-timeout, clear-path, and approach-obstacle procedures in ${PROJECT_DIR}/README.md."
