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

wait_for_node() {
  local node_name="$1"
  local attempt
  for ((attempt = 1; attempt <= 20; attempt++)); do
    if ros2 node list 2>/dev/null | grep -Eq "(^|/)${node_name}$"; then
      printf 'Node %-32s OK\n' "$node_name"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${node_name} was not visible in the ROS graph after 20 seconds." >&2
  echo "Nodes currently visible:" >&2
  ros2 node list >&2 || true
  return 1
}

echo "LunaBot Phase 1 live-interface smoke test"
for node_name in \
  command_guard \
  obstacle_monitor \
  lunabot_phase1_bridge \
  lunabot_phase1_camera_bridge \
  lunabot_phase1_lidar_bridge; do
  wait_for_node "$node_name"
done

assert_type /cmd_vel geometry_msgs/msg/Twist
assert_type /cmd_vel_sim geometry_msgs/msg/Twist
assert_type /odom nav_msgs/msg/Odometry
assert_type /camera/image_raw sensor_msgs/msg/Image
assert_type /camera/camera_info sensor_msgs/msg/CameraInfo
assert_type /scan sensor_msgs/msg/LaserScan
assert_type /tf_static tf2_msgs/msg/TFMessage
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
