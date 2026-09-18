#!/bin/bash
# ============================================================
#  LunaBot V4 - Phase D  (launch-d)
#  AUTONOMOUS NAVIGATION
# ============================================================
#  Independent launch. It starts the Phase A world and rover directly,
#  then inserts the Phase D ROS control and odometry-monitoring layer.
#  It does not call launch-a or depend on another launch script.
#
#  Environment variables:
#    HEADLESS=1    Gazebo without GUI, no RViz2
#    DEMO=1        automated command -> controller -> rover test
#    AUTO_GOAL=1   enable the deterministic regression goal (DEMO implies it)
#    EVIDENCE=1    record control/odometry runtime evidence
#
#  Usage:
#    launch-d
#    HEADLESS=1 DEMO=1 EVIDENCE=1 launch-d
#
#  Interactive controls: W forward | S reverse | A left | D right
#                         Space stop | Q quit
# ============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
CONTROL_PATH="$REPO_DIR/scripts/control_odometry.py"
ODOM_PATH="$REPO_DIR/scripts/odometry_monitor.py"
SLAM_CONFIG="$REPO_DIR/config/slam_toolbox_phase_c.yaml"
NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_d.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-d-launch-d"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"
AUTO_GOAL="${AUTO_GOAL:-false}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
SLAM_PID=""
NAV_PID=""
GOAL_WAIT_PID=""
TF_PIDS=()
BRIDGE_PKG=""
MAP_SAVER_AVAILABLE=0
MAP_SAVE_DONE=0
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

# Terminate a launcher/node process group, wait briefly, then force-kill only
# that group. This prevents a Phase D relaunch from inheriting old children.
stop_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 0
  kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.25
  done
  kill -KILL -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

save_final_map() {
  [ "$MAP_SAVE_DONE" = "1" ] && return 0
  [ "$EVIDENCE" = "1" ] || return 0
  [ "$MAP_SAVER_AVAILABLE" = "1" ] || return 0
  [ -n "$SLAM_PID" ] || return 0
  MAP_SAVE_DONE=1
  say "      saving final map evidence..."
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$EVIDENCE_DIR/phase_d_map" \
    --ros-args -p save_map_timeout:=10.0 2>/dev/null \
    >> "$EVIDENCE_DIR/map_saver.log" 2>&1 || true
  if [ -s "$EVIDENCE_DIR/phase_d_map.yaml" ] && [ -s "$EVIDENCE_DIR/phase_d_map.pgm" ]; then
    say "      saved map evidence (YAML + PGM): PASS"
    log "validation PASS: final map files saved"
    return 0
  fi
  say "      saved map evidence (YAML + PGM): FAIL"
  log "validation FAIL: map saver package was available but map files are missing"
  OVERALL="FAIL"
  return 1
}

shutdown() {
  local rc="${1:-$EXIT_CODE}"
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase D safely..."
  say "------------------------------------------------------------"
  # Save after interactive/demo motion but before stopping slam_toolbox. This
  # also makes the GUI run produce final map evidence, not only an initial map.
  if ! save_final_map; then
    rc=1
  fi
  # Stop any status subscriber first so Ctrl+C cannot leave the goal wait loop
  # holding the launcher open, then stop navigation and the inherited stack.
  if [ -n "$GOAL_WAIT_PID" ]; then
    stop_group "$GOAL_WAIT_PID"
    wait "$GOAL_WAIT_PID" 2>/dev/null || true
    GOAL_WAIT_PID=""
  fi
  if [ -n "$NAV_PID" ]; then
    stop_group "$NAV_PID"
    wait "$NAV_PID" 2>/dev/null || true
  fi
  if [ -n "$SLAM_PID" ]; then
    stop_group "$SLAM_PID"
    wait "$SLAM_PID" 2>/dev/null || true
  fi
  if [ -n "$CONTROL_PID" ]; then
    stop_group "$CONTROL_PID"
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
  if [ -n "$ODOM_PID" ]; then
    stop_group "$ODOM_PID"
    wait "$ODOM_PID" 2>/dev/null || true
  fi
  [ -n "$BRIDGE_PID" ] && stop_group "$BRIDGE_PID"
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do stop_group "$p"; done
  [ -n "$RVIZ_PID" ] && stop_group "$RVIZ_PID"
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null || true
    fi
    wait "$GAZEBO_PID" 2>/dev/null || true
    say "      Gazebo stopped."
  fi
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null || true); do
    kill -9 "$p" 2>/dev/null || true
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch D environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit "$rc"
}
abort() {
  say "ERROR: $*"
  OVERALL="FAIL"
  shutdown 1
}
trap 'shutdown 130' INT TERM

# ------------------------------------------------------------
# Banner
# ------------------------------------------------------------
echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE D"
echo "                 AUTONOMOUS NAVIGATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

# ------------------------------------------------------------
# [1/14] Check project files
# ------------------------------------------------------------
echo "[1/14] Checking Phase A baseline + Phase D files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" "$SLAM_CONFIG" "$NAV_PATH" \
         "$RVIZ_CONFIG" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/D files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, planner, rviz: OK"
log "[1/14] files OK"

# ------------------------------------------------------------
# [2/14] Check ROS 2 / Gazebo environment
# ------------------------------------------------------------
echo "[2/14] Checking ROS 2 / Gazebo environment........"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
# ROS setup.bash references AMENT_TRACE_SETUP_FILES unguarded.
set +u
source /opt/ros/humble/setup.bash
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  exit 2
fi
IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz')."
  exit 2
fi
if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  exit 2
fi
if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "      ERROR: slam_toolbox not found."
  echo "      Install: sudo apt install ros-humble-slam-toolbox"
  exit 2
fi
if ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  MAP_SAVER_AVAILABLE=1
  MAP_SAVER_STATUS="available"
else
  MAP_SAVER_STATUS="not installed (map saving will be skipped)"
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
echo "      SLAM system: slam_toolbox | map saver: $MAP_SAVER_STATUS"
log "[2/14] environment OK ($IGN/$MSGNS, $BRIDGE_PKG, slam_toolbox)"

# ------------------------------------------------------------
# [3/14] Clean state
# ------------------------------------------------------------
echo "[3/14] Checking for stale Phase A/D processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
# A prior interrupted `ros2 run` can leave its child bridge alive. Remove
# only processes belonging to this Phase D command/node contract.
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py" "scripts/astar_navigation.py" "slam_toolbox.*online_async"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/14] clean state OK"

# ------------------------------------------------------------
# [4/14] Start Gazebo - same validated Phase A world
# ------------------------------------------------------------
echo "[4/14] Starting Gazebo lunar world.................."
export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
: > "$EVIDENCE_DIR/gazebo.log"
if [ "$HEADLESS" = "1" ]; then
  setsid "$IGN" gazebo -s -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
else
  setsid "$IGN" gazebo -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
fi
GAZEBO_PID=$!
ready=0
for _ in $(seq 1 90); do
  if timeout 3 "$IGN" service -s "/world/$WORLD_NAME/info" \
       --reqtype "$MSGNS.Empty" --reptype "$MSGNS.World" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    abort "Gazebo exited during startup; see $EVIDENCE_DIR/gazebo.log"
  fi
  sleep 1
done
[ "$ready" -eq 1 ] || abort "Gazebo did not become ready within 90 s"
# Gazebo Sim starts paused in server-only mode. Explicitly unpause so /clock,
# sensors, DiffDrive odometry and the Phase D demo can actually advance.
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/14] gazebo OK and unpaused (pid $GAZEBO_PID)"

# ------------------------------------------------------------
# [5/14] Spawn LunaBot V4
# ------------------------------------------------------------
echo "[5/14] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/14] spawn OK (z=$SPAWN_Z)"

# ------------------------------------------------------------
# [6/14] ROS 2 <-> Gazebo bridge - same sensor contract
# ------------------------------------------------------------
echo "[6/14] Starting ROS 2 <-> Gazebo bridge............."
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
  "/clock@rosgraph_msgs/msg/Clock[$MSGNS.Clock" \
  "/lunabot/camera/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/depth/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[$MSGNS.LaserScan" \
  "/lunabot/imu@sensor_msgs/msg/Imu[$MSGNS.IMU" \
  "/lunabot/odom@nav_msgs/msg/Odometry[$MSGNS.Odometry" \
  "/lunabot/joint_states@sensor_msgs/JointState[$MSGNS.Model" \
  "/tf@tf2_msgs/msg/TFMessage[$MSGNS.Pose_V" \
  "/lunabot/steer/front_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/front_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/pan@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/tilt@std_msgs/msg/Float64]$MSGNS.Double" \
  >> "$EVIDENCE_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3
kill -0 "$BRIDGE_PID" 2>/dev/null || abort "bridge exited; see $EVIDENCE_DIR/bridge.log"
echo "      bridge running (PID $BRIDGE_PID), 16 mappings"
log "[6/14] bridge OK (pid $BRIDGE_PID)"

# ------------------------------------------------------------
# [7/14] Static TF - unchanged Phase A sensor frames
# ------------------------------------------------------------
echo "[7/14] Starting static TF (sensor frames)............"
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
  # Gazebo Fortress scopes the LaserScan frame in the message. Keep the
  # Phase B unscoped alias and add the exact frame used by slam_toolbox.
  "sensor_head|lunabot_v4/sensor_head/lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  # shellcheck disable=SC2086
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      5 static transforms published (including scoped Gazebo LaserScan frame)"
log "[7/14] static TF OK"

# ------------------------------------------------------------
# [8/14] Phase D control node
# ------------------------------------------------------------
echo "[8/14] Starting Phase D control layer................"
: > "$EVIDENCE_DIR/control.log"
setsid python3 "$CONTROL_PATH" \
  --input-topic /cmd_vel_in --output-topic /cmd_vel \
  --max-linear 0.45 --max-angular 1.0 \
  --max-linear-accel 0.4 --max-angular-accel 0.8 \
  --watchdog-sec 0.5 --rate 30 \
  >> "$EVIDENCE_DIR/control.log" 2>&1 &
CONTROL_PID=$!
sleep 2
kill -0 "$CONTROL_PID" 2>/dev/null || abort "control node exited; see $EVIDENCE_DIR/control.log"
echo "      control running (PID $CONTROL_PID): /cmd_vel_in -> /cmd_vel"
log "[8/14] control OK (pid $CONTROL_PID)"

# ------------------------------------------------------------
# [9/14] Phase D odometry monitor
# ------------------------------------------------------------
echo "[9/14] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" --phase-label "Phase D" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[9/14] odometry monitor OK (pid $ODOM_PID)"

# ------------------------------------------------------------
# [10/14] Phase D SLAM and localization
# ------------------------------------------------------------
echo "[10/14] Starting slam_toolbox mapping................"
: > "$EVIDENCE_DIR/slam.log"
setsid ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$SLAM_CONFIG" use_sim_time:=true \
  >> "$EVIDENCE_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 4
kill -0 "$SLAM_PID" 2>/dev/null || abort "slam_toolbox exited; see $EVIDENCE_DIR/slam.log"
echo "      slam_toolbox running (PID $SLAM_PID), mapping /lunabot/lidar/scan"
log "[10/14] slam_toolbox OK (pid $SLAM_PID)"

# ------------------------------------------------------------
# [11/14] Phase D A* autonomous navigation
# ------------------------------------------------------------
echo "[11/14] Starting A* autonomous navigation..........."
: > "$EVIDENCE_DIR/navigation.log"
if [ "$DEMO" = "1" ]; then
  AUTO_GOAL=true
fi
if [ "$EVIDENCE" = "1" ] && [ "$DEMO" = "1" ]; then
  timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_before_navigation.txt" || true
fi
setsid python3 "$NAV_PATH" --ros-args \
  -p auto_goal:="$AUTO_GOAL" \
  -p map_topic:=/map -p goal_topic:=/goal_pose \
  -p path_topic:=/plan -p cmd_topic:=/cmd_vel_in \
  -p odom_topic:=/lunabot/odom \
  -p status_topic:=/lunabot/navigation/status \
  -p map_frame:=map -p unknown_is_obstacle:=true \
  -p inflation_radius:=0.25 -p auto_goal_distance:=1.5 \
  >> "$EVIDENCE_DIR/navigation.log" 2>&1 &
NAV_PID=$!
sleep 3
kill -0 "$NAV_PID" 2>/dev/null || abort "A* navigation exited; see $EVIDENCE_DIR/navigation.log"
echo "      A* navigation running (PID $NAV_PID), /plan -> /cmd_vel_in"
log "[11/14] A* navigation OK (pid $NAV_PID, auto_goal=$AUTO_GOAL)"

# ------------------------------------------------------------
# [12/14] RViz2
# ------------------------------------------------------------
if [ "$HEADLESS" = "1" ]; then
  echo "[12/14] RViz2 (skipped - HEADLESS).................. OK"
  log "[12/14] rviz skipped (headless)"
else
  echo "[12/14] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f map >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: map"
    log "[12/14] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[12/14] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

# ------------------------------------------------------------
# [13/14] Runtime validation
# ------------------------------------------------------------
echo "[13/14] Runtime validation.........................."
wait_for_goal() {
  local status_file="$EVIDENCE_DIR/goal_wait_status.txt"
  rm -f "$status_file"
  echo "      waiting for A* GOAL_REACHED..."
  # Keep the ROS subscriber out of the launcher's foreground wait. This lets
  # the INT/TERM trap run immediately and shutdown() can terminate this group.
  setsid timeout 180 ros2 topic echo /lunabot/navigation/status \
    --qos-reliability best_effort --qos-durability transient_local \
    > "$status_file" 2>/dev/null &
  GOAL_WAIT_PID=$!
  for _ in $(seq 1 180); do
    if grep -q "GOAL_REACHED" "$status_file" 2>/dev/null; then
      stop_group "$GOAL_WAIT_PID"
      wait "$GOAL_WAIT_PID" 2>/dev/null || true
      GOAL_WAIT_PID=""
      echo "      A* goal reached: PASS"
      log "validation PASS: A* GOAL_REACHED"
      return 0
    fi
    if ! kill -0 "$GOAL_WAIT_PID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  stop_group "$GOAL_WAIT_PID"
  wait "$GOAL_WAIT_PID" 2>/dev/null || true
  GOAL_WAIT_PID=""
  echo "      A* goal reached: FAIL"
  log "validation FAIL: A* did not reach goal within 180 s"
  OVERALL="FAIL"
  return 1
}

type_ok() {
  local label="$1"
  local topic="$2"
  local expected="$3"
  local value=""
  for _ in 1 2 3; do
    value="$(timeout 10 ros2 topic type "$topic" 2>/dev/null || true)"
    if printf '%s\n' "$value" | grep -qx "$expected"; then
      echo "      $label: PASS"
      log "validation PASS: type $topic=$expected"
      return 0
    fi
    sleep 1
  done
  echo "      $label: FAIL"
  log "validation FAIL: type $topic was [$value]"
  OVERALL="FAIL"
  return 1
}

topic_ok() {
  local sample=""
  local durability="volatile"
  case "$2" in
    /map|/plan|/lunabot/navigation/status) durability="transient_local" ;;
  esac
  # Do not pipe to head: head can exit successfully before ros2 receives a
  # message, creating a false PASS. This assignment waits for --once data.
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability best_effort \
      --qos-durability "$durability" --once 2>/dev/null)" && [ -n "$sample" ]; then
    # /map must be an OccupancyGrid-shaped message, not merely any message.
    if [ "$2" = "/map" ] && { [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/plan" ] && { [[ "$sample" != *"poses:"* ]] || [[ "$sample" == *"poses: []"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/goal_pose" ] && [[ "$sample" != *"pose:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/navigation/status" ] && [[ "$sample" != *"data:"* ]]; then
      sample=""
    fi
  fi
  if [ -n "$sample" ]; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
    if [ "$2" = "/lunabot/imu" ]; then
      timeout 10 "$IGN" topic -l 2>/dev/null \
        | grep -iE 'imu|inertial' > "$EVIDENCE_DIR/imu_gazebo_topics.txt" || true
      timeout 8 "$IGN" topic -e -t "/lunabot/imu" 2>/dev/null \
        > "$EVIDENCE_DIR/imu_gazebo_sample.txt" || true
      timeout 10 ros2 topic info /lunabot/imu --verbose 2>/dev/null \
        > "$EVIDENCE_DIR/imu_topic_info.txt" || true
      {
        echo "--- bridge IMU diagnostics ---"
        grep -iE 'imu|error|fail|warn' "$EVIDENCE_DIR/bridge.log" 2>/dev/null || true
        echo "--- Gazebo IMU diagnostics ---"
        grep -iE 'imu|sensor|error|fail|warn' "$EVIDENCE_DIR/gazebo.log" 2>/dev/null | tail -80 || true
      } > "$EVIDENCE_DIR/imu_diagnostics.txt"
      echo "      IMU diagnostics written to $EVIDENCE_DIR/imu_*"
    fi
  fi
}
tf_ok() {
  # Dynamic odom TF starts after DiffDrive has its first wheel update. Retry
  # the lookup so a cold Gazebo startup is not reported as a false failure.
  for _ in 1 2 3; do
    if timeout 10 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
      echo "      $1: PASS"
      log "validation PASS: TF $2->$3"
      return 0
    fi
    sleep 1
  done
  echo "      $1: FAIL"
  log "validation FAIL: TF $2->$3"
  OVERALL="FAIL"
  return 1
}
kill -0 "$GAZEBO_PID" 2>/dev/null && echo "      Gazebo process alive: PASS" || { echo "      Gazebo process alive: FAIL"; OVERALL="FAIL"; }
map_type=""
for _ in 1 2 3; do
  map_type="$(timeout 10 ros2 topic type /map 2>/dev/null || true)"
  if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
    break
  fi
  sleep 1
done
if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
  echo "      /map type nav_msgs/msg/OccupancyGrid: PASS"
  log "validation PASS: /map type nav_msgs/msg/OccupancyGrid"
else
  echo "      /map type nav_msgs/msg/OccupancyGrid: FAIL"
  log "validation FAIL: /map type was [$map_type]"
  OVERALL="FAIL"
fi
topic_ok "topic /map"                         "/map"
tf_ok "TF map -> odom" "map" "odom"
topic_ok "topic /clock"                       "/clock"
topic_ok "topic /lunabot/odom"                "/lunabot/odom"
topic_ok "topic /lunabot/control/status"      "/lunabot/control/status"
topic_ok "topic /lunabot/odometry/status"     "/lunabot/odometry/status"
topic_ok "topic /lunabot/camera/image_raw"   "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"
tf_ok "TF sensor_head -> scoped LaserScan frame" "sensor_head" "lunabot_v4/sensor_head/lidar"
type_ok "type /goal_pose geometry_msgs/PoseStamped" "/goal_pose" "geometry_msgs/msg/PoseStamped"
type_ok "type /plan nav_msgs/Path" "/plan" "nav_msgs/msg/Path"
type_ok "type navigation status std_msgs/String" "/lunabot/navigation/status" "std_msgs/msg/String"
topic_ok "topic /goal_pose"                    "/goal_pose"
topic_ok "topic /plan"                         "/plan"
topic_ok "topic /lunabot/navigation/status"    "/lunabot/navigation/status"
topic_ok "topic /cmd_vel_in"                  "/cmd_vel_in"
if [ "$DEMO" = "1" ]; then
  wait_for_goal || true
fi

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase D runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 15 ros2 topic echo /lunabot/navigation/status --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/navigation_status.txt"
  timeout 15 ros2 topic echo /goal_pose --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/goal_pose.txt"
  timeout 15 ros2 topic echo /plan --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/plan.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    > "$EVIDENCE_DIR/tf_odom_chassis.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo map odom 2>/dev/null \
    > "$EVIDENCE_DIR/tf_map_odom.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo sensor_head lunabot_v4/sensor_head/lidar 2>/dev/null \
    > "$EVIDENCE_DIR/tf_lidar_scoped.txt" || true
  timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_sample.txt"
  if [ "$DEMO" = "1" ]; then
    timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
      > "$EVIDENCE_DIR/map_after_navigation.txt" || true
    if [ -s "$EVIDENCE_DIR/map_before_navigation.txt" ] && \
       [ -s "$EVIDENCE_DIR/map_after_navigation.txt" ] && \
       ! cmp -s "$EVIDENCE_DIR/map_before_navigation.txt" "$EVIDENCE_DIR/map_after_navigation.txt"; then
      echo "      live map updates during autonomous navigation: PASS"
      log "validation PASS: map changed during A* navigation"
    else
      echo "      live map updates during autonomous navigation: FAIL"
      log "validation FAIL: map did not produce distinct A* before/after samples"
      OVERALL="FAIL"
    fi
  fi
  if [ "$MAP_SAVER_AVAILABLE" = "1" ]; then
    echo "      final map saver will run during clean shutdown"
    echo "map saver scheduled for clean shutdown" > "$EVIDENCE_DIR/map_saver.log"
  else
    echo "      map saver unavailable; map_sample.txt is retained"
    echo "map saver skipped: install ros-humble-nav2-map-server to write PGM/YAML" \
      > "$EVIDENCE_DIR/map_saver.log"
  fi
  echo "      evidence written (map, TF map->odom, odom, SLAM log)"
  log "[13/14] evidence recorded"
fi

# ------------------------------------------------------------
# [14/14] Status
# ------------------------------------------------------------
echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "SLAM              : RUNNING (slam_toolbox mapping)"
echo "Navigation        : RUNNING (A* planner + follower)"
echo "Map               : RUNNING (/map + map->odom TF)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 5 static)"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "  /cmd_vel_in                  geometry_msgs/Twist (control input)"
echo "  /cmd_vel                     geometry_msgs/Twist (smoothed output)"
echo "  /lunabot/control/status      std_msgs/String (watchdog + limits)"
echo "  /lunabot/odom                nav_msgs/Odometry (DiffDrive source)"
echo "  /lunabot/odometry/status     std_msgs/String (quality monitor)"
echo "  /map                         nav_msgs/OccupancyGrid (SLAM map)"
echo "  /goal_pose                  geometry_msgs/PoseStamped (navigation goal)"
echo "  /plan                       nav_msgs/Path (A* path)"
echo "  /lunabot/navigation/status std_msgs/String (planner state)"
echo "  map -> odom                  TF (SLAM localization)"
echo "  evidence/phase-d-launch-d/map_sample.txt"
echo "  evidence/phase-d-launch-d/phase_d_map.yaml/.pgm"
echo "  evidence/phase-d-launch-d/odometry_samples.csv"
echo "  evidence/phase-d-launch-d/odometry_report.txt"
echo "  evidence/phase-d-launch-d/demo_drive_result.txt"
echo "  evidence/phase-d-launch-d/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors, control and odometry preserved"
echo "  - one SLAM system: slam_toolbox"
echo "  - A* planner consumes /map and publishes /plan"
echo "  - navigation commands remain behind the Phase B controller"
echo "  - /map and map->odom localization outputs verified"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[14/14] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

# ------------------------------------------------------------
# ------------------------------------------------------------
# Autonomous Phase D run
# ------------------------------------------------------------
if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED A* NAVIGATION TEST (DEMO mode)"
  echo "============================================================"
  sleep 1
  if [ "$OVERALL" != "PASS" ]; then
    EXIT_CODE=1
  fi
  if ! save_final_map; then
    EXIT_CODE=1
  fi
  echo ""
  echo "============================================================"
  printf "PHASE D RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase D autonomous navigation is running."
  echo ""
  echo " A* auto goal: $AUTO_GOAL"
  echo " RViz can publish a replacement goal on /goal_pose."
  echo " Press Ctrl+C to stop Phase D safely."
  echo "============================================================"
  while true; do sleep 1; done
fi
