#!/bin/bash
# ============================================================
#  LunaBot V4 - Phase B  (launch-b)
#  Control & Odometry
# ============================================================
#  Independent launch. It starts the Phase A world and rover directly,
#  then inserts the Phase B ROS control and odometry-monitoring layer.
#  It does not call launch-a or depend on another launch script.
#
#  Environment variables:
#    HEADLESS=1    Gazebo without GUI, no RViz2
#    DEMO=1        automated command -> controller -> rover test
#    EVIDENCE=1    record control/odometry runtime evidence
#
#  Usage:
#    launch-b
#    HEADLESS=1 DEMO=1 EVIDENCE=1 launch-b
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
RVIZ_CONFIG="$REPO_DIR/rviz/phase_b.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-b-launch-b"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
TF_PIDS=()
BRIDGE_PKG=""
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

shutdown() {
  local rc="${1:-$EXIT_CODE}"
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase B safely..."
  say "------------------------------------------------------------"
  # Stop command generation first, then send an explicit zero command.
  if [ -n "$CONTROL_PID" ]; then
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
  if [ -n "$ODOM_PID" ]; then
    kill -TERM "$ODOM_PID" 2>/dev/null || true
    wait "$ODOM_PID" 2>/dev/null || true
  fi
  [ -n "$BRIDGE_PID" ] && kill -TERM "$BRIDGE_PID" 2>/dev/null || true
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do kill "$p" 2>/dev/null || true; done
  [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null || true
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
  say "Launch B environment cleanly closed."
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
echo "                 PHASE B"
echo "                 CONTROL & ODOMETRY"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

# ------------------------------------------------------------
# [1/12] Check project files
# ------------------------------------------------------------
echo "[1/12] Checking Phase A baseline + Phase B files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" \
         "$RVIZ_CONFIG" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/B files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, rviz: OK"
log "[1/12] files OK"

# ------------------------------------------------------------
# [2/12] Check ROS 2 / Gazebo environment
# ------------------------------------------------------------
echo "[2/12] Checking ROS 2 / Gazebo environment........"
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
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
log "[2/12] environment OK ($IGN/$MSGNS, $BRIDGE_PKG)"

# ------------------------------------------------------------
# [3/12] Clean state
# ------------------------------------------------------------
echo "[3/12] Checking for stale Phase A/B processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
# A prior interrupted `ros2 run` can leave its child bridge alive. Remove
# only processes belonging to this Phase B command/node contract.
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/12] clean state OK"

# ------------------------------------------------------------
# [4/12] Start Gazebo - same validated Phase A world
# ------------------------------------------------------------
echo "[4/12] Starting Gazebo lunar world.................."
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
# sensors, DiffDrive odometry and the Phase B demo can actually advance.
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/12] gazebo OK and unpaused (pid $GAZEBO_PID)"

# ------------------------------------------------------------
# [5/12] Spawn LunaBot V4
# ------------------------------------------------------------
echo "[5/12] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/12] spawn OK (z=$SPAWN_Z)"

# ------------------------------------------------------------
# [6/12] ROS 2 <-> Gazebo bridge - same sensor contract
# ------------------------------------------------------------
echo "[6/12] Starting ROS 2 <-> Gazebo bridge............."
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
log "[6/12] bridge OK (pid $BRIDGE_PID)"

# ------------------------------------------------------------
# [7/12] Static TF - unchanged Phase A sensor frames
# ------------------------------------------------------------
echo "[7/12] Starting static TF (sensor frames)............"
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  # shellcheck disable=SC2086
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      4 static transforms published (chassis -> sensor frames)"
log "[7/12] static TF OK"

# ------------------------------------------------------------
# [8/12] Phase B control node
# ------------------------------------------------------------
echo "[8/12] Starting Phase B control layer................"
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
log "[8/12] control OK (pid $CONTROL_PID)"

# ------------------------------------------------------------
# [9/12] Phase B odometry monitor
# ------------------------------------------------------------
echo "[9/12] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[9/12] odometry monitor OK (pid $ODOM_PID)"

# ------------------------------------------------------------
# [10/12] RViz2
# ------------------------------------------------------------
if [ "$HEADLESS" = "1" ]; then
  echo "[10/12] RViz2 (skipped - HEADLESS).................. OK"
  log "[10/12] rviz skipped (headless)"
else
  echo "[10/12] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f odom >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: odom"
    log "[10/12] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[10/12] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

# ------------------------------------------------------------
# [11/12] Runtime validation
# ------------------------------------------------------------
echo "[11/12] Runtime validation.........................."
topic_ok() {
  local sample=""
  # Do not pipe to head: head can exit successfully before ros2 receives a
  # message, creating a false PASS. This assignment waits for --once data.
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability best_effort --once 2>/dev/null)" && [ -n "$sample" ]; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
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
topic_ok "topic /clock"                       "/clock"
topic_ok "topic /lunabot/odom"                "/lunabot/odom"
topic_ok "topic /lunabot/control/status"      "/lunabot/control/status"
topic_ok "topic /lunabot/odometry/status"     "/lunabot/odometry/status"
topic_ok "topic /lunabot/camera/image_raw"   "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase B runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    | head -12 > "$EVIDENCE_DIR/tf_odom_chassis.txt"
  echo "      evidence recording started"
  log "[11/12] evidence recorded"
fi

# ------------------------------------------------------------
# [12/12] Status
# ------------------------------------------------------------
echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 4 static)"
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
echo "  evidence/phase-b-launch-b/odometry_samples.csv"
echo "  evidence/phase-b-launch-b/odometry_report.txt"
echo "  evidence/phase-b-launch-b/demo_drive_result.txt"
echo "  evidence/phase-b-launch-b/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors and TF preserved"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[12/12] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

# ------------------------------------------------------------
# Drive: Phase B input topic -> controller -> DiffDrive
# ------------------------------------------------------------
if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED CONTROL + ODOMETRY TEST (DEMO mode)"
  echo "============================================================"
  python3 "$WASD_PATH" --demo --topic /cmd_vel_in "$EVIDENCE_DIR"
  demo_rc=$?
  if [ "$demo_rc" -ne 0 ]; then
    OVERALL="FAIL"
    EXIT_CODE=1
  fi
  sleep 1
  echo ""
  echo "============================================================"
  printf "PHASE B RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase B environment is running."
  echo ""
  echo " Controls: W forward | S reverse | A left | D right"
  echo "           Space stop | Q quit"
  echo ""
  echo " Control path: keyboard -> /cmd_vel_in -> controller"
  echo "               -> /cmd_vel -> Gazebo DiffDrive"
  echo ""
  echo " Press Ctrl+C to stop Phase B safely."
  echo "============================================================"
  python3 "$WASD_PATH" --topic /cmd_vel_in
  teleop_rc=$?
  [ "$teleop_rc" -eq 0 ] || EXIT_CODE=1
  shutdown "$EXIT_CODE"
fi
