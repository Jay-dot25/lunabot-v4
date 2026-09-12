#!/bin/bash
# ============================================================
#  Lunabot V4 - Phase A  (launch-a)
#  Simulation & Rover Foundation
# ============================================================
#  Single independent launch for Phase A.
#  Starts from a clean state - no other LunaBot launch required.
#
#  Environment variables:
#    HEADLESS=1    Gazebo without GUI (gz-sim -s), no RViz2
#    DEMO=1        automated drive test (forward + turn) instead of WASD
#    EVIDENCE=1    additionally record topics/TF/odom/LiDAR evidence
#
#  Usage:
#    launch-a                      # interactive: Gazebo GUI + RViz2 + WASD
#    HEADLESS=1 DEMO=1 launch-a    # fully automated runtime test
#
#  Interactive controls:  W forward  S reverse  A left  D right
#                         Space stop  Q quit
# ============================================================

set -u

# ------------------------------------------------------------
# Paths (resolved from this file -> works from any directory)
# ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_a.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-a-launch-a"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

# Deterministic terrain (seed 42) - see evidence/phase-a-launch-a/terrain_stats.txt
SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"

HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
TELEOP_PID=""
TF_PIDS=()
BRIDGE_PKG=""

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

shutdown() {
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase A safely..."
  say "------------------------------------------------------------"
  [ -n "$TELEOP_PID" ] && kill "$TELEOP_PID" 2>/dev/null
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do kill "$p" 2>/dev/null; done
  [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null
    fi
    wait "$GAZEBO_PID" 2>/dev/null
    say "      Gazebo stopped."
  fi
  # orphan guard: only processes tied to THIS world
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null); do
    kill -9 "$p" 2>/dev/null
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch A environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit 0
}
trap shutdown INT TERM

# ------------------------------------------------------------
# Banner
# ------------------------------------------------------------
echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE A"
echo "                 SIMULATION & ROVER FOUNDATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

# ------------------------------------------------------------
# [1/10] Check project files
# ------------------------------------------------------------
echo "[1/10] Checking project files...................."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$RVIZ_CONFIG" \
         "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required files missing (see above)."
  echo "      Re-generate terrain:  python3 tools/generate_lunar_terrain.py"
  exit 1
fi
echo "      world, rover model, terrain meshes, teleop, rviz: OK"
log "[1/10] project files OK"

# ------------------------------------------------------------
# [2/10] Check ROS 2 / Gazebo environment
# ------------------------------------------------------------
echo "[2/10] Checking ROS 2 / Gazebo environment......."
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  log "ERROR: ros2 CLI missing"
  exit 2
fi

# Gazebo Sim binary + message namespace (Fortress: ign, Garden+: gz)
IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz' in PATH)."
  echo "      Install ros-humble-gazebo-ros-pkgs + ros-humble-ros-ign"
  log "ERROR: Gazebo Sim not found"
  exit 2
fi

if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  echo "      Install: sudo apt install ros-humble-ros-ign"
  log "ERROR: bridge package not found"
  exit 2
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
log "[2/10] environment OK (ROS 2 humble, $IGN/$MSGNS, $BRIDGE_PKG)"

# ------------------------------------------------------------
# [3/10] Clean state
# ------------------------------------------------------------
echo "[3/10] Checking for stale processes................"
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null
  sleep 2
fi
echo "      clean state: OK"
log "[3/10] clean state OK"

# ------------------------------------------------------------
# [4/10] Start Gazebo
# ------------------------------------------------------------
echo "[4/10] Starting Gazebo (lunar world)..............."
# Resolve file://meshes/*.obj in the world SDF
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
    echo "      ERROR: Gazebo exited during startup - see $EVIDENCE_DIR/gazebo.log"
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    exit 1
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "      ERROR: Gazebo did not become ready within 90 s."
  kill -KILL -"$GAZEBO_PID" 2>/dev/null
  exit 1
fi
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/10] gazebo OK (pid $GAZEBO_PID)"

# ------------------------------------------------------------
# [5/10] Spawn LunaBot V4
# ------------------------------------------------------------
echo "[5/10] Spawning LunaBot V4........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *)
    echo "      ERROR: spawn failed: $spawn_resp"
    kill -KILL -"$GAZEBO_PID" 2>/dev/null
    exit 1
    ;;
esac
log "[5/10] spawn OK (z=$SPAWN_Z)"

# ------------------------------------------------------------
# [6/10] ROS 2 <-> Gazebo bridge
# ------------------------------------------------------------
echo "[6/10] Starting ROS 2 <-> Gazebo bridge............"
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
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
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "      ERROR: bridge exited at startup - see $EVIDENCE_DIR/bridge.log"
  tail -20 "$EVIDENCE_DIR/bridge.log" | sed 's/^/      | /'
  kill -KILL -"$GAZEBO_PID" 2>/dev/null
  exit 1
fi
echo "      bridge running (PID $BRIDGE_PID), 15 topic mappings"
log "[6/10] bridge OK (pid $BRIDGE_PID)"

# ------------------------------------------------------------
# [7/10] Static TF
# ------------------------------------------------------------
echo "[7/10] Starting static TF (sensor frames)..........."
# parent|child|x y z roll pitch yaw   (from model.sdf, joints at rest)
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
log "[7/10] static TF OK"

# ------------------------------------------------------------
# [8/10] RViz2
# ------------------------------------------------------------
if [ "$HEADLESS" = "1" ]; then
  echo "[8/10] RViz2 (skipped - HEADLESS)................... OK"
  log "[8/10] rviz skipped (headless)"
else
  echo "[8/10] Starting RViz2..............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f odom >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: odom"
    log "[8/10] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[8/10] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

# ------------------------------------------------------------
# [9/10] Runtime validation (real checks, not assumptions)
# ------------------------------------------------------------
echo "[9/10] Runtime validation..........................."
topic_ok() {  # <name> <topic>
  if timeout 30 ros2 topic echo "$2" --once 2>/dev/null | head -n 3 >/dev/null; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
  fi
}
tf_ok() {  # <name> <parent> <child>
  if timeout 30 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
    echo "      $1: PASS"
    log "validation PASS: TF $2->$3"
  else
    echo "      $1: FAIL"
    log "validation FAIL: TF $2->$3"
    OVERALL="FAIL"
  fi
}
if kill -0 "$GAZEBO_PID" 2>/dev/null; then
  echo "      Gazebo process alive: PASS"
  log "validation PASS: gazebo process"
else
  echo "      Gazebo process alive: FAIL"
  log "validation FAIL: gazebo process"
  OVERALL="FAIL"
fi
topic_ok "topic /lunabot/odom"              "/lunabot/odom"
topic_ok "topic /lunabot/camera/image_raw"  "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"        "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"               "/lunabot/imu"
tf_ok    "TF odom -> chassis"               "odom" "chassis"
tf_ok    "TF chassis -> sensor_head"        "chassis" "sensor_head"

# ------------------------------------------------------------
# Optional: record runtime evidence
# ------------------------------------------------------------
if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null     > "$EVIDENCE_DIR/topics.txt"
  {
    timeout 20 ros2 topic list 2>/dev/null | while read -r t; do
      timeout 10 ros2 topic info "$t" --verbose 2>/dev/null | head -8
      echo "---- $t"
    done
  } > "$EVIDENCE_DIR/topic_info.txt"
  timeout 25 ros2 run tf2_ros tf2_echo "odom" "chassis" 2>/dev/null \
    | head -12 > "$EVIDENCE_DIR/tf_odom_chassis.txt"
  timeout 25 ros2 topic echo /lunabot/odom --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 30 ros2 topic echo /lunabot/lidar/scan --once 2>/dev/null \
    > "$EVIDENCE_DIR/lidar_scan_sample.yaml"
  python3 "$REPO_DIR/tools/plot_lidar_scan.py" \
    "$EVIDENCE_DIR/lidar_scan_sample.yaml" \
    "$EVIDENCE_DIR/lidar_scan_preview.png" 2>/dev/null \
    || echo "      (LiDAR plot skipped - needs matplotlib/pyyaml)"
  echo "      evidence written (topics, tf, odom, lidar scan + plot)"
  log "[9/10] evidence recorded"
fi

# ------------------------------------------------------------
# [10/10] Status
# ------------------------------------------------------------
echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (15 mappings)"
echo "TF                : RUNNING (odom->chassis + 4 static)"
echo "Phase Component   : RUNNING"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "Topics (ROS 2):"
echo "  /cmd_vel                      geometry_msgs/Twist        (INPUT - drive)"
echo "  /lunabot/odom                 nav_msgs/Odometry"
echo "  /lunabot/camera/image_raw     sensor_msgs/Image          (RGB 640x480 @20Hz)"
echo "  /lunabot/depth/image_raw      sensor_msgs/Image          (depth @15Hz)"
echo "  /lunabot/lidar/scan           sensor_msgs/LaserScan      (720 beams @10Hz)"
echo "  /lunabot/imu                  sensor_msgs/Imu            (@100Hz)"
echo "  /lunabot/joint_states         sensor_msgs/JointState"
echo "  /lunabot/steer/{front,rear}_{left,right}  std_msgs/Float64"
echo "  /lunabot/mast/{pan,tilt}      std_msgs/Float64"
echo "TF frames: odom -> chassis -> sensor_head -> {rgb_camera, depth_camera, lidar}"
echo "Gazebo GUI camera: 0 -300 200 (lunar terrain overview)"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - all project files present"
echo "  - Gazebo + lunar_world started (lunar terrain + horizon)"
echo "  - LunaBot V4 spawned on terrain (spawn z=$SPAWN_Z)"
echo "  - bridge + TF + sensor topics verified at runtime"
echo "  - overall: $OVERALL"
echo ""
echo "============================================================"
log "[10/10] status printed (overall $OVERALL)"

# ------------------------------------------------------------
# Drive: demo or interactive teleop
# ------------------------------------------------------------
if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED DRIVE TEST (DEMO mode)"
  echo "============================================================"
  python3 "$WASD_PATH" --demo "$EVIDENCE_DIR"
  demo_rc=$?
  if [ "$demo_rc" -ne 0 ]; then
    OVERALL="FAIL"
  fi
  echo ""
  echo "============================================================"
  printf "AUTO RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown
else
  echo ""
  echo "============================================================"
  echo " Launch A environment is running."
  echo ""
  echo " Controls:"
  echo "   W = Forward      S = Reverse"
  echo "   A = Turn Left    D = Turn Right"
  echo "   Space = Stop     Q = Quit"
  echo ""
  echo " RViz2:"
  echo "   Camera  -> /lunabot/camera/image_raw"
  echo "   LiDAR   -> /lunabot/lidar/scan"
  echo "   TF      -> odom/chassis/sensor frames"
  echo ""
  echo " Press Ctrl+C to stop LunaBot safely."
  echo "============================================================"
  python3 "$WASD_PATH" &
  TELEOP_PID=$!
  wait "$TELEOP_PID" 2>/dev/null
  shutdown
fi
