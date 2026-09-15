#!/bin/bash
# ============================================================
#  LunaBot V4 - Phase K  (launch-k)
#  AUTONOMOUS NAVIGATION
# ============================================================
#  Independent launch. It starts the Phase A world and rover directly,
#  then inserts the Phase K ROS control and odometry-monitoring layer.
#  It does not call launch-a or depend on another launch script.
#
#  Environment variables:
#    HEADLESS=1    Gazebo without GUI, no RViz2
#    DEMO=1        automated command -> controller -> rover test
#    EVIDENCE=1    record control/odometry runtime evidence
#
#  Usage:
#    launch-k
#    HEADLESS=1 DEMO=1 EVIDENCE=1 launch-k
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
PERCEPTION_PATH="$REPO_DIR/scripts/terrain_segmentation.py"
MAPPER_PATH="$REPO_DIR/scripts/semantic_terrain_mapper.py"
COST_PATH="$REPO_DIR/scripts/terrain_cost_mapper.py"
TERRAIN_PLANNER_PATH="$REPO_DIR/scripts/terrain_aware_planner.py"
FOLLOWER_PATH="$REPO_DIR/scripts/terrain_path_follower.py"
REPLAN_MONITOR_PATH="$REPO_DIR/scripts/dynamic_replan_monitor.py"
EVALUATOR_PATH="$REPO_DIR/scripts/phase_k_evaluator.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_k.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-k-launch-k"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"
AUTO_GOAL="${AUTO_GOAL:-true}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
SLAM_PID=""
NAV_PID=""
PERCEPTION_PID=""
MAPPER_PID=""
COST_PID=""
TERRAIN_PLANNER_PID=""
FOLLOWER_PID=""
REPLAN_MONITOR_PID=""
EVALUATOR_PID=""
GOAL_WAIT_PID=""
TF_PIDS=()
BRIDGE_PKG=""
MAP_SAVER_AVAILABLE=0
MAP_SAVE_DONE=0
CMD_INPUT_CAPTURE_PID=""
CMD_OUTPUT_CAPTURE_PID=""
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

# Terminate a launcher/node process group, wait briefly, then force-kill only
# that group. This prevents a Phase K relaunch from inheriting old children.
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
  if [ "$MAP_SAVER_AVAILABLE" != "1" ]; then
    say "      saved map evidence (YAML + PGM): FAIL (nav2_map_server unavailable)"
    log "validation FAIL: Phase K requires nav2_map_server for final map evidence"
    OVERALL="FAIL"
    return 1
  fi
  [ -n "$SLAM_PID" ] || return 0
  MAP_SAVE_DONE=1
  say "      saving final map evidence..."
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$EVIDENCE_DIR/phase_k_map" \
    --ros-args -p save_map_timeout:=10.0 2>/dev/null \
    >> "$EVIDENCE_DIR/map_saver.log" 2>&1 || true
  if [ -s "$EVIDENCE_DIR/phase_k_map.yaml" ] && [ -s "$EVIDENCE_DIR/phase_k_map.pgm" ]; then
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
  say " Shutting down LunaBot Phase K safely..."
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
  if [ -n "$CMD_INPUT_CAPTURE_PID" ]; then
    stop_group "$CMD_INPUT_CAPTURE_PID"
    wait "$CMD_INPUT_CAPTURE_PID" 2>/dev/null || true
    CMD_INPUT_CAPTURE_PID=""
  fi
  if [ -n "$CMD_OUTPUT_CAPTURE_PID" ]; then
    stop_group "$CMD_OUTPUT_CAPTURE_PID"
    wait "$CMD_OUTPUT_CAPTURE_PID" 2>/dev/null || true
    CMD_OUTPUT_CAPTURE_PID=""
  fi
  if [ -n "$FOLLOWER_PID" ]; then
    stop_group "$FOLLOWER_PID"
    wait "$FOLLOWER_PID" 2>/dev/null || true
  fi
  if [ -n "$REPLAN_MONITOR_PID" ]; then
    stop_group "$REPLAN_MONITOR_PID"
    wait "$REPLAN_MONITOR_PID" 2>/dev/null || true
  fi
  if [ -n "$EVALUATOR_PID" ]; then
    stop_group "$EVALUATOR_PID"
    wait "$EVALUATOR_PID" 2>/dev/null || true
  fi
  if [ -n "$NAV_PID" ]; then
    stop_group "$NAV_PID"
    wait "$NAV_PID" 2>/dev/null || true
  fi
  if [ -n "$TERRAIN_PLANNER_PID" ]; then
    stop_group "$TERRAIN_PLANNER_PID"
    wait "$TERRAIN_PLANNER_PID" 2>/dev/null || true
  fi
  if [ -n "$COST_PID" ]; then
    stop_group "$COST_PID"
    wait "$COST_PID" 2>/dev/null || true
  fi
  if [ -n "$MAPPER_PID" ]; then
    stop_group "$MAPPER_PID"
    wait "$MAPPER_PID" 2>/dev/null || true
  fi
  if [ -n "$PERCEPTION_PID" ]; then
    stop_group "$PERCEPTION_PID"
    wait "$PERCEPTION_PID" 2>/dev/null || true
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
  say "Launch K environment cleanly closed."
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
echo "                 PHASE K"
echo "                 AUTONOMOUS NAVIGATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

# ------------------------------------------------------------
# [1/21] Check project files
# ------------------------------------------------------------
echo "[1/21] Checking Phase A baseline + Phase K files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" "$SLAM_CONFIG" "$NAV_PATH" \
         "$RVIZ_CONFIG" "$PERCEPTION_PATH" "$MAPPER_PATH" "$COST_PATH" "$TERRAIN_PLANNER_PATH" "$FOLLOWER_PATH" "$REPLAN_MONITOR_PATH" "$EVALUATOR_PATH" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/I files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, planner, perception, mapper, cost mapper, terrain planner, follower, rviz: OK"
log "[1/21] files OK"

# ------------------------------------------------------------
# [2/21] Check ROS 2 / Gazebo environment
# ------------------------------------------------------------
echo "[2/21] Checking ROS 2 / Gazebo environment........"
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
log "[2/21] environment OK ($IGN/$MSGNS, $BRIDGE_PKG, slam_toolbox)"

# ------------------------------------------------------------
# [3/21] Clean state
# ------------------------------------------------------------
echo "[3/21] Checking for stale Phase A/I processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
# A prior interrupted `ros2 run` can leave its child bridge alive. Remove
# only processes belonging to this Phase K command/node contract.
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py" "scripts/astar_navigation.py" "scripts/dynamic_replan_monitor.py" "scripts/phase_k_evaluator.py" "slam_toolbox.*online_async"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/21] clean state OK"

# ------------------------------------------------------------
# [4/21] Start Gazebo - same validated Phase A world
# ------------------------------------------------------------
echo "[4/21] Starting Gazebo lunar world.................."
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
# sensors, DiffDrive odometry and the Phase K demo can actually advance.
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/21] gazebo OK and unpaused (pid $GAZEBO_PID)"

# ------------------------------------------------------------
# [5/21] Spawn LunaBot V4
# ------------------------------------------------------------
echo "[5/21] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/21] spawn OK (z=$SPAWN_Z)"

# ------------------------------------------------------------
# [6/21] ROS 2 <-> Gazebo bridge - same sensor contract
# ------------------------------------------------------------
echo "[6/21] Starting ROS 2 <-> Gazebo bridge............."
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
log "[6/21] bridge OK (pid $BRIDGE_PID)"

# ------------------------------------------------------------
# [7/21] Phase K RGB-D terrain segmentation
# ------------------------------------------------------------
echo "[7/21] Starting terrain segmentation................"
: > "$EVIDENCE_DIR/segmentation.log"
setsid python3 "$PERCEPTION_PATH" --ros-args \
  -p image_topic:=/lunabot/camera/image_raw \
  -p depth_topic:=/lunabot/depth/image_raw \
  -p mask_topic:=/lunabot/terrain/segmentation \
  -p overlay_topic:=/lunabot/terrain/overlay \
  -p status_topic:=/lunabot/terrain/segmentation/status \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/segmentation.log" 2>&1 &
PERCEPTION_PID=$!
sleep 2
kill -0 "$PERCEPTION_PID" 2>/dev/null || abort "terrain segmentation exited; see $EVIDENCE_DIR/segmentation.log"
echo "      segmentation running (PID $PERCEPTION_PID), RGB-D -> terrain mask"
log "[7/21] segmentation OK (pid $PERCEPTION_PID)"

# ------------------------------------------------------------
# [8/21] Phase K semantic terrain mapper
# ------------------------------------------------------------
echo "[8/21] Starting semantic terrain mapping..........."
: > "$EVIDENCE_DIR/semantic_mapping.log"
setsid python3 "$MAPPER_PATH" --ros-args \
  -p mask_topic:=/lunabot/terrain/segmentation \
  -p depth_topic:=/lunabot/depth/image_raw \
  -p map_topic:=/lunabot/terrain/semantic_map \
  -p status_topic:=/lunabot/terrain/semantic_map/status \
  -p map_frame:=map -p base_frame:=chassis \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/semantic_mapping.log" 2>&1 &
MAPPER_PID=$!
sleep 2
kill -0 "$MAPPER_PID" 2>/dev/null || abort "semantic terrain mapper exited; see $EVIDENCE_DIR/semantic_mapping.log"
echo "      semantic mapper running (PID $MAPPER_PID), mask + depth -> map"
log "[8/21] semantic mapper OK (pid $MAPPER_PID)"

# ------------------------------------------------------------
# [9/21] Phase K terrain cost-map generator
# ------------------------------------------------------------
echo "[9/21] Starting terrain cost-map generator........."
: > "$EVIDENCE_DIR/cost_mapping.log"
setsid python3 "$COST_PATH" --ros-args \
  -p semantic_topic:=/lunabot/terrain/semantic_map \
  -p cost_topic:=/lunabot/terrain/cost_map \
  -p status_topic:=/lunabot/terrain/cost_map/status \
  -p inflation_radius:=0.30 \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/cost_mapping.log" 2>&1 &
COST_PID=$!
sleep 2
kill -0 "$COST_PID" 2>/dev/null || abort "terrain cost mapper exited; see $EVIDENCE_DIR/cost_mapping.log"
echo "      cost mapper running (PID $COST_PID), semantic map -> cost map"
log "[9/21] cost mapper OK (pid $COST_PID)"

# ------------------------------------------------------------
# [10/21] Phase K terrain-aware path planner
# ------------------------------------------------------------
echo "[10/21] Starting terrain-aware path planner........"
: > "$EVIDENCE_DIR/terrain_planner.log"
setsid python3 "$TERRAIN_PLANNER_PATH" --ros-args \
  -p cost_map_topic:=/lunabot/terrain/cost_map \
  -p goal_topic:=/goal_pose \
  -p odom_topic:=/lunabot/odom \
  -p plan_topic:=/lunabot/terrain/plan \
  -p status_topic:=/lunabot/terrain/planner/status \
  -p map_frame:=map -p base_frame:=chassis \
  -p cost_weight:=2.0 -p replan_period:=1.0 \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/terrain_planner.log" 2>&1 &
TERRAIN_PLANNER_PID=$!
sleep 2
kill -0 "$TERRAIN_PLANNER_PID" 2>/dev/null || abort "terrain-aware planner exited; see $EVIDENCE_DIR/terrain_planner.log"
echo "      terrain-aware planner running (PID $TERRAIN_PLANNER_PID), cost map -> /terrain/plan"
log "[10/21] terrain-aware planner OK (pid $TERRAIN_PLANNER_PID)"

# ------------------------------------------------------------
# [11/21] Phase K dynamic replanning monitor
# ------------------------------------------------------------
echo "[11/21] Starting dynamic replanning monitor........"
: > "$EVIDENCE_DIR/replan.log"
setsid python3 "$REPLAN_MONITOR_PATH" --ros-args \
  -p plan_topic:=/lunabot/terrain/plan \
  -p goal_topic:=/goal_pose \
  -p odom_topic:=/lunabot/odom \
  -p planner_status_topic:=/lunabot/terrain/planner/status \
  -p status_topic:=/lunabot/autonomy/replan_status \
  -p minimum_revisions:=2 -p minimum_motion:=0.05 \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/replan.log" 2>&1 &
REPLAN_MONITOR_PID=$!
sleep 2
kill -0 "$REPLAN_MONITOR_PID" 2>/dev/null || abort "dynamic replanning monitor exited; see $EVIDENCE_DIR/replan.log"
echo "      dynamic replanning monitor running (PID $REPLAN_MONITOR_PID)"
log "[11/21] dynamic replanning monitor OK (pid $REPLAN_MONITOR_PID)"

# ------------------------------------------------------------
# [12/21] Phase K runtime evaluator
# ------------------------------------------------------------
echo "[12/21] Starting runtime evaluation monitor........"
: > "$EVIDENCE_DIR/evaluation.log"
setsid python3 "$EVALUATOR_PATH" --ros-args \
  -p plan_topic:=/lunabot/terrain/plan \
  -p replan_topic:=/lunabot/autonomy/replan_status \
  -p autonomy_topic:=/lunabot/autonomy/status \
  -p control_topic:=/lunabot/control/status \
  -p odom_topic:=/lunabot/odom \
  -p cmd_input_topic:=/cmd_vel_in \
  -p cmd_output_topic:=/cmd_vel \
  -p status_topic:=/lunabot/evaluation/status \
  -p minimum_motion:=0.05 \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/evaluation.log" 2>&1 &
EVALUATOR_PID=$!
sleep 2
kill -0 "$EVALUATOR_PID" 2>/dev/null || abort "runtime evaluation monitor exited; see $EVIDENCE_DIR/evaluation.log"
echo "      runtime evaluation monitor running (PID $EVALUATOR_PID)"
log "[12/21] runtime evaluation monitor OK (pid $EVALUATOR_PID)"

# ------------------------------------------------------------
# [13/21] Static TF - unchanged Phase A sensor frames
# ------------------------------------------------------------
echo "[13/21] Starting static TF (sensor frames)............"
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
log "[13/21] static TF OK"

# ------------------------------------------------------------
# [14/21] Phase K control node
# ------------------------------------------------------------
echo "[14/21] Starting Phase K control layer................"
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
log "[14/21] control OK (pid $CONTROL_PID)"

# ------------------------------------------------------------
# [15/21] Phase K odometry monitor
# ------------------------------------------------------------
echo "[15/21] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[15/21] odometry monitor OK (pid $ODOM_PID)"

# ------------------------------------------------------------
# [16/21] Phase K SLAM and localization
# ------------------------------------------------------------
echo "[16/21] Starting slam_toolbox mapping................"
: > "$EVIDENCE_DIR/slam.log"
setsid ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$SLAM_CONFIG" use_sim_time:=true \
  >> "$EVIDENCE_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 4
kill -0 "$SLAM_PID" 2>/dev/null || abort "slam_toolbox exited; see $EVIDENCE_DIR/slam.log"
echo "      slam_toolbox running (PID $SLAM_PID), mapping /lunabot/lidar/scan"
log "[16/21] slam_toolbox OK (pid $SLAM_PID)"

# ------------------------------------------------------------
# [17/21] Phase K A* autonomous navigation
# ------------------------------------------------------------
echo "[17/21] Starting A* autonomous navigation..........."
: > "$EVIDENCE_DIR/navigation.log"
if [ "$DEMO" = "1" ]; then
  AUTO_GOAL=true
fi
if [ "$EVIDENCE" = "1" ] && [ "$DEMO" = "1" ]; then
  timeout 20 ros2 topic echo /map --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_before_navigation.txt" || true
fi
setsid python3 "$NAV_PATH" --ros-args \
  -p auto_goal:="$AUTO_GOAL" \
  -p map_topic:=/map -p goal_topic:=/goal_pose \
  -p path_topic:=/plan -p cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel \
  -p odom_topic:=/lunabot/odom \
  -p status_topic:=/lunabot/navigation/status \
  -p map_frame:=map -p unknown_is_obstacle:=true \
  -p inflation_radius:=0.25 -p auto_goal_distance:=1.5 \
  >> "$EVIDENCE_DIR/navigation.log" 2>&1 &
NAV_PID=$!
sleep 3
kill -0 "$NAV_PID" 2>/dev/null || abort "A* navigation exited; see $EVIDENCE_DIR/navigation.log"
echo "      diagnostic A* running (PID $NAV_PID), /plan + status only"
log "[17/21] diagnostic A* OK (pid $NAV_PID, auto_goal=$AUTO_GOAL, cmd isolated)"

# ------------------------------------------------------------
# [18/21] Phase K terrain-aware motion integration
# ------------------------------------------------------------
echo "[18/21] Starting terrain-aware path follower........"
: > "$EVIDENCE_DIR/integration.log"
setsid python3 "$FOLLOWER_PATH" --ros-args \
  -p path_topic:=/lunabot/terrain/plan \
  -p goal_topic:=/goal_pose \
  -p odom_topic:=/lunabot/odom \
  -p cmd_topic:=/cmd_vel_in \
  -p status_topic:=/lunabot/autonomy/status \
  -p map_frame:=map -p base_frame:=chassis \
  -p goal_tolerance:=0.35 -p max_linear:=0.20 -p max_angular:=0.60 \
  -p use_sim_time:=true \
  >> "$EVIDENCE_DIR/integration.log" 2>&1 &
FOLLOWER_PID=$!
sleep 2
kill -0 "$FOLLOWER_PID" 2>/dev/null || abort "terrain path follower exited; see $EVIDENCE_DIR/integration.log"
echo "      terrain-aware follower running (PID $FOLLOWER_PID), /terrain/plan -> /cmd_vel_in"
log "[18/21] integration follower OK (pid $FOLLOWER_PID)"

# ------------------------------------------------------------
# [19/21] RViz2
# ------------------------------------------------------------
if [ "$HEADLESS" = "1" ]; then
  echo "[19/21] RViz2 (skipped - HEADLESS).................. OK"
  log "[19/21] rviz skipped (headless)"
else
  echo "[19/21] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f map >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: map"
    log "[19/21] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[19/21] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

# ------------------------------------------------------------
# [20/21] Runtime validation
# ------------------------------------------------------------
twist_has_motion() {
  # ros2 topic echo renders Twist as `linear:` / `angular:` blocks. Accept
  # either forward/reverse or rotation as real motion, but reject an all-zero
  # watchdog stream. The threshold avoids treating formatting noise as motion.
  awk '
    /^linear:/ { section="linear"; next }
    /^angular:/ { section="angular"; next }
    section == "linear" && $1 == "x:" {
      if (($2 + 0.0) > 0.001 || ($2 + 0.0) < -0.001) found=1
    }
    section == "angular" && $1 == "z:" {
      if (($2 + 0.0) > 0.001 || ($2 + 0.0) < -0.001) found=1
    }
    END { exit(found ? 0 : 1) }
  ' "$1"
}

capture_motion_evidence() {
  [ "$DEMO" = "1" ] || return 0
  : > "$EVIDENCE_DIR/cmd_vel_in_motion.txt"
  : > "$EVIDENCE_DIR/cmd_vel_motion.txt"
  setsid timeout 240 ros2 topic echo /cmd_vel_in --qos-reliability best_effort \
    > "$EVIDENCE_DIR/cmd_vel_in_motion.txt" 2>/dev/null &
  CMD_INPUT_CAPTURE_PID=$!
  setsid timeout 240 ros2 topic echo /cmd_vel --qos-reliability best_effort \
    > "$EVIDENCE_DIR/cmd_vel_motion.txt" 2>/dev/null &
  CMD_OUTPUT_CAPTURE_PID=$!
  log "runtime evidence: capturing /cmd_vel_in and /cmd_vel motion"
}

finish_motion_evidence() {
  [ "$DEMO" = "1" ] || return 0
  # Leave a short window for the last command and its controller output to be
  # flushed before stopping the capture groups.
  sleep 2
  if [ -n "$CMD_INPUT_CAPTURE_PID" ]; then
    stop_group "$CMD_INPUT_CAPTURE_PID"
    wait "$CMD_INPUT_CAPTURE_PID" 2>/dev/null || true
    CMD_INPUT_CAPTURE_PID=""
  fi
  if [ -n "$CMD_OUTPUT_CAPTURE_PID" ]; then
    stop_group "$CMD_OUTPUT_CAPTURE_PID"
    wait "$CMD_OUTPUT_CAPTURE_PID" 2>/dev/null || true
    CMD_OUTPUT_CAPTURE_PID=""
  fi
  if twist_has_motion "$EVIDENCE_DIR/cmd_vel_in_motion.txt"; then
    echo "      nonzero /cmd_vel_in motion evidence: PASS"
    log "validation PASS: nonzero /cmd_vel_in motion"
  else
    echo "      nonzero /cmd_vel_in motion evidence: FAIL"
    log "validation FAIL: /cmd_vel_in contained no nonzero Twist"
    OVERALL="FAIL"
  fi
  if twist_has_motion "$EVIDENCE_DIR/cmd_vel_motion.txt"; then
    echo "      controller output /cmd_vel motion evidence: PASS"
    log "validation PASS: nonzero controller output /cmd_vel"
  else
    echo "      controller output /cmd_vel motion evidence: FAIL"
    log "validation FAIL: /cmd_vel contained no nonzero Twist"
    OVERALL="FAIL"
  fi
  control_boundary="$(timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null || true)"
  printf '%s\n' "$control_boundary" > "$EVIDENCE_DIR/controller_boundary_status.txt"
  if printf '%s\n' "$control_boundary" | grep -qE "ACTIVE|WATCHDOG_STOP" && \
     printf '%s\n' "$control_boundary" | grep -q "input=/cmd_vel_in"; then
    echo "      controller boundary evidence (/cmd_vel_in -> /cmd_vel): PASS"
    log "validation PASS: controller ACTIVE boundary evidence"
  else
    echo "      controller boundary evidence (/cmd_vel_in -> /cmd_vel): FAIL"
    log "validation FAIL: controller boundary status was [$control_boundary]"
    OVERALL="FAIL"
  fi
}

echo "[20/21] Runtime validation.........................."
capture_motion_evidence
wait_for_evaluation() {
  local status_file="$EVIDENCE_DIR/evaluation_wait_status.txt"
  rm -f "$status_file"
  echo "      waiting for aggregate runtime evaluation..."
  setsid timeout 180 ros2 topic echo /lunabot/evaluation/status \
    --qos-reliability reliable --qos-durability transient_local \
    > "$status_file" 2>/dev/null &
  local wait_pid=$!
  for _ in $(seq 1 180); do
    if grep -q "EVALUATION_PASS" "$status_file" 2>/dev/null; then
      stop_group "$wait_pid"
      wait "$wait_pid" 2>/dev/null || true
      echo "      aggregate runtime evaluation: PASS"
      log "validation PASS: EVALUATION_PASS"
      return 0
    fi
    if ! kill -0 "$wait_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  stop_group "$wait_pid"
  wait "$wait_pid" 2>/dev/null || true
  echo "      aggregate runtime evaluation: FAIL"
  log "validation FAIL: evaluator did not reach pass"
  OVERALL="FAIL"
  return 1
}

wait_for_dynamic_replan() {
  local status_file="$EVIDENCE_DIR/replan_wait_status.txt"
  rm -f "$status_file"
  echo "      waiting for dynamic terrain-plan revisions..."
  setsid timeout 180 ros2 topic echo /lunabot/autonomy/replan_status \
    --qos-reliability reliable --qos-durability transient_local \
    > "$status_file" 2>/dev/null &
  local wait_pid=$!
  for _ in $(seq 1 180); do
    if grep -q "DYNAMIC_REPLAN_PASS" "$status_file" 2>/dev/null; then
      stop_group "$wait_pid"
      wait "$wait_pid" 2>/dev/null || true
      echo "      dynamic terrain-plan replanning: PASS"
      log "validation PASS: DYNAMIC_REPLAN_PASS"
      return 0
    fi
    if ! kill -0 "$wait_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  stop_group "$wait_pid"
  wait "$wait_pid" 2>/dev/null || true
  echo "      dynamic terrain-plan replanning: FAIL"
  log "validation FAIL: dynamic replan monitor did not reach pass"
  OVERALL="FAIL"
  return 1
}

wait_for_integration_goal() {
  local status_file="$EVIDENCE_DIR/goal_wait_status.txt"
  rm -f "$status_file"
  echo "      waiting for terrain-integrated GOAL_REACHED..."
  # Keep the ROS subscriber out of the launcher's foreground wait. This lets
  # the INT/TERM trap run immediately and shutdown() can terminate this group.
  setsid timeout 180 ros2 topic echo /lunabot/autonomy/status \
    --qos-reliability reliable --qos-durability transient_local \
    > "$status_file" 2>/dev/null &
  GOAL_WAIT_PID=$!
  for _ in $(seq 1 180); do
    if grep -q "INTEGRATION_GOAL_REACHED" "$status_file" 2>/dev/null; then
      stop_group "$GOAL_WAIT_PID"
      wait "$GOAL_WAIT_PID" 2>/dev/null || true
      GOAL_WAIT_PID=""
      echo "      terrain-integrated goal reached: PASS"
      log "validation PASS: terrain-integrated INTEGRATION_GOAL_REACHED"
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
  echo "      terrain-integrated goal reached: FAIL"
  log "validation FAIL: terrain-integrated follower did not reach goal within 180 s"
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
  local reliability="best_effort"
  case "$2" in
    /map|/plan|/lunabot/navigation/status|/lunabot/terrain/semantic_map|/lunabot/terrain/semantic_map/status|/lunabot/terrain/cost_map|/lunabot/terrain/cost_map/status|/lunabot/terrain/plan|/lunabot/terrain/planner/status|/lunabot/autonomy/status|/lunabot/autonomy/replan_status|/lunabot/evaluation/status)
      durability="transient_local"
      # Reliable + transient-local is required to retrieve the retained status
      # from a publisher after the node emitted its startup message.
      reliability="reliable"
      ;;
    /goal_pose)
      # Goals are volatile commands, but use a reliable observer for the
      # reliable A* publisher rather than a best-effort late observer.
      reliability="reliable"
      ;;
  esac
  # Do not pipe to head: head can exit successfully before ros2 receives a
  # message, creating a false PASS. This assignment waits for --once data.
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability "$reliability" \
      --qos-durability "$durability" --once 2>/dev/null)" && [ -n "$sample" ]; then
    # /map must be an OccupancyGrid-shaped message, not merely any message.
    if [ "$2" = "/map" ] && { [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/plan" ] && { [[ "$sample" != *"poses:"* ]] || [[ "$sample" == *"poses: []"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/plan" ] && { [[ "$sample" != *"poses:"* ]] || [[ "$sample" == *"poses: []"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/goal_pose" ] && [[ "$sample" != *"pose:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/navigation/status" ] && [[ "$sample" != *"data:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/segmentation" ] && {
      [[ "$sample" != *"height:"* ]] || [[ "$sample" != *"width:"* ]] ||
      [[ "$sample" != *"encoding:"* ]] || [[ "$sample" != *"data:"* ]];
    }; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/overlay" ] && {
      [[ "$sample" != *"height:"* ]] || [[ "$sample" != *"width:"* ]] ||
      [[ "$sample" != *"encoding:"* ]] || [[ "$sample" != *"data:"* ]];
    }; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/segmentation/status" ] && [[ "$sample" != *"data:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/semantic_map" ] && {
      [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]];
    }; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/semantic_map/status" ] && [[ "$sample" != *"data:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/cost_map" ] && {
      [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]];
    }; then
      sample=""
    fi
    if [ "$2" = "/lunabot/terrain/cost_map/status" ] && [[ "$sample" != *"data:"* ]]; then
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
topic_ok "topic /lunabot/depth/image_raw"    "/lunabot/depth/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"
tf_ok "TF sensor_head -> scoped LaserScan frame" "sensor_head" "lunabot_v4/sensor_head/lidar"
type_ok "type /goal_pose geometry_msgs/PoseStamped" "/goal_pose" "geometry_msgs/msg/PoseStamped"
type_ok "type /plan nav_msgs/Path" "/plan" "nav_msgs/msg/Path"
type_ok "type navigation status std_msgs/String" "/lunabot/navigation/status" "std_msgs/msg/String"
# A* intentionally stops republishing its volatile goal after GOAL_REACHED.
# In DEMO mode, use the recorded AUTO_GOAL_SENT event plus the real goal
# completion gate instead of requiring a late volatile sample.
if [ "$DEMO" = "1" ] && grep -q "AUTO_GOAL_SENT" "$EVIDENCE_DIR/navigation.log" 2>/dev/null; then
  echo "      topic /goal_pose: PASS (AUTO_GOAL_SENT + completion evidence)"
  log "validation PASS: /goal_pose AUTO_GOAL_SENT"
else
  topic_ok "topic /goal_pose"                "/goal_pose"
fi
topic_ok "topic /plan"                         "/plan"
topic_ok "topic /lunabot/navigation/status"    "/lunabot/navigation/status"
topic_ok "topic /cmd_vel_in"                  "/cmd_vel_in"
type_ok "type terrain mask sensor_msgs/Image" "/lunabot/terrain/segmentation" "sensor_msgs/msg/Image"
type_ok "type terrain overlay sensor_msgs/Image" "/lunabot/terrain/overlay" "sensor_msgs/msg/Image"
type_ok "type terrain status std_msgs/String" "/lunabot/terrain/segmentation/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/terrain/segmentation" "/lunabot/terrain/segmentation"
topic_ok "topic /lunabot/terrain/overlay"      "/lunabot/terrain/overlay"
topic_ok "topic /lunabot/terrain/segmentation/status" "/lunabot/terrain/segmentation/status"
segmentation_status="$(timeout 15 ros2 topic echo /lunabot/terrain/segmentation/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$segmentation_status" | grep -q "SEGMENTATION_PASS"; then
  echo "      terrain segmentation content: PASS"
  log "validation PASS: SEGMENTATION_PASS status"
else
  echo "      terrain segmentation content: FAIL"
  log "validation FAIL: segmentation status was [$segmentation_status]"
  OVERALL="FAIL"
fi
type_ok "type semantic map nav_msgs/OccupancyGrid" "/lunabot/terrain/semantic_map" "nav_msgs/msg/OccupancyGrid"
type_ok "type semantic map status std_msgs/String" "/lunabot/terrain/semantic_map/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/terrain/semantic_map" "/lunabot/terrain/semantic_map"
topic_ok "topic /lunabot/terrain/semantic_map/status" "/lunabot/terrain/semantic_map/status"
semantic_status="$(timeout 15 ros2 topic echo /lunabot/terrain/semantic_map/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$semantic_status" | grep -q "SEMANTIC_MAP_PASS"; then
  echo "      semantic map content: PASS"
  log "validation PASS: SEMANTIC_MAP_PASS status"
else
  echo "      semantic map content: FAIL"
  log "validation FAIL: semantic map status was [$semantic_status]"
  OVERALL="FAIL"
fi
type_ok "type cost map nav_msgs/OccupancyGrid" "/lunabot/terrain/cost_map" "nav_msgs/msg/OccupancyGrid"
type_ok "type cost map status std_msgs/String" "/lunabot/terrain/cost_map/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/terrain/cost_map" "/lunabot/terrain/cost_map"
topic_ok "topic /lunabot/terrain/cost_map/status" "/lunabot/terrain/cost_map/status"
cost_status="$(timeout 15 ros2 topic echo /lunabot/terrain/cost_map/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$cost_status" | grep -q "COST_MAP_PASS"; then
  echo "      cost map content: PASS"
  log "validation PASS: COST_MAP_PASS status"
else
  echo "      cost map content: FAIL"
  log "validation FAIL: cost map status was [$cost_status]"
  OVERALL="FAIL"
fi
type_ok "type terrain plan nav_msgs/Path" "/lunabot/terrain/plan" "nav_msgs/msg/Path"
type_ok "type terrain planner status std_msgs/String" "/lunabot/terrain/planner/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/terrain/plan" "/lunabot/terrain/plan"
topic_ok "topic /lunabot/terrain/planner/status" "/lunabot/terrain/planner/status"
terrain_plan_status="$(timeout 15 ros2 topic echo /lunabot/terrain/planner/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$terrain_plan_status" | grep -q "TERRAIN_PLAN_PASS"; then
  echo "      terrain-aware plan content: PASS"
  log "validation PASS: TERRAIN_PLAN_PASS status"
else
  echo "      terrain-aware plan content: FAIL"
  log "validation FAIL: terrain planner status was [$terrain_plan_status]"
  OVERALL="FAIL"
fi
type_ok "type dynamic replan status std_msgs/String" \
  "/lunabot/autonomy/replan_status" "std_msgs/msg/String"
topic_ok "topic /lunabot/autonomy/replan_status" \
  "/lunabot/autonomy/replan_status"
replan_status="$(timeout 15 ros2 topic echo /lunabot/autonomy/replan_status \
  --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$replan_status" | grep -q "DYNAMIC_REPLAN_PASS"; then
  echo "      dynamic replan status content: PASS"
  log "validation PASS: DYNAMIC_REPLAN_PASS status"
else
  echo "      dynamic replan status content: FAIL"
  log "validation FAIL: dynamic replan status was [$replan_status]"
  OVERALL="FAIL"
fi
type_ok "type evaluation status std_msgs/String" \
  "/lunabot/evaluation/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/evaluation/status" \
  "/lunabot/evaluation/status"
evaluation_status="$(timeout 15 ros2 topic echo /lunabot/evaluation/status \
  --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$evaluation_status" | grep -q "EVALUATION_"; then
  echo "      evaluation status content: PASS"
  log "validation PASS: evaluation status available"
else
  echo "      evaluation status content: FAIL"
  log "validation FAIL: evaluation status was [$evaluation_status]"
  OVERALL="FAIL"
fi
type_ok "type integration status std_msgs/String" "/lunabot/autonomy/status" "std_msgs/msg/String"
topic_ok "topic /lunabot/autonomy/status" "/lunabot/autonomy/status"
integration_status="$(timeout 15 ros2 topic echo /lunabot/autonomy/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null || true)"
if printf '%s\n' "$integration_status" | grep -q "INTEGRATION_"; then
  echo "      integration status content: PASS"
  log "validation PASS: terrain integration status"
else
  echo "      integration status content: FAIL"
  log "validation FAIL: integration status was [$integration_status]"
  OVERALL="FAIL"
fi
if [ "$DEMO" = "1" ]; then
  if printf '%s\n' "$replan_status" | grep -q "DYNAMIC_REPLAN_PASS"; then
    printf '%s\n' "$replan_status" > "$EVIDENCE_DIR/replan_wait_status.txt"
    echo "      dynamic terrain-plan replanning: PASS"
    log "validation PASS: DYNAMIC_REPLAN_PASS"
  else
    wait_for_dynamic_replan || true
  fi
  wait_for_integration_goal || true
  wait_for_evaluation || true
  finish_motion_evidence
fi

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase K runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 15 ros2 topic echo /lunabot/navigation/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/navigation_status.txt"
  timeout 15 ros2 topic echo /lunabot/autonomy/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/integration_status.txt"
  timeout 15 ros2 topic echo /lunabot/autonomy/replan_status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/replan_status.txt"
  timeout 15 ros2 topic echo /lunabot/evaluation/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/evaluation_status.txt"
  timeout 15 ros2 topic echo /goal_pose --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/goal_pose.txt"
  timeout 15 ros2 topic echo /plan --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/plan.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 20 ros2 topic echo /lunabot/depth/image_raw --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/depth_sample.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/segmentation --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/segmentation_sample.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/overlay --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/overlay_sample.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/segmentation/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/segmentation_status.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/semantic_map --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/semantic_map_sample.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/semantic_map/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/semantic_map_status.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/cost_map --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/cost_map_sample.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/cost_map/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/cost_map_status.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/plan --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/terrain_plan.txt"
  timeout 20 ros2 topic echo /lunabot/terrain/planner/status --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/terrain_planner_status.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    > "$EVIDENCE_DIR/tf_odom_chassis.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo map odom 2>/dev/null \
    > "$EVIDENCE_DIR/tf_map_odom.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo sensor_head lunabot_v4/sensor_head/lidar 2>/dev/null \
    > "$EVIDENCE_DIR/tf_lidar_scoped.txt" || true
  timeout 20 ros2 topic echo /map --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_sample.txt"
  if [ "$DEMO" = "1" ]; then
    timeout 20 ros2 topic echo /map --qos-reliability reliable --qos-durability transient_local --once 2>/dev/null \
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
    echo "      map saver unavailable; final map evidence will fail clean shutdown"
    echo "map saver required: install ros-humble-nav2-map-server to write PGM/YAML" \
      > "$EVIDENCE_DIR/map_saver.log"
  fi
  echo "      evidence written (map, TF map->odom, odom, SLAM log)"
  log "[20/21] evidence recorded"
fi

# ------------------------------------------------------------
# [21/21] Status
# ------------------------------------------------------------
echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "SLAM              : RUNNING (slam_toolbox mapping)"
echo "Navigation        : RUNNING (terrain-aware follower -> /cmd_vel_in)"
echo "Dynamic replanning: RUNNING (live terrain-plan revisions)"
echo "Evaluation        : RUNNING (runtime metrics and safety evidence)"
echo "Diagnostic A*     : RUNNING (path/status only)"
echo "Map               : RUNNING (/map + map->odom TF)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Terrain perception: RUNNING (RGB-D segmentation)"
echo "Semantic mapping  : RUNNING (map-frame terrain grid)"
echo "Terrain cost map  : RUNNING (inflated traversability costs)"
echo "Terrain-aware plan: RUNNING (weighted A* path output)"
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
echo "  /lunabot/navigation/status std_msgs/String (diagnostic A* state)"
echo "  /lunabot/autonomy/status std_msgs/String (active integration)"
echo "  /lunabot/autonomy/replan_status std_msgs/String (dynamic replanning)"
echo "  /lunabot/evaluation/status std_msgs/String (runtime evaluation)"
echo "  /lunabot/terrain/segmentation sensor_msgs/Image (mask)"
echo "  /lunabot/terrain/overlay      sensor_msgs/Image (overlay)"
echo "  /lunabot/terrain/segmentation/status std_msgs/String"
echo "  /lunabot/terrain/semantic_map nav_msgs/OccupancyGrid"
echo "  /lunabot/terrain/semantic_map/status std_msgs/String"
echo "  /lunabot/terrain/cost_map nav_msgs/OccupancyGrid"
echo "  /lunabot/terrain/cost_map/status std_msgs/String"
echo "  /lunabot/terrain/plan nav_msgs/Path"
echo "  /lunabot/terrain/planner/status std_msgs/String"
echo "  map -> odom                  TF (SLAM localization)"
echo "  evidence/phase-k-launch-k/map_sample.txt"
echo "  evidence/phase-k-launch-k/phase_k_map.yaml/.pgm"
echo "  evidence/phase-k-launch-k/odometry_samples.csv"
echo "  evidence/phase-k-launch-k/odometry_report.txt"
echo "  evidence/phase-k-launch-k/demo_drive_result.txt"
echo "  evidence/phase-k-launch-k/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors, control and odometry preserved"
echo "  - one SLAM system: slam_toolbox"
echo "  - diagnostic A* consumes /map and publishes /plan"
echo "  - RGB-D terrain segmentation publishes mask and overlay"
echo "  - semantic terrain mapper accumulates labels in map frame"
echo "  - terrain cost map converts labels to inflated traversability costs"
echo "  - terrain-aware planner plus follower drives through /cmd_vel_in"
echo "  - dynamic replanning monitor proves changed terrain plans during motion"
echo "  - runtime evaluator proves plan, goal, motion, controller, and odometry metrics"
echo "  - navigation commands remain behind the Phase B controller"
echo "  - /map and map->odom localization outputs verified"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[21/21] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

# ------------------------------------------------------------
# ------------------------------------------------------------
# Autonomous Phase K run
# ------------------------------------------------------------
if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED TERRAIN-AWARE AUTONOMY TEST (DEMO mode)"
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
  printf "PHASE K RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase K autonomous navigation is running."
  echo ""
  echo " A* auto goal: $AUTO_GOAL"
  echo " RViz can publish a replacement goal on /goal_pose."
  echo " Press Ctrl+C to stop Phase K safely."
  echo "============================================================"
  while true; do sleep 1; done
fi
