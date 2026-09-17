#!/usr/bin/env python3
"""Static Phase D gate: A* autonomous navigation.

This validator checks the independent Phase D repository contract and the
inherited Phase A/B/C safety interfaces. It does not claim that a real rover
reached a goal; that requires EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d.
"""

from pathlib import Path
import ast
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []


def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)


required = [
    "launch-a", "launch-b", "launch-c", "launch-d",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/wasd_teleop.py",
    "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "rviz/phase_d.rviz", "tools/validate_phase_a.py",
    "tools/validate_phase_b.py", "tools/validate_phase_c.py",
    "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md",
    "evidence/phase-c-launch-c/README.md",
    "evidence/phase-d-launch-d/README.md",
    "evidence/phase-d-launch-d/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-d", "scripts/launch-d.sh", "scripts/astar_navigation.py",
            "tools/validate_phase_d.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/wasd_teleop.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "scripts/astar_navigation.py",
            "tools/validate_phase_a.py", "tools/validate_phase_b.py",
            "tools/validate_phase_c.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d",
            "scripts/launch-a.sh", "scripts/launch-b.sh",
            "scripts/launch-c.sh", "scripts/launch-d.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

# Earlier phase static gates remain part of the Phase D baseline.
for rel, expected in [("tools/validate_phase_a.py", "80/80"),
                      ("tools/validate_phase_b.py", "103/103"),
                      ("tools/validate_phase_c.py", "133/133")]:
    r = run([sys.executable, rel])
    check(f"existing static gate remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-d.sh")
wrapper = read("launch-d")
planner = read("scripts/astar_navigation.py")
rviz = read("rviz/phase_d.rviz")
world = read("src/lunabot_gazebo/worlds/lunar_world.sdf")
model = read("src/lunabot_gazebo/models/lunabot_v4/model.sdf")
docs = read("docs/phase-4-launch-d.md")

# Independent launch and inherited cleanup.
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))
check("root launch-d resolves its real path", "readlink -f" in wrapper)
check("root launch-d execs scripts/launch-d.sh", "scripts/launch-d.sh" in wrapper)
for old in ["launch-a", "launch-b", "launch-c"]:
    check(f"Phase D does not invoke {old}", old not in noncomment)
check("Phase D has its own evidence directory",
      "phase-d-launch-d" in launch and "phase-c-launch-c" not in noncomment)
check("Phase D is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase D uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase D has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase D shuts down navigation before SLAM", 'stop_group "$NAV_PID"' in launch and
      launch.index('stop_group "$NAV_PID"') < launch.index('stop_group "$SLAM_PID"'))
check("Phase D has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase D interrupts and cleans the goal wait", "GOAL_WAIT_PID" in launch and
      'stop_group "$GOAL_WAIT_PID"' in launch)
check("Phase D cleans stale navigation nodes", "scripts/astar_navigation.py" in launch)
check("Phase D avoids head-based early passes", "| head" not in launch)

# Single mapping/localization system: retain slam_toolbox, add only A*.
check("slam_toolbox remains the only SLAM system", "slam_toolbox" in launch and
      "slam_toolbox" in read("config/slam_toolbox_phase_c.yaml"))
for forbidden in ["cartographer", "nav2_amcl", "hector_slam", "karto_slam", "rtabmap"]:
    check(f"no second stack: {forbidden}", forbidden not in (launch + planner).lower())
check("no AMCL runtime", "amcl" not in (launch + planner).lower())
check("Phase D starts slam_toolbox directly", "ros2 launch slam_toolbox online_async_launch.py" in launch)
check("Phase D uses the approved SLAM config", 'slam_params_file:="$SLAM_CONFIG"' in launch)
check("Phase D uses simulated time", "use_sim_time:=true" in launch)

# A* planner implementation.
check("planner has a ROS node", "class AStarNavigation(Node)" in planner)
check("planner consumes OccupancyGrid", "OccupancyGrid" in planner and "_map_callback" in planner)
check("planner implements a heap-based open set", "heapq" in planner and "open_set" in planner)
check("planner tracks A* g scores", "g_score" in planner and "came_from" in planner)
check("planner uses eight-connected neighbors", "(1, 1), (1, -1), (-1, 1), (-1, -1)" in planner)
check("planner publishes nav_msgs Path", "Path" in planner and "self.path_pub.publish(path)" in planner)
check("planner publishes goal pose", "self.goal_pub.publish(goal)" in planner)
check("planner consumes goal pose", "PoseStamped" in planner and "_goal_callback" in planner)
check("planner uses real odometry", "Odometry" in planner and "self.odom_msg" in planner)
check("planner transforms odom pose into map", "lookup_transform" in planner and
      "self.map_frame" in planner)
check("planner handles map origin orientation", "_world_to_grid" in planner and
      "origin.orientation" in planner)
check("planner rejects occupied cells", "occupied_threshold" in planner and
      "_cell_is_free" in planner)
check("planner handles unknown cells explicitly", "unknown_is_obstacle" in planner)
check("planner inflates obstacles", "inflation_radius" in planner and "_safe_cell" in planner)
check("planner has goal tolerance", "goal_tolerance" in planner and "GOAL_REACHED" in planner)
check("planner publishes conservative cmd_vel_in", "cmd_topic" in planner and
      "self.cmd_pub.publish(cmd)" in planner)
check("planner stops on goal", "self._publish_stop()" in planner and "goal_distance" in planner)
check("planner publishes auditable status", "/lunabot/navigation/status" in planner and
      "PLANNING_PASS" in planner and "NO_PATH" in planner)
check("planner provides a deterministic auto goal", "auto_goal_distance" in planner and
      "AUTO_GOAL_SENT" in planner)
check("GUI defaults to operator-selected goal", 'AUTO_GOAL="${AUTO_GOAL:-false}"' in launch and
      'if [ "$DEMO" = "1" ]; then' in launch and "AUTO_GOAL=true" in launch)
check("planner uses transient-local map input", "TRANSIENT_LOCAL" in planner and
      "map_qos" in planner)
check("planner makes path/status late-join safe and republishes goals",
      planner.count("latched_qos") >= 1 and "self.status_pub" in planner and
      "self.path_pub" in planner and "goal_pub.publish(self.goal_msg)" in planner)
check("planner has no direct Gazebo dependency", "ignition" not in planner.lower() and
      "gazebo" not in planner.lower())

# Phase A/B/C runtime contract carried forward.
check("Phase D uses the validated world", 'WORLD_PATH="$WORLD_DIR/lunar_world.sdf"' in launch)
check("Phase D resolves its A* planner path", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase D preserves spawn height", 'SPAWN_Z="-2.308"' in launch)

check("Phase D supports headless Gazebo", 'HEADLESS="${HEADLESS:-0}"' in launch and
      "gazebo -s" in launch)
check("Phase D explicitly unpauses Gazebo", "WorldControl" in launch and
      "--req 'pause: false'" in launch)
check("Phase D detects both bridge families", "ros_ign_bridge" in launch and
      "ros_gz_bridge" in launch)
check("Phase D bridges clock", '"/clock@rosgraph_msgs/msg/Clock' in launch)
check("Phase D bridges LaserScan", '"/lunabot/lidar/scan@sensor_msgs/msg/LaserScan' in launch)
check("Phase D bridges camera", '"/lunabot/camera/image_raw@sensor_msgs/msg/Image' in launch)
check("Phase D bridges IMU", '"/lunabot/imu@sensor_msgs/msg/Imu' in launch)
check("Phase D bridges odometry", '"/lunabot/odom@nav_msgs/msg/Odometry' in launch)
check("Phase D keeps scoped LaserScan TF", '"sensor_head|lunabot_v4/sensor_head/lidar|' in launch)
check("Phase D keeps controller input boundary", "--input-topic /cmd_vel_in" in launch)
check("Phase D keeps controller output boundary", "--output-topic /cmd_vel" in launch)
check("Phase D keeps watchdog", "--watchdog-sec 0.5" in launch)
check("Phase D starts odometry monitor", "odometry_monitor.py" in launch and "ODOM_PID" in launch)
check("Phase D keeps Fortress IMU plugin", "ignition-gazebo-imu-system" in world and
      "ignition::gazebo::systems::Imu" in world)
check("model odometry contract is unchanged", "<odom_topic>/lunabot/odom</odom_topic>" in model and
      "<child_frame_id>chassis</child_frame_id>" in model)

# Runtime gates.
check("runtime validates real topic messages", 'sample="$(timeout 30 ros2 topic echo' in launch)
check("runtime has no head-based topic validation", "| head" not in launch)
check("runtime validates OccupancyGrid type", "nav_msgs/msg/OccupancyGrid" in launch)
check("runtime validates map structure", '"info:"' in launch and '"data:"' in launch)
check("runtime validates map TF", 'tf_ok "TF map -> odom" "map" "odom"' in launch)
check("runtime validates odom TF", 'tf_ok "TF odom -> chassis" "odom" "chassis"' in launch)
check("runtime validates scoped lidar TF", "TF sensor_head -> scoped LaserScan frame" in launch)
check("runtime validates IMU", 'topic_ok "topic /lunabot/imu"' in launch)
check("runtime validates Camera and LaserScan", 'topic_ok "topic /lunabot/camera/image_raw"' in launch and
      'topic_ok "topic /lunabot/lidar/scan"' in launch)
check("runtime validates A* goal/path/status", 'topic_ok "topic /goal_pose"' in launch and
      'topic_ok "topic /plan"' in launch and
      'topic_ok "topic /lunabot/navigation/status"' in launch)
check("runtime validates A* message types", 'type_ok "type /goal_pose geometry_msgs/PoseStamped"' in launch and
      'type_ok "type /plan nav_msgs/Path"' in launch and
      'type_ok "type navigation status std_msgs/String"' in launch)
check("runtime requires a non-empty path", '[[ "$sample" == *"poses: []"* ]]' in launch)
check("runtime waits for goal reached", "wait_for_goal" in launch and "GOAL_REACHED" in launch)
check("runtime compares map before/after A*", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch and "map changed during A* navigation" in launch)
check("runtime saves Phase D map evidence", "phase_d_map.yaml" in launch and
      "map_saver_cli" in launch)
check("demo failure affects exit code", "EXIT_CODE=1" in launch and
      "PHASE D RUN COMPLETE" in launch)

# Phase D RViz preserves the Phase C visual fixes and adds A* path.
check("Phase D RViz uses map fixed frame", "Fixed Frame: map" in rviz)
check("Phase D RViz has SLAM Map", "rviz_default_plugins/Map" in rviz and
      "Value: /map" in rviz)
check("Phase D RViz has A* Path", "rviz_default_plugins/Path" in rviz and
      "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase D RViz has explicit LaserScan", "Value: /lunabot/lidar/scan" in rviz)
check("Phase D RViz has explicit Camera", "Value: /lunabot/camera/image_raw" in rviz)
check("Phase D RViz uses best effort sensor QoS", rviz.count("Reliability Policy: Best Effort") >= 2)
check("Phase D RViz has explicit odometry", "Value: /lunabot/odom" in rviz)
check("Phase D RViz provides Set Goal tool", "rviz_default_plugins/SetGoal" in rviz and
      "Topic: /goal_pose" in rviz)
check("Phase D RViz displays selected goal", "Name: Selected Goal" in rviz and
      "Value: /goal_pose" in rviz)
check("Phase D RViz stores map/odom TF", "map:\n          odom:" in rviz)

# Documentation and evidence contract.
for phrase, name in [
    ("sudo apt install -y", "installation"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d", "automated command"),
    ("A* goal reached: PASS", "goal acceptance"),
    ("/plan", "path output"),
    ("/cmd_vel_in", "controller boundary"),
    ("live map updates during autonomous navigation", "map motion acceptance"),
    ("phase_d_map.yaml", "map evidence"),
    ("clean relaunch", "relaunch"),
    ("before Phase E", "Phase E gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "does not claim" in read("tools/validate_phase_d.py") and
      "runtime" in read("tools/validate_phase_d.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE D STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-d-launch-d/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
