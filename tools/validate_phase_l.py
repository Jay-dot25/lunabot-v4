#!/usr/bin/env python3
"""Static Phase L gate: final mission demonstration (injector + 5-class graded + real metrics)."""

from pathlib import Path
import ast
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
    "launch-l", "scripts/launch-l.sh", "scripts/phase_l_mission.py",
    "scripts/phase_k_evaluator.py", "scripts/inject_dynamic_obstacle.py",
    "tools/validate_phase_l.py", "docs/phase-12-launch-l.md",
    "evidence/phase-l-launch-l/README.md", "rviz/phase_l.rviz",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-l", "scripts/launch-l.sh", "scripts/phase_l_mission.py",
            "scripts/inject_dynamic_obstacle.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/phase_l_mission.py", "scripts/phase_k_evaluator.py",
            "scripts/inject_dynamic_obstacle.py", "scripts/obstacle_detector.py",
            "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-l", "scripts/launch-l.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, label in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                   ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{label} SDF well-formed", True)
    except Exception as exc:
        check(f"{label} SDF well-formed", False, str(exc))

for ph in ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k"]:
    check(f"baseline {ph} static validation exists", (ROOT / f"evidence/phase-{ph}-launch-{ph}/static_validation.txt").is_file())

launch = read("scripts/launch-l.sh")
wrapper = read("launch-l")
mission = read("scripts/phase_l_mission.py")
rviz = read("rviz/phase_l.rviz")
docs = read("docs/phase-12-launch-l.md")
noncomment = "\n".join(line for line in launch.splitlines() if not line.lstrip().startswith("#"))

check("root launch-l resolves its real path", "readlink -f" in wrapper and "SCRIPT_PATH" in wrapper)
check("root launch-l execs scripts/launch-l.sh", "scripts/launch-l.sh" in wrapper)
check("Phase L has its own evidence directory", "phase-l-launch-l" in launch)
check("Phase L is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase L uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase L has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase L cleans mission observer", 'stop_group "$MISSION_PID"' in launch)
check("Phase L cleans evaluator", 'stop_group "$EVALUATOR_PID"' in launch)
check("Phase L cleans injector", 'stop_group "$INJECTOR_PID"' in launch)
check("Phase L reports clean shutdown", "Launch L environment cleanly closed." in launch)
check("Phase L has 23 independent stages", "[23/23]" in launch)
check("Phase L preserves evaluator", "phase_k_evaluator.py" in launch and "EVALUATION_PASS" in launch)
check("Phase L starts obstacle detector", 'python3 "$OBSTACLE_PATH" --ros-args' in launch)
check("Phase L supports manual final mode", "FINAL_DEMO" in launch and "AUTO_GOAL" in launch)
check("Phase L uses its own RViz config", "rviz/phase_l.rviz" in launch)
check("Phase L preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch)
check("Phase L retains diagnostic isolation", "cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel" in launch)
check("Phase L retains single active motion source", "cmd_topic:=/cmd_vel_in" in launch)
check("Phase L requires final map evidence", "phase_l_map.yaml" in launch and "phase_l_map.pgm" in launch)

check("mission supervisor is a ROS node", "class PhaseLMission(Node)" in mission)
check("mission supervisor observes terrain plan", "self.plan_seen" in mission)
check("mission supervisor observes replanning", "DYNAMIC_REPLAN_PASS" in mission and "self.replan_pass" in mission)
check("mission supervisor observes integrated goal", "INTEGRATION_GOAL_REACHED" in mission)
check("mission supervisor observes aggregate evaluation", "EVALUATION_PASS" in mission and "self.evaluation_pass" in mission)
check("mission supervisor observes obstacle status", "OBSTACLE_DETECTED" in mission and "self.obstacle_detected" in mission)
check("mission supervisor observes real goal and map", "PoseStamped" in mission and "OccupancyGrid" in mission)
check("mission supervisor observes dynamic_obstacle_topic cost_map_topic semantic_map_topic", "dynamic_obstacle_topic" in mission and "cost_map_topic" in mission and "semantic_map_topic" in mission)
check("mission supervisor emits retained pass", "MISSION_DEMO_PASS" in mission and "TRANSIENT_LOCAL" in mission)
check("mission supervisor has no motion publisher", "create_publisher(Twist" not in mission)

check("Phase L resolves injector path", 'INJECTOR_PATH="$REPO_DIR/scripts/inject_dynamic_obstacle.py"' in launch)
check("Phase L starts injector", 'python3 "$INJECTOR_PATH" --ros-args' in launch)
check("Phase L injector injection params", "injection_distance:=2.0" in launch and "min_motion:=0.1" in launch and "auto_inject:=true" in launch)
check("Phase L cost_weight 2.5", "cost_weight:=2.5" in launch)
check("Phase L evaluator expanded", "goal_topic:=/goal_pose" in launch and "cost_map_topic:=/lunabot/terrain/cost_map" in launch and "dynamic_obstacle_topic:=/lunabot/dynamic_obstacle/status" in launch and "goal_tolerance:=0.35" in launch)
check("Phase L mission expanded topics", "dynamic_obstacle_topic:=/lunabot/dynamic_obstacle/status" in launch and "cost_map_topic:=/lunabot/terrain/cost_map" in launch and "semantic_map_topic:=/lunabot/terrain/semantic_map" in launch)
check("Phase L starts mission supervisor directly", 'python3 "$MISSION_PATH" --ros-args' in launch)
check("Phase L validates mission status type", 'type_ok "type mission status std_msgs/String"' in launch)
check("Phase L validates mission status topic", 'topic_ok "topic /lunabot/mission/status"' in launch)
check("Phase L validates obstacle topics", 'topic_ok "topic /lunabot/obstacles/status"' in launch)
check("Phase L waits for mission pass", "MISSION_DEMO_PASS" in launch)
check("Phase L records mission evidence", "mission.log" in launch and "mission_status.txt" in launch)
check("Phase L records injector evidence", "dynamic_obstacle.log" in launch)
check("Phase L retains aggregate evaluation gate", "aggregate runtime evaluation: PASS" in launch)
check("Phase L retains dynamic replanning gate", "dynamic terrain-plan replanning: PASS" in launch)

for phrase, name in [
    ("Final Mission Demonstration", "scope"),
    ("MISSION_DEMO_PASS", "mission acceptance"),
    ("/lunabot/mission/status", "status output"),
]:
    check(f"docs contain {name}", phrase in docs)

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE L STATIC VALIDATION (injector + 5-class + real metrics)")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " + ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-l-launch-l/static_validation.txt"
report.parent.mkdir(parents=True, exist_ok=True)
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
