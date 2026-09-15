#!/usr/bin/env python3
"""Static Phase L gate: final mission demonstration.

Static checks protect the approved Phase A-K baseline. A real workstation run
is still required for MISSION_DEMO_PASS; this validator never substitutes for
that runtime gate.
"""

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


def run_preserving(cmd, paths):
    saved = {}
    for rel in paths:
        path = ROOT / rel
        saved[rel] = path.read_bytes() if path.exists() else None
    result = run(cmd)
    for rel, content in saved.items():
        path = ROOT / rel
        if content is None:
            if path.exists():
                path.unlink()
        else:
            path.write_bytes(content)
    return result


required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f",
    "launch-g", "launch-h", "launch-i", "launch-j", "launch-k", "launch-l",
    "scripts/launch-k.sh", "scripts/launch-l.sh",
    "scripts/phase_k_evaluator.py", "scripts/phase_l_mission.py",
    "scripts/obstacle_detector.py", "scripts/terrain_cost_mapper.py",
    "tools/validate_phase_k.py", "tools/validate_phase_l.py",
    "docs/phase-11-launch-k.md", "docs/phase-12-launch-l.md",
    "evidence/phase-k-launch-k/README.md",
    "evidence/phase-k-launch-k/verification_checklist.md",
    "evidence/phase-l-launch-l/README.md",
    "evidence/phase-l-launch-l/verification_checklist.md",
    "rviz/phase_k.rviz", "rviz/phase_l.rviz",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-l", "scripts/launch-l.sh", "scripts/phase_l_mission.py",
            "tools/validate_phase_l.py"]:
    path = ROOT / rel
    check(f"executable: {rel}", path.is_file() and bool(path.stat().st_mode & 0o111))

for rel in ["scripts/phase_l_mission.py", "scripts/phase_k_evaluator.py",
            "scripts/obstacle_detector.py", "scripts/astar_navigation.py",
            "scripts/terrain_cost_mapper.py", "scripts/dynamic_replan_monitor.py",
            "scripts/terrain_aware_planner.py",
            "scripts/terrain_path_follower.py", "tools/validate_phase_l.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-l", "scripts/launch-l.sh"]:
    result = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", result.returncode == 0, result.stderr.strip())

for rel, label in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                   ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{label} SDF well-formed", True)
    except Exception as exc:
        check(f"{label} SDF well-formed", False, str(exc))

baseline_reports = [
    "evidence/phase-k-launch-k/static_validation.txt",
    *[f"evidence/phase-{p}-launch-{p}/static_validation.txt"
      for p in "abcdefghij"],
]
result = run_preserving([sys.executable, "tools/validate_phase_k.py"],
                        baseline_reports)
check("approved Phase K baseline remains green", result.returncode == 0 and
      "105/105" in result.stdout, result.stdout[-220:].strip())

launch = read("scripts/launch-l.sh")
wrapper = read("launch-l")
mission = read("scripts/phase_l_mission.py")
rviz = read("rviz/phase_l.rviz")
docs = read("docs/phase-12-launch-l.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

check("root launch-l resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-l execs scripts/launch-l.sh", "scripts/launch-l.sh" in wrapper)
check("Phase L is independent of launch-k", "launch-k" not in noncomment and
      "launch-j" not in noncomment)
check("Phase L uses its own evidence directory", "phase-l-launch-l" in launch and
      "phase-k-launch-k" not in noncomment)
check("Phase L is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase L uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase L has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase L cleans mission observer", 'stop_group "$MISSION_PID"' in launch and
      'MISSION_PID=""' in launch)
check("Phase L cleans inherited evaluator", 'stop_group "$EVALUATOR_PID"' in launch)
check("Phase L reports clean shutdown", "Launch L environment cleanly closed." in launch)
check("Phase L has 23 independent stages", "[23/23]" in launch and
      "[13/23]" in launch and "[22/23]" in launch)
check("Phase L preserves Phase K evaluator", "phase_k_evaluator.py" in launch and
      "EVALUATION_PASS" in launch)
check("Phase L starts real obstacle detector", 'python3 "$OBSTACLE_PATH" --ros-args' in launch and
      "/lunabot/obstacles/status" in launch and "/lunabot/obstacles/map" in launch)
check("Phase L cleans obstacle detector", 'stop_group "$OBSTACLE_PID"' in launch)
check("Phase L supports manual final mode", "FINAL_DEMO" in launch and
      'AUTO_GOAL="${AUTO_GOAL:-false}"' in launch and
      "wait_for_goal_selection" in launch)
check("Phase L uses its own RViz config", "rviz/phase_l.rviz" in launch and
      "rviz/phase_k.rviz" not in launch)
check("RViz provides manual Set Goal tool", "rviz_default_plugins/SetGoal" in rviz and
      "Topic: /goal_pose" in rviz)
check("RViz shows sensed obstacle map", "Sensed Obstacle Map" in rviz and
      "Value: /lunabot/obstacles/map" in rviz)
check("RViz shows forward obstacle markers", "Forward Obstacle Returns" in rviz and
      "Value: /lunabot/obstacles/markers" in rviz)
check("RViz shows selected goal", "Selected Goal" in rviz)
check("Phase L preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + mission).lower() and
      "nav2_amcl" not in (launch + mission).lower())
check("Phase L retains diagnostic command isolation", "cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel" in launch)
check("Phase L retains the single active motion source", "cmd_topic:=/cmd_vel_in" in launch and
      "cmd_vel_in_motion.txt" in launch)
check("Phase L requires final map evidence", "phase_l_map.yaml" in launch and
      "phase_l_map.pgm" in launch and
      "saved map evidence (YAML + PGM): PASS" in launch)
world = read("src/lunabot_gazebo/worlds/lunar_world.sdf")
check("world contains habitat presentation models", "lunar_habitat_main" in world and
      "lunar_habitat_equipment" in world)
check("world contains a physical presentation obstacle", "presentation_obstacle_forward" in world and
      "obstacle_collision" in world)

# Observation-only mission supervisor contract.
check("mission supervisor is a ROS node", "class PhaseLMission(Node)" in mission)
check("mission supervisor observes terrain plan", "Path" in mission and
      "self.plan_seen" in mission)
check("mission supervisor observes replanning", "DYNAMIC_REPLAN_PASS" in mission and
      "self.replan_pass" in mission)
check("mission supervisor observes integrated goal", "INTEGRATION_GOAL_REACHED" in mission and
      "self.goal_reached" in mission)
check("mission supervisor observes aggregate evaluation", "EVALUATION_PASS" in mission and
      "self.evaluation_pass" in mission)
check("mission supervisor observes manual-goal status", "MANUAL_GOAL_SELECTED" in mission and
      "self.manual_goal_seen" in mission and "require_manual_goal" in mission)
check("mission supervisor observes obstacle status", "OBSTACLE_DETECTED" in mission and
      "self.obstacle_detected" in mission)
check("mission supervisor observes real goal", "PoseStamped" in mission and
      "self.goal_seen" in mission)
check("mission supervisor observes map", "OccupancyGrid" in mission and
      "self.map_seen" in mission)
check("mission supervisor emits retained pass", "MISSION_DEMO_PASS" in mission and
      "self.status_pub" in mission and "TRANSIENT_LOCAL" in mission)
check("mission supervisor has no motion publisher", "create_publisher(Twist" not in mission and
      "geometry_msgs.msg.Twist" not in mission)
check("mission supervisor does not publish a goal", "create_publisher(PoseStamped" not in mission)
obstacle = read("scripts/obstacle_detector.py")
check("obstacle detector is observation-only", "LaserScan" in obstacle and
      "create_publisher(Twist" not in obstacle and
      "OBSTACLE_DETECTED" in obstacle)
check("cost map consumes sensed obstacle overlay", "obstacle_topic" in read("scripts/terrain_cost_mapper.py") and
      "sensed_obstacles" in read("scripts/terrain_cost_mapper.py"))

# Launcher integration and runtime acceptance.
check("Phase L starts mission supervisor directly", 'python3 "$MISSION_PATH" --ros-args' in launch)
check("Phase L validates mission status type", 'type_ok "type mission status std_msgs/String"' in launch)
check("Phase L validates mission status topic", 'topic_ok "topic /lunabot/mission/status"' in launch and
      "/lunabot/mission/status|" in launch)
check("Phase L validates obstacle topics", 'topic_ok "topic /lunabot/obstacles/status"' in launch and
      'topic_ok "topic /lunabot/obstacles/map"' in launch)
check("Phase L waits for mission pass", "MISSION_DEMO_PASS" in launch and
      "final mission demonstration: PASS" in launch)
check("Phase L records mission evidence", "mission.log" in launch and
      "mission_status.txt" in launch and "mission_wait_status.txt" in launch)
check("Phase L retains aggregate evaluation gate", "aggregate runtime evaluation: PASS" in launch)
check("Phase L retains dynamic replanning gate", "dynamic terrain-plan replanning: PASS" in launch and
      "require_obstacle:=true" in launch)
check("Phase L uses retained mission QoS", "--qos-reliability reliable --qos-durability transient_local" in launch and
      "/lunabot/mission/status" in launch)

for phrase, name in [
    ("Final Mission Demonstration", "scope"),
    ("DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false", "automated command"),
    ("MISSION_DEMO_PASS", "mission acceptance"),
    ("/lunabot/mission/status", "status output"),
    ("do not publish velocity", "safety boundary"),
    ("clean relaunch", "relaunch"),
    ("no later phase is defined", "final-phase gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never substitutes" in read("tools/validate_phase_l.py") and
      "real workstation run" in read("tools/validate_phase_l.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE L STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-l-launch-l/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
sys.exit(0 if passed == len(checks) else 1)
