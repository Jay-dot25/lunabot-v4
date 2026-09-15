#!/usr/bin/env python3
"""Static Phase K gate: runtime testing and evaluation.

This validator checks the independent launch-k contract and the approved
Phase A-J baseline. It never claims that a real evaluation passed; that
requires the workstation command documented in docs/phase-11-launch-k.md.
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


def run_preserving_reports(cmd, paths):
    originals = {}
    for rel in paths:
        path = ROOT / rel
        originals[rel] = path.read_bytes() if path.exists() else None
    result = run(cmd)
    for rel, content in originals.items():
        path = ROOT / rel
        if content is None:
            if path.exists():
                path.unlink()
        else:
            path.write_bytes(content)
    return result


required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h", "launch-i", "launch-j", "launch-k",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh", "scripts/launch-d.sh",
    "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh", "scripts/launch-i.sh", "scripts/launch-j.sh", "scripts/launch-k.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/terrain_cost_mapper.py", "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py",
    "scripts/dynamic_replan_monitor.py", "scripts/phase_k_evaluator.py",
    "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_j.rviz", "rviz/phase_k.rviz",
    "tools/validate_phase_j.py", "tools/validate_phase_k.py",
    "docs/phase-10-launch-j.md", "docs/phase-11-launch-k.md",
    "evidence/phase-j-launch-j/README.md", "evidence/phase-j-launch-j/verification_checklist.md",
    "evidence/phase-k-launch-k/README.md", "evidence/phase-k-launch-k/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf", "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-k", "scripts/launch-k.sh", "scripts/phase_k_evaluator.py", "tools/validate_phase_k.py"]:
    path = ROOT / rel
    check(f"executable: {rel}", path.is_file() and bool(path.stat().st_mode & 0o111))

for rel in ["scripts/phase_k_evaluator.py", "scripts/dynamic_replan_monitor.py",
            "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py",
            "tools/validate_phase_j.py", "tools/validate_phase_k.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-k", "scripts/launch-k.sh"]:
    result = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", result.returncode == 0, result.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

baseline_reports = [
    f"evidence/phase-{phase}-launch-{phase}/static_validation.txt"
    for phase in "abcdefghij"
]
baseline_state = {
    rel: (ROOT / rel).read_bytes() if (ROOT / rel).exists() else None
    for rel in baseline_reports
}
result = run_preserving_reports([sys.executable, "tools/validate_phase_j.py"], baseline_reports)
check("approved Phase J baseline remains green", result.returncode == 0 and
      "99/99" in result.stdout, result.stdout[-180:].strip())

launch = read("scripts/launch-k.sh")
wrapper = read("launch-k")
evaluator = read("scripts/phase_k_evaluator.py")
rviz = read("rviz/phase_k.rviz")
docs = read("docs/phase-11-launch-k.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

for rel, content in baseline_state.items():
    path = ROOT / rel
    if content is None:
        if path.exists():
            path.unlink()
    else:
        path.write_bytes(content)

check("root launch-k resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-k execs scripts/launch-k.sh", "scripts/launch-k.sh" in wrapper)
check("Phase K is independent of launch-j", "launch-j" not in noncomment and
      "launch-i" not in noncomment and "launch-h" not in noncomment)
check("Phase K has its own evidence directory", "phase-k-launch-k" in launch and
      "phase-j-launch-j" not in noncomment)
check("Phase K is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase K uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase K has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase K cleans evaluator and replan monitor", "EVALUATOR_PID" in launch and
      'stop_group "$EVALUATOR_PID"' in launch and
      'stop_group "$REPLAN_MONITOR_PID"' in launch)
check("Phase K reports clean shutdown", "Launch K environment cleanly closed." in launch)
check("Phase K cleans evaluator on relaunch", "scripts/phase_k_evaluator.py" in launch)
check("Phase K preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + evaluator).lower() and
      "nav2_amcl" not in (launch + evaluator).lower())

# Evaluator contract.
check("evaluator is a ROS node", "class PhaseKEvaluator(Node)" in evaluator)
check("evaluator consumes terrain plan", "Path, str(get(\"plan_topic\").value)" in evaluator and
      "self.plan_seen" in evaluator)
check("evaluator consumes replan status", "DYNAMIC_REPLAN_PASS" in evaluator and
      "self.replan_pass" in evaluator)
check("evaluator consumes integrated goal status", "INTEGRATION_GOAL_REACHED" in evaluator and
      "self.goal_reached" in evaluator)
check("evaluator consumes controller status", "self.controller_active" in evaluator and
      "ACTIVE" in evaluator)
check("evaluator consumes odometry", "self.total_motion" in evaluator and
      "Odometry" in evaluator)
check("evaluator checks both command boundaries", "self.input_motion" in evaluator and
      "self.output_motion" in evaluator)
check("evaluator emits aggregate pass", "EVALUATION_PASS" in evaluator and
      "self.status_pub" in evaluator)
check("evaluator latches aggregate pass", "if self.passed:" in evaluator)
check("evaluator has no motion publisher", "create_publisher(Twist" not in evaluator and
      "self.status_pub = self.create_publisher(String" in evaluator)

# Launcher integration.
check("Phase K resolves evaluator path", 'EVALUATOR_PATH="$REPO_DIR/scripts/phase_k_evaluator.py"' in launch)
check("Phase K starts evaluator directly", 'python3 "$EVALUATOR_PATH" --ros-args' in launch)
check("Phase K validates evaluation type", 'type_ok "type evaluation status std_msgs/String"' in launch)
check("Phase K validates evaluation topic", 'topic_ok "topic /lunabot/evaluation/status"' in launch)
check("Phase K waits for aggregate pass", "EVALUATION_PASS" in launch and
      "aggregate runtime evaluation: PASS" in launch)
check("Phase K retains dynamic replan gate", "dynamic terrain-plan replanning: PASS" in launch)
check("Phase K retains active follower boundary", "-p cmd_topic:=/cmd_vel_in" in launch and
      "cmd_vel_in_motion.txt" in launch)
check("Phase K retains diagnostic command isolation", "cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel" in launch)
check("Phase K uses reliable evaluation QoS", "--qos-reliability reliable --qos-durability transient_local" in launch and
      "/lunabot/evaluation/status" in launch)
check("Phase K records evaluation evidence", "evaluation.log" in launch and
      "evaluation_status.txt" in launch and "evaluation_wait_status.txt" in launch)
check("Phase K requires final map evidence", "phase_k_map.yaml" in launch and
      "phase_k_map.pgm" in launch and
      "saved map evidence (YAML + PGM): PASS" in launch)

# RViz and docs.
check("Phase K RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase K RViz shows active terrain path", "Active Terrain Path" in rviz and
      "Value: /lunabot/terrain/plan" in rviz)
check("Phase K RViz keeps diagnostic path", "Name: A* Path" in rviz and
      "Value: /plan" in rviz)
check("Phase K RViz keeps cost map", "Terrain Cost Map" in rviz and
      "Value: /lunabot/terrain/cost_map" in rviz)
for phrase, name in [
    ("Testing and Evaluation", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k", "automated command"),
    ("EVALUATION_PASS", "evaluation acceptance"),
    ("/lunabot/evaluation/status", "status output"),
    ("both sides of the `/cmd_vel_in -> /cmd_vel` boundary", "boundary metrics"),
    ("clean relaunch", "relaunch"),
    ("Phase L", "Phase L gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_k.py") and
      "workstation" in read("tools/validate_phase_k.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE K STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-k-launch-k/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
sys.exit(0 if passed == len(checks) else 1)
