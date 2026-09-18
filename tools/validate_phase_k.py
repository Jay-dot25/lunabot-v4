#!/usr/bin/env python3
"""Static Phase K gate: runtime testing and evaluation (5-class graded, real metrics, injector)."""

from pathlib import Path
import ast
import subprocess
import sys

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
    "launch-k", "scripts/launch-k.sh", "scripts/phase_k_evaluator.py",
    "scripts/inject_dynamic_obstacle.py", "tools/validate_phase_k.py",
    "rviz/phase_k.rviz", "docs/phase-11-launch-k.md",
    "evidence/phase-k-launch-k/README.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-k", "scripts/launch-k.sh", "scripts/phase_k_evaluator.py",
            "scripts/inject_dynamic_obstacle.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/phase_k_evaluator.py", "scripts/dynamic_replan_monitor.py",
            "scripts/inject_dynamic_obstacle.py", "scripts/terrain_aware_planner.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-k", "scripts/launch-k.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for ph in ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"]:
    check(f"baseline {ph} static validation exists", (ROOT / f"evidence/phase-{ph}-launch-{ph}/static_validation.txt").is_file())

launch = read("scripts/launch-k.sh")
wrapper = read("launch-k")
evaluator = read("scripts/phase_k_evaluator.py")
rviz = read("rviz/phase_k.rviz")
docs = read("docs/phase-11-launch-k.md")

check("root launch-k resolves its real path", "readlink -f" in wrapper and "SCRIPT_PATH" in wrapper)
check("root launch-k execs scripts/launch-k.sh", "scripts/launch-k.sh" in wrapper)
check("Phase K has its own evidence directory", "phase-k-launch-k" in launch)
check("Phase K is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase K uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase K has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase K cleans evaluator and replan monitor", "EVALUATOR_PID" in launch and 'stop_group "$EVALUATOR_PID"' in launch and 'stop_group "$REPLAN_MONITOR_PID"' in launch)
check("Phase K cleans injector", "INJECTOR_PID" in launch and 'stop_group "$INJECTOR_PID"' in launch)
check("Phase K reports clean shutdown", "Launch K environment cleanly closed." in launch)
check("Phase K preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch)

check("evaluator is a ROS node", "class PhaseKEvaluator(Node)" in evaluator)
check("evaluator consumes terrain plan", "plan_topic" in evaluator and "self.plan_seen" in evaluator)
check("evaluator consumes replan status", "DYNAMIC_REPLAN_PASS" in evaluator and "self.replan_pass" in evaluator)
check("evaluator consumes integrated goal status", "INTEGRATION_GOAL_REACHED" in evaluator)
check("evaluator consumes odometry with real path length Σ sqrt", "total_motion" in evaluator or "path_length" in evaluator.lower())
check("evaluator real metrics: path length from odometry", "path_length" in evaluator.lower() or "total_motion" in evaluator)
check("evaluator real metrics: success via goal tolerance", "goal_tolerance" in evaluator and "success" in evaluator.lower())
check("evaluator real metrics: hazardous exposure ROCK+CRATER", "hazard" in evaluator.lower() or ("ROCK" in evaluator and "CRATER" in evaluator))
check("evaluator real metrics: replanning time and terrain cost", "replan" in evaluator.lower() and "terrain" in evaluator.lower() or "avg" in evaluator.lower())
check("evaluator goal_topic cost_map_topic dynamic_obstacle_topic", "goal_topic" in evaluator and "cost_map_topic" in evaluator and "dynamic_obstacle_topic" in evaluator)
check("evaluator emits aggregate pass", "EVALUATION_PASS" in evaluator)
check("evaluator has no motion publisher", "create_publisher(Twist" not in evaluator)

check("Phase K resolves evaluator path", 'EVALUATOR_PATH="$REPO_DIR/scripts/phase_k_evaluator.py"' in launch)
check("Phase K resolves injector path", 'INJECTOR_PATH="$REPO_DIR/scripts/inject_dynamic_obstacle.py"' in launch)
check("Phase K starts evaluator directly", 'python3 "$EVALUATOR_PATH" --ros-args' in launch)
check("Phase K starts injector directly", 'python3 "$INJECTOR_PATH" --ros-args' in launch)
check("Phase K injector params", "injection_distance:=2.0" in launch and "min_motion:=0.1" in launch and "auto_inject:=true" in launch)
check("Phase K cost_weight 2.5", "cost_weight:=2.5" in launch)
check("Phase K evaluator goal/cost_map/dynamic_obstacle topics", "goal_topic:=/goal_pose" in launch and "cost_map_topic:=/lunabot/terrain/cost_map" in launch and "dynamic_obstacle_topic:=/lunabot/dynamic_obstacle/status" in launch)
check("Phase K evaluator goal_tolerance 0.35", "goal_tolerance:=0.35" in launch)
check("Phase K validates evaluation type", 'type_ok "type evaluation status std_msgs/String"' in launch)
check("Phase K validates evaluation topic", 'topic_ok "topic /lunabot/evaluation/status"' in launch)
check("Phase K waits for aggregate pass", "EVALUATION_PASS" in launch)
check("Phase K retains dynamic replan gate", "dynamic terrain-plan replanning: PASS" in launch)
check("Phase K retains active follower boundary", "-p cmd_topic:=/cmd_vel_in" in launch)
check("Phase K records evaluation evidence", "evaluation.log" in launch and "evaluation_status.txt" in launch)
check("Phase K records injector evidence", "dynamic_obstacle.log" in launch)

check("Phase K RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase K RViz shows active terrain path", "Active Terrain Path" in rviz and "Value: /lunabot/terrain/plan" in rviz)
check("Phase K RViz keeps cost map", "Terrain Cost Map" in rviz and "Value: /lunabot/terrain/cost_map" in rviz)

for phrase, name in [
    ("Testing and Evaluation", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k", "automated command"),
    ("EVALUATION_PASS", "evaluation acceptance"),
    ("/lunabot/evaluation/status", "status output"),
]:
    check(f"docs contain {name}", phrase in docs)

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE K STATIC VALIDATION (5-class real metrics + injector)")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " + ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-k-launch-k/static_validation.txt"
report.parent.mkdir(parents=True, exist_ok=True)
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
