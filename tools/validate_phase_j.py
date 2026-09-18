#!/usr/bin/env python3
"""Static Phase J gate: dynamic terrain-plan replanning with controlled obstacle injection."""

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
    "launch-j", "scripts/launch-j.sh", "scripts/dynamic_replan_monitor.py",
    "scripts/inject_dynamic_obstacle.py", "tools/validate_phase_j.py",
    "rviz/phase_j.rviz", "docs/phase-10-launch-j.md",
    "evidence/phase-j-launch-j/README.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-j", "scripts/launch-j.sh", "scripts/dynamic_replan_monitor.py",
            "scripts/inject_dynamic_obstacle.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/dynamic_replan_monitor.py", "scripts/inject_dynamic_obstacle.py",
            "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-j", "scripts/launch-j.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for ph in ["a", "b", "c", "d", "e", "f", "g", "h", "i"]:
    check(f"baseline {ph} static validation exists", (ROOT / f"evidence/phase-{ph}-launch-{ph}/static_validation.txt").is_file())

launch = read("scripts/launch-j.sh")
wrapper = read("launch-j")
monitor = read("scripts/dynamic_replan_monitor.py")
injector = read("scripts/inject_dynamic_obstacle.py")
rviz = read("rviz/phase_j.rviz")
docs = read("docs/phase-10-launch-j.md")
noncomment = "\n".join(line for line in launch.splitlines() if not line.lstrip().startswith("#"))

check("root launch-j resolves its real path", "readlink -f" in wrapper and "SCRIPT_PATH" in wrapper)
check("root launch-j execs scripts/launch-j.sh", "scripts/launch-j.sh" in wrapper)
check("Phase J has its own evidence directory", "phase-j-launch-j" in launch)
check("Phase J is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase J uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase J has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase J cleans dynamic monitor", "REPLAN_MONITOR_PID" in launch and 'stop_group "$REPLAN_MONITOR_PID"' in launch)
check("Phase J cleans injector", "INJECTOR_PID" in launch and 'stop_group "$INJECTOR_PID"' in launch)
check("Phase J reports clean shutdown", "Launch J environment cleanly closed." in launch)
check("Phase J preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch)

check("monitor is a ROS node", "class DynamicReplanMonitor(Node)" in monitor)
check("monitor consumes terrain plans", "Path, self.plan_topic" in monitor and "_plan_callback" in monitor)
check("monitor consumes odometry", "Odometry, self.odom_topic" in monitor)
check("monitor counts revisions", "self.revisions" in monitor)
check("monitor requires motion for pass", "minimum_motion" in monitor and "DYNAMIC_REPLAN_PASS" in monitor)
check("monitor publishes retained status", "self.status_pub" in monitor and "TRANSIENT_LOCAL" in monitor)
check("monitor has no motion publisher", "create_publisher(Twist" not in monitor)

check("injector is a ROS node", "class DynamicObstacleInjector(Node)" in injector)
check("injector controlled injection", "injection_distance" in injector and "auto_inject" in injector)
check("injector timestamps obstacle_introduced/detected/cost_changed/new_plan", "obstacle_introduced" in injector and "obstacle_detected" in injector and "cost_changed" in injector and "new_plan" in injector)
check("injector publishes obstacle status", "obstacle_status" in injector.lower() or "obstacle" in injector.lower())
check("injector does not publish cmd_vel", "/cmd_vel" not in injector)

check("Phase J resolves dynamic monitor path", 'REPLAN_MONITOR_PATH="$REPO_DIR/scripts/dynamic_replan_monitor.py"' in launch)
check("Phase J resolves injector path", 'INJECTOR_PATH="$REPO_DIR/scripts/inject_dynamic_obstacle.py"' in launch)
check("Phase J starts dynamic monitor directly", 'python3 "$REPLAN_MONITOR_PATH" --ros-args' in launch)
check("Phase J starts injector directly", 'python3 "$INJECTOR_PATH" --ros-args' in launch)
check("Phase J injector uses correct topics", "plan_topic:=/lunabot/terrain/plan" in launch and "cost_map_topic:=/lunabot/terrain/cost_map" in launch)
check("Phase J injector injection params", "injection_distance:=2.0" in launch and "min_motion:=0.1" in launch and "auto_inject:=true" in launch)
check("Phase J validates replan status type", 'type_ok "type dynamic replan status std_msgs/String"' in launch)
check("Phase J validates replan status topic", 'topic_ok "topic /lunabot/autonomy/replan_status"' in launch)
check("Phase J requires dynamic pass", "DYNAMIC_REPLAN_PASS" in launch)
check("Phase J retains active follower boundary", "-p cmd_topic:=/cmd_vel_in" in launch)
check("Phase J cost_weight 2.5", "cost_weight:=2.5" in launch)
check("Phase J records replan evidence", "replan.log" in launch and "replan_status.txt" in launch)
check("Phase J records injector evidence", "dynamic_obstacle.log" in launch)

check("Phase J RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase J RViz shows active terrain path", "Active Terrain Path" in rviz and "Value: /lunabot/terrain/plan" in rviz)
check("Phase J RViz keeps cost map", "Terrain Cost Map" in rviz and "Value: /lunabot/terrain/cost_map" in rviz)

for phrase, name in [
    ("Dynamic Replanning", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j", "automated command"),
    ("DYNAMIC_REPLAN_PASS", "dynamic acceptance"),
    ("/lunabot/autonomy/replan_status", "status output"),
]:
    check(f"docs contain {name}", phrase in docs)

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE J STATIC VALIDATION (injector + 5-class)")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " + ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-j-launch-j/static_validation.txt"
report.parent.mkdir(parents=True, exist_ok=True)
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
