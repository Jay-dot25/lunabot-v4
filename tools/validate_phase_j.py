#!/usr/bin/env python3
"""Static Phase J gate: dynamic terrain-plan replanning.

This validator checks the independent launch-j contract and the approved
Phase A-I baseline. It never claims that dynamic replanning occurred on a
real rover; that requires the workstation command documented in
 docs/phase-10-launch-j.md.
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
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h", "launch-i", "launch-j",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh", "scripts/launch-d.sh",
    "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh", "scripts/launch-i.sh", "scripts/launch-j.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/terrain_cost_mapper.py", "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py", "scripts/dynamic_replan_monitor.py",
    "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_i.rviz", "rviz/phase_j.rviz",
    "tools/validate_phase_i.py", "tools/validate_phase_j.py",
    "docs/phase-9-launch-i.md", "docs/phase-10-launch-j.md",
    "evidence/phase-i-launch-i/README.md", "evidence/phase-i-launch-i/verification_checklist.md",
    "evidence/phase-j-launch-j/README.md", "evidence/phase-j-launch-j/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf", "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-j", "scripts/launch-j.sh", "scripts/dynamic_replan_monitor.py", "tools/validate_phase_j.py"]:
    path = ROOT / rel
    check(f"executable: {rel}", path.is_file() and bool(path.stat().st_mode & 0o111))

for rel in ["scripts/dynamic_replan_monitor.py", "scripts/terrain_aware_planner.py",
            "scripts/terrain_path_follower.py", "tools/validate_phase_i.py", "tools/validate_phase_j.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-j", "scripts/launch-j.sh"]:
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
    for phase in "abcdefghi"
]
baseline_state = {
    rel: (ROOT / rel).read_bytes() if (ROOT / rel).exists() else None
    for rel in baseline_reports
}
result = run_preserving_reports([sys.executable, "tools/validate_phase_i.py"], baseline_reports)
check("approved Phase I baseline remains green", result.returncode == 0 and
      "160/160" in result.stdout, result.stdout[-180:].strip())

launch = read("scripts/launch-j.sh")
wrapper = read("launch-j")
monitor = read("scripts/dynamic_replan_monitor.py")
rviz = read("rviz/phase_j.rviz")
docs = read("docs/phase-10-launch-j.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Restore inherited reports after the Phase I validator's recursive checks.
for rel, content in baseline_state.items():
    path = ROOT / rel
    if content is None:
        if path.exists():
            path.unlink()
    else:
        path.write_bytes(content)

check("root launch-j resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-j execs scripts/launch-j.sh", "scripts/launch-j.sh" in wrapper)
check("Phase J is independent of launch-i", "launch-i" not in noncomment and
      "launch-h" not in noncomment and "launch-g" not in noncomment)
check("Phase J has its own evidence directory", "phase-j-launch-j" in launch and
      "phase-i-launch-i" not in noncomment)
check("Phase J is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase J uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase J has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase J cleans the dynamic monitor", "REPLAN_MONITOR_PID" in launch and
      'stop_group "$REPLAN_MONITOR_PID"' in launch and
      "scripts/dynamic_replan_monitor.py" in launch)
check("Phase J reports clean shutdown", "Launch J environment cleanly closed." in launch)
check("Phase J preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + monitor).lower() and "nav2_amcl" not in (launch + monitor).lower())

# Dynamic monitor contract.
check("monitor is a ROS node", "class DynamicReplanMonitor(Node)" in monitor)
check("monitor consumes terrain plans", "Path, self.plan_topic" in monitor and
      "_plan_callback" in monitor)
check("monitor consumes real odometry", "Odometry, self.odom_topic" in monitor and
      "self.total_motion" in monitor)
check("monitor consumes the active goal", "PoseStamped, self.goal_topic" in monitor and
      "self.goal_seen" in monitor)
check("monitor counts changed revisions", "self.revisions += 1" in monitor and
      "signature != self.last_signature" in monitor)
check("monitor requires motion for pass", "self.minimum_motion" in monitor and
      "DYNAMIC_REPLAN_PASS" in monitor)
check("monitor latches a successful pass", "if self.passed:" in monitor and
      "Keep the successful marker latched" in monitor)
check("monitor publishes retained status", "self.status_pub" in monitor and
      "TRANSIENT_LOCAL" in monitor)
check("monitor has no motion publisher", "create_publisher(Twist" not in monitor and
      "/cmd_vel" not in monitor)

# Launcher integration contract.
check("Phase J resolves dynamic monitor path", 'REPLAN_MONITOR_PATH="$REPO_DIR/scripts/dynamic_replan_monitor.py"' in launch)
check("Phase J starts dynamic monitor directly", 'python3 "$REPLAN_MONITOR_PATH" --ros-args' in launch)
check("Phase J validates replan status type", 'type_ok "type dynamic replan status std_msgs/String"' in launch)
check("Phase J validates replan status topic", 'topic_ok "topic /lunabot/autonomy/replan_status"' in launch)
check("Phase J requires dynamic pass", "DYNAMIC_REPLAN_PASS" in launch and
      "dynamic terrain-plan replanning: PASS" in launch)
check("Phase J retains active follower boundary", "-p cmd_topic:=/cmd_vel_in" in launch and
      "cmd_vel_in_motion.txt" in launch)
check("Phase J retains diagnostic command isolation", "cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel" in launch)
check("Phase J uses reliable retained replan QoS", "--qos-reliability reliable --qos-durability transient_local" in launch)
check("Phase J records replan evidence", "replan.log" in launch and
      "replan_status.txt" in launch and "replan_wait_status.txt" in launch)
check("Phase J requires final map evidence", "phase_j_map.yaml" in launch and
      "phase_j_map.pgm" in launch and
      "saved map evidence (YAML + PGM): PASS" in launch)

# RViz and documentation.
check("Phase J RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase J RViz shows active terrain path", "Active Terrain Path" in rviz and
      "Value: /lunabot/terrain/plan" in rviz)
check("Phase J RViz keeps diagnostic path", "Name: A* Path" in rviz and
      "Value: /plan" in rviz)
check("Phase J RViz keeps cost map", "Terrain Cost Map" in rviz and
      "Value: /lunabot/terrain/cost_map" in rviz)
for phrase, name in [
    ("Dynamic Replanning", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j", "automated command"),
    ("DYNAMIC_REPLAN_PASS", "dynamic acceptance"),
    ("/lunabot/autonomy/replan_status", "status output"),
    ("two changed terrain-plan revisions", "revision requirement"),
    ("clean relaunch", "relaunch"),
    ("Phase K", "Phase K gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_j.py") and
      "workstation" in read("tools/validate_phase_j.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE J STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-j-launch-j/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
sys.exit(0 if passed == len(checks) else 1)
