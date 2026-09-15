#!/usr/bin/env python3
"""Static Phase I gate: full terrain-aware autonomous integration.

This validator checks the independent launch-i contract and the approved
Phase A-H baseline. It never claims that a real integrated rover reached a
goal; that requires the workstation command documented in
 docs/phase-9-launch-i.md.
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
    """Run an inherited validator without rewriting its existing evidence."""
    originals = {}
    for path in paths:
        target = ROOT / path
        originals[path] = target.read_bytes() if target.exists() else None
    result = run(cmd)
    for path, content in originals.items():
        target = ROOT / path
        if content is None:
            if target.exists():
                target.unlink()
        else:
            target.write_bytes(content)
    return result


required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h", "launch-i",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh", "scripts/launch-d.sh",
    "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh", "scripts/launch-i.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/terrain_cost_mapper.py", "scripts/terrain_aware_planner.py", "scripts/terrain_path_follower.py",
    "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz", "rviz/phase_d.rviz",
    "rviz/phase_e.rviz", "rviz/phase_f.rviz", "rviz/phase_g.rviz", "rviz/phase_h.rviz", "rviz/phase_i.rviz",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py", "tools/validate_phase_c.py", "tools/validate_phase_d.py",
    "tools/validate_phase_e.py", "tools/validate_phase_f.py", "tools/validate_phase_g.py", "tools/validate_phase_h.py",
    "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md", "docs/phase-5-launch-e.md", "docs/phase-6-launch-f.md",
    "docs/phase-7-launch-g.md", "docs/phase-8-launch-h.md", "docs/phase-9-launch-i.md",
    "evidence/phase-h-launch-h/README.md", "evidence/phase-i-launch-i/README.md",
    "evidence/phase-i-launch-i/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf", "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-i", "scripts/launch-i.sh", "scripts/terrain_path_follower.py", "tools/validate_phase_i.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/terrain_path_follower.py", "scripts/terrain_aware_planner.py", "scripts/terrain_cost_mapper.py",
            "scripts/semantic_terrain_mapper.py", "scripts/terrain_segmentation.py", "scripts/astar_navigation.py",
            "scripts/control_odometry.py", "scripts/odometry_monitor.py", "tools/validate_phase_a.py",
            "tools/validate_phase_b.py", "tools/validate_phase_c.py", "tools/validate_phase_d.py",
            "tools/validate_phase_e.py", "tools/validate_phase_f.py", "tools/validate_phase_g.py", "tools/validate_phase_h.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h", "launch-i",
            "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh", "scripts/launch-d.sh",
            "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh", "scripts/launch-i.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

baseline_reports = [
    f"evidence/phase-{phase}-launch-{phase}/static_validation.txt"
    for phase in "abcdefgh"
]
baseline_report_state = {
    path: (ROOT / path).read_bytes() if (ROOT / path).exists() else None
    for path in baseline_reports
}
for rel, expected, report in [("tools/validate_phase_a.py", "76/76", "evidence/phase-a-launch-a/static_validation.txt"),
                              ("tools/validate_phase_b.py", "103/103", "evidence/phase-b-launch-b/static_validation.txt"),
                              ("tools/validate_phase_c.py", "133/133", "evidence/phase-c-launch-c/static_validation.txt"),
                              ("tools/validate_phase_d.py", "149/149", "evidence/phase-d-launch-d/static_validation.txt"),
                              ("tools/validate_phase_e.py", "112/112", "evidence/phase-e-launch-e/static_validation.txt"),
                              ("tools/validate_phase_f.py", "122/122", "evidence/phase-f-launch-f/static_validation.txt"),
                              ("tools/validate_phase_g.py", "131/131", "evidence/phase-g-launch-g/static_validation.txt"),
                              ("tools/validate_phase_h.py", "144/144", "evidence/phase-h-launch-h/static_validation.txt")]:
    r = run_preserving_reports([sys.executable, rel], [report])
    check(f"approved baseline remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

# Some inherited validators recursively run older validators. Restore every
# inherited report after the complete chain so this gate is non-destructive.
for path, content in baseline_report_state.items():
    target = ROOT / path
    if content is None:
        if target.exists():
            target.unlink()
    else:
        target.write_bytes(content)

launch = read("scripts/launch-i.sh")
wrapper = read("launch-i")
follower = read("scripts/terrain_path_follower.py")
rviz = read("rviz/phase_i.rviz")
docs = read("docs/phase-9-launch-i.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Independent launch and inherited safety.
check("root launch-i resolves its real path", "readlink -f" in wrapper and "SCRIPT_PATH" in wrapper)
check("root launch-i execs scripts/launch-i.sh", "scripts/launch-i.sh" in wrapper)
check("Phase I does not invoke an earlier launcher", "launch-h" not in noncomment and
      "launch-g" not in noncomment and "launch-f" not in noncomment and "launch-e" not in noncomment and
      "launch-d" not in noncomment and "launch-c" not in noncomment and "launch-b" not in noncomment and
      "launch-a" not in noncomment)
check("Phase I has its own evidence directory", "phase-i-launch-i" in launch and
      "phase-h-launch-h" not in noncomment)
check("Phase I is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase I uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase I has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase I cleans active follower", "FOLLOWER_PID" in launch and
      'stop_group "$FOLLOWER_PID"' in launch)
check("Phase I reports its clean shutdown", "Launch I environment cleanly closed." in launch)
check("Phase I has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase I preserves the terrain planner", 'TERRAIN_PLANNER_PATH="$REPO_DIR/scripts/terrain_aware_planner.py"' in launch)
check("Phase I preserves the Phase D planner diagnostically", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch and
      "diagnostic_cmd_vel" in launch)
check("Phase I preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + follower).lower() and "nav2_amcl" not in (launch + follower).lower())

# Active follower implementation.
check("follower is a ROS node", "class TerrainPathFollower(Node)" in follower)
check("follower consumes terrain path", "Path, self.path_topic" in follower and "_path_callback" in follower)
check("follower consumes real odometry", "Odometry, self.odom_topic" in follower and "_odom_callback" in follower)
check("follower consumes the goal", "PoseStamped, self.goal_topic" in follower and "_goal_callback" in follower)
check("follower uses existing localization", "lookup_transform" in follower and
      "self.map_frame" in follower and "self.base_frame" in follower)
check("follower publishes into controller boundary", "self.cmd_topic" in follower and
      "self.cmd_pub.publish(cmd)" in follower and "/cmd_vel_in" in follower)
check("follower has conservative limits", "max_linear" in follower and "max_angular" in follower)
check("follower publishes integrated status", "INTEGRATION_GOAL_REACHED" in follower and
      "self.status_pub" in follower)
check("follower stops safely", "_publish_stop" in follower and "Twist()" in follower)
check("follower latches a completed goal safely", "if self.reached:" in follower and
      "if self.reached:\n            self._publish_stop()" in follower)
check("follower does not publish final cmd_vel", 'cmd_topic", "/cmd_vel"' not in follower)

# Phase I launcher/runtime integration.
check("Phase I resolves follower path", 'FOLLOWER_PATH="$REPO_DIR/scripts/terrain_path_follower.py"' in launch)
check("Phase I starts follower directly", 'python3 "$FOLLOWER_PATH" --ros-args' in launch)
check("Phase I isolates diagnostic A* command", "cmd_topic:=/lunabot/navigation/diagnostic_cmd_vel" in launch)
check("Phase I validates integration status type", 'type_ok "type integration status std_msgs/String"' in launch)
check("Phase I validates real integration status", 'topic_ok "topic /lunabot/autonomy/status"' in launch)
check("Phase I validates integrated goal content", "INTEGRATION_GOAL_REACHED" in launch and
      "integration status content: PASS" in launch)
check("Phase I defines motion capture before use",
      launch.index("capture_motion_evidence() {") < launch.index("\ncapture_motion_evidence\n"))
check("Phase I captures active input motion", "capture_motion_evidence" in launch and
      "cmd_vel_in_motion.txt" in launch and "nonzero /cmd_vel_in motion evidence: PASS" in launch)
check("Phase I validates controller-boundary motion", "cmd_vel_motion.txt" in launch and
      "controller_boundary_status.txt" in launch and
      "controller boundary evidence (/cmd_vel_in -> /cmd_vel): PASS" in launch)
check("Phase I retrieves retained status with compatible QoS",
      "--qos-reliability reliable --qos-durability transient_local" in launch)
check("terrain planner guards ROS shutdown", "if rclpy.ok():" in read("scripts/terrain_aware_planner.py"))
check("Phase I records integration evidence", "integration_status.txt" in launch and
      "integration.log" in launch)
check("Phase I retains terrain plan gate", "terrain-aware plan content: PASS" in launch)
check("Phase I retains cost/semantic gates", "cost map content: PASS" in launch and
      "semantic map content: PASS" in launch)
check("Phase I retains map motion evidence", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch)
check("Phase I requires final map evidence", "nav2_map_server unavailable" in launch and
      "phase_i_map.yaml" in launch and "phase_i_map.pgm" in launch and
      "saved map evidence (YAML + PGM): PASS" in launch)

# RViz and documentation.
check("Phase I RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase I RViz shows active terrain path", "Active Terrain Path" in rviz and
      "Value: /lunabot/terrain/plan" in rviz)
check("Phase I RViz keeps diagnostic A* path", "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase I RViz keeps cost map", "Terrain Cost Map" in rviz and
      "Value: /lunabot/terrain/cost_map" in rviz)
check("Phase I RViz uses sensor-compatible QoS", rviz.count("Reliability Policy: Best Effort") >= 4)
for phrase, name in [
    ("Full Autonomous Integration", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i", "automated command"),
    ("INTEGRATION_GOAL_REACHED", "integration acceptance"),
    ("/lunabot/autonomy/status", "status output"),
    ("/cmd_vel_in", "controller boundary"),
    ("diagnostic_cmd_vel", "command isolation"),
    ("without motion output", "phase H distinction"),
    ("clean relaunch", "relaunch"),
    ("Phase J", "Phase J gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_i.py") and
      "workstation" in read("tools/validate_phase_i.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE I STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-i-launch-i/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
