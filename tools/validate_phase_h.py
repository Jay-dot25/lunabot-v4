#!/usr/bin/env python3
"""Static Phase H gate: terrain-aware weighted path planning.

This validator checks the independent launch-h contract and the approved
Phase A-G baseline. It never claims that a real terrain-aware path was
published; that requires the workstation command documented in
 docs/phase-8-launch-h.md.
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


required = [
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/terrain_cost_mapper.py", "scripts/terrain_aware_planner.py",
    "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz", "rviz/phase_d.rviz",
    "rviz/phase_e.rviz", "rviz/phase_f.rviz", "rviz/phase_g.rviz", "rviz/phase_h.rviz",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py", "tools/validate_phase_c.py",
    "tools/validate_phase_d.py", "tools/validate_phase_e.py", "tools/validate_phase_f.py", "tools/validate_phase_g.py",
    "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md", "docs/phase-5-launch-e.md",
    "docs/phase-6-launch-f.md", "docs/phase-7-launch-g.md", "docs/phase-8-launch-h.md",
    "evidence/phase-g-launch-g/README.md", "evidence/phase-h-launch-h/README.md",
    "evidence/phase-h-launch-h/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf", "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-h", "scripts/launch-h.sh", "scripts/terrain_aware_planner.py", "tools/validate_phase_h.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/terrain_aware_planner.py", "scripts/terrain_cost_mapper.py", "scripts/semantic_terrain_mapper.py",
            "scripts/terrain_segmentation.py", "scripts/astar_navigation.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "tools/validate_phase_a.py", "tools/validate_phase_b.py",
            "tools/validate_phase_c.py", "tools/validate_phase_d.py", "tools/validate_phase_e.py",
            "tools/validate_phase_f.py", "tools/validate_phase_g.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g", "launch-h",
            "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh", "scripts/launch-d.sh",
            "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh", "scripts/launch-h.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

for rel, expected in [("tools/validate_phase_a.py", "80/80"),
                      ("tools/validate_phase_b.py", "103/103"),
                      ("tools/validate_phase_c.py", "133/133"),
                      ("tools/validate_phase_d.py", "152/152"),
                      ("tools/validate_phase_e.py", "112/112"),
                      ("tools/validate_phase_f.py", "122/122"),
                      ("tools/validate_phase_g.py", "131/131")]:
    r = run([sys.executable, rel])
    check(f"approved baseline remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-h.sh")
wrapper = read("launch-h")
planner = read("scripts/terrain_aware_planner.py")
rviz = read("rviz/phase_h.rviz")
docs = read("docs/phase-8-launch-h.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Independent launch and inherited safety.
check("root launch-h resolves its real path", "readlink -f" in wrapper and "SCRIPT_PATH" in wrapper)
check("root launch-h execs scripts/launch-h.sh", "scripts/launch-h.sh" in wrapper)
check("Phase H does not invoke an earlier launcher", "launch-g" not in noncomment and
      "launch-f" not in noncomment and "launch-e" not in noncomment and "launch-d" not in noncomment and
      "launch-c" not in noncomment and "launch-b" not in noncomment and "launch-a" not in noncomment)
check("Phase H has its own evidence directory", "phase-h-launch-h" in launch and
      "phase-g-launch-g" not in noncomment)
check("Phase H is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase H uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase H has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase H cleans terrain planner process", "TERRAIN_PLANNER_PID" in launch and
      'stop_group "$TERRAIN_PLANNER_PID"' in launch)
check("Phase H reports its clean shutdown", "Launch H environment cleanly closed." in launch)
check("Phase H has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase H preserves cost mapper", 'COST_PATH="$REPO_DIR/scripts/terrain_cost_mapper.py"' in launch)
check("Phase H preserves the Phase D planner", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase H preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + planner).lower() and "nav2_amcl" not in (launch + planner).lower())

# Terrain-aware planner implementation.
check("terrain planner is a ROS node", "class TerrainAwarePlanner(Node)" in planner)
check("planner consumes cost map", "OccupancyGrid, self.cost_map_topic" in planner and "_cost_callback" in planner)
check("planner consumes the existing goal", "PoseStamped, self.goal_topic" in planner and "_goal_callback" in planner)
check("planner uses existing localization", "lookup_transform" in planner and
      "self.map_frame" in planner and "self.base_frame" in planner)
check("planner uses weighted A*", "heapq" in planner and "cost_weight" in planner and
      "weighted_cost" in planner)
check("planner blocks high-cost cells", "blocked_cost" in planner and "_cell_cost" in planner)
check("planner uses eight-connected neighbors", "(1, 1), (1, -1), (-1, 1), (-1, -1)" in planner)
check("planner publishes terrain Path", "Path" in planner and "self.plan_pub.publish(path)" in planner)
check("planner publishes auditable status", "TERRAIN_PLAN_PASS" in planner and "self.status_pub" in planner)
check("planner plan/status are late-join safe", planner.count("TRANSIENT_LOCAL") >= 1 and
      "map_qos" in planner)
check("planner does not publish motion commands", "/cmd_vel" not in planner)

# Phase H launcher/runtime evidence.
check("Phase H resolves terrain planner path", 'TERRAIN_PLANNER_PATH="$REPO_DIR/scripts/terrain_aware_planner.py"' in launch)
check("Phase H starts planner directly", 'python3 "$TERRAIN_PLANNER_PATH" --ros-args' in launch)
check("Phase H validates plan type", 'type_ok "type terrain plan nav_msgs/Path"' in launch)
check("Phase H validates planner status type", 'type_ok "type terrain planner status std_msgs/String"' in launch)
check("Phase H validates real plan messages", 'topic_ok "topic /lunabot/terrain/plan"' in launch and
      'topic_ok "topic /lunabot/terrain/planner/status"' in launch)
check("Phase H validates plan content", "TERRAIN_PLAN_PASS" in launch and
      "terrain-aware plan content: PASS" in launch and
      '[[ "$sample" == *"poses: []"* ]]' in launch)
check("Phase H records planner evidence", "terrain_plan.txt" in launch and
      "terrain_planner_status.txt" in launch and "terrain_planner.log" in launch)
check("Phase H retains cost content gate", "COST_MAP_PASS" in launch and
      "cost map content: PASS" in launch)
check("Phase H retains semantic content gate", "SEMANTIC_MAP_PASS" in launch and
      "semantic map content: PASS" in launch)
check("Phase H retains A* goal gate", "wait_for_goal" in launch and "A* goal reached" in launch)
check("Phase H retains map motion evidence", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch)

# RViz and documentation.
check("Phase H RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase H RViz keeps A* path", "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase H RViz keeps cost map", "Terrain Cost Map" in rviz and
      "Value: /lunabot/terrain/cost_map" in rviz)
check("Phase H RViz shows terrain-aware plan", "Terrain-aware Plan" in rviz and
      "Value: /lunabot/terrain/plan" in rviz)
check("Phase H RViz uses sensor-compatible QoS", rviz.count("Reliability Policy: Best Effort") >= 4)
for phrase, name in [
    ("Terrain-aware path planning", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h", "automated command"),
    ("TERRAIN_PLAN_PASS", "plan acceptance"),
    ("/lunabot/terrain/plan", "plan output"),
    ("/lunabot/terrain/cost_map", "cost input"),
    ("weighted", "weighted search"),
    ("without motion output", "motion isolation"),
    ("clean relaunch", "relaunch"),
    ("Phase I", "Phase I gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_h.py") and
      "workstation" in read("tools/validate_phase_h.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE H STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-h-launch-h/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
