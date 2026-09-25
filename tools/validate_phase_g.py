#!/usr/bin/env python3
"""Static Phase G gate: terrain cost-map generation.

This validator checks the independent launch-g contract and the approved
Phase A-F baseline. It never claims that a real cost map was generated; that
requires the workstation command documented in docs/phase-7-launch-g.md.
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
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py",
    "scripts/odometry_monitor.py", "scripts/astar_navigation.py",
    "scripts/terrain_segmentation.py", "scripts/semantic_terrain_mapper.py",
    "scripts/terrain_cost_mapper.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "rviz/phase_d.rviz", "rviz/phase_e.rviz", "rviz/phase_f.rviz", "rviz/phase_g.rviz",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py",
    "tools/validate_phase_c.py", "tools/validate_phase_d.py", "tools/validate_phase_e.py",
    "tools/validate_phase_f.py", "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md",
    "docs/phase-5-launch-e.md", "docs/phase-6-launch-f.md", "docs/phase-7-launch-g.md",
    "evidence/phase-f-launch-f/README.md", "evidence/phase-g-launch-g/README.md",
    "evidence/phase-g-launch-g/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-g", "scripts/launch-g.sh", "scripts/terrain_cost_mapper.py",
            "tools/validate_phase_g.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/terrain_cost_mapper.py", "scripts/semantic_terrain_mapper.py",
            "scripts/terrain_segmentation.py", "scripts/astar_navigation.py",
            "scripts/control_odometry.py", "scripts/odometry_monitor.py",
            "tools/validate_phase_a.py", "tools/validate_phase_b.py",
            "tools/validate_phase_c.py", "tools/validate_phase_d.py",
            "tools/validate_phase_e.py", "tools/validate_phase_f.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d", "launch-e", "launch-f", "launch-g",
            "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
            "scripts/launch-d.sh", "scripts/launch-e.sh", "scripts/launch-f.sh", "scripts/launch-g.sh"]:
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
                      ("tools/validate_phase_f.py", "122/122")]:
    r = run([sys.executable, rel])
    check(f"approved baseline remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-g.sh")
wrapper = read("launch-g")
cost = read("scripts/terrain_cost_mapper.py")
rviz = read("rviz/phase_g.rviz")
docs = read("docs/phase-7-launch-g.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Independent launch and inherited safety.
check("root launch-g resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-g execs scripts/launch-g.sh", "scripts/launch-g.sh" in wrapper)
check("Phase G does not invoke an earlier launcher", "launch-f" not in noncomment and
      "launch-e" not in noncomment and "launch-d" not in noncomment and
      "launch-c" not in noncomment and "launch-b" not in noncomment and
      "launch-a" not in noncomment)
check("Phase G has its own evidence directory", "phase-g-launch-g" in launch and
      "phase-f-launch-f" not in noncomment)
check("Phase G is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase G uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase G has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase G cleans cost mapper process", "COST_PID" in launch and
      'stop_group "$COST_PID"' in launch)
check("Phase G reports its clean shutdown", "Launch G environment cleanly closed." in launch)
check("Phase G has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase G preserves semantic mapper", 'MAPPER_PATH="$REPO_DIR/scripts/semantic_terrain_mapper.py"' in launch)
check("Phase G preserves the Phase D planner", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase G preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + cost).lower() and
      "nav2_amcl" not in (launch + cost).lower())

# Cost mapper implementation.
check("cost mapper is a ROS node", "class TerrainCostMapper(Node)" in cost)
check("cost mapper consumes semantic map", "OccupancyGrid, self.semantic_topic" in cost and
      "_semantic_callback" in cost)
check("cost mapper publishes cost map", "self.cost_pub.publish" in cost and
      "OccupancyGrid" in cost)
check("cost mapper defines terrain cost", "class_costs" in cost and
      "BEDROCK" in cost and "REGOLITH" in cost and "ROCK" in cost and
      "CRATER" in cost and "SHADOW" in cost)
check("cost mapper defines unknown caution cost", "unknown_cost" in cost and
      "UNKNOWN = -1" in cost)
check("cost mapper defines obstacle cost", "obstacle_cost" in cost and
      "OBSTACLE_VALUE" in cost)
check("cost mapper inflates obstacles", "inflation_radius" in cost and
      "_inflate" in cost and "inflated" in cost)
check("cost mapper preserves map geometry", "output.info = msg.info" in cost and
      "output.header = msg.header" in cost)
check("cost mapper publishes auditable status", "COST_MAP_PASS" in cost and
      "self.status_pub" in cost)
check("cost mapper map/status are late-join safe", cost.count("TRANSIENT_LOCAL") >= 2 and
      "input_qos" in cost and "output_qos" in cost)
check("cost mapper does not publish motion commands", "/cmd_vel" not in cost)

# Phase G launcher/runtime evidence.
check("Phase G resolves cost mapper path", 'COST_PATH="$REPO_DIR/scripts/terrain_cost_mapper.py"' in launch)
check("Phase G starts cost mapper directly", 'python3 "$COST_PATH" --ros-args' in launch)
check("Phase G validates cost map type", 'type_ok "type cost map nav_msgs/OccupancyGrid"' in launch)
check("Phase G validates cost status type", 'type_ok "type cost map status std_msgs/String"' in launch)
check("Phase G validates real cost messages", 'topic_ok "topic /lunabot/terrain/cost_map"' in launch and
      'topic_ok "topic /lunabot/terrain/cost_map/status"' in launch)
check("Phase G validates cost content", "COST_MAP_PASS" in launch and
      "cost map content: PASS" in launch)
check("Phase G records cost evidence", "cost_map_sample.txt" in launch and
      "cost_map_status.txt" in launch and "cost_mapping.log" in launch)
check("Phase G retains semantic content gate", "SEMANTIC_MAP_PASS" in launch and
      "semantic map content: PASS" in launch)
check("Phase G retains A* goal gate", "wait_for_goal" in launch and
      "A* goal reached" in launch)
check("Phase G retains map motion evidence", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch)

# RViz and documentation.
check("Phase G RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase G RViz keeps A* path", "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase G RViz keeps semantic map", "Semantic Terrain Map" in rviz and
      "Value: /lunabot/terrain/semantic_map" in rviz)
check("Phase G RViz shows cost map", "Terrain Cost Map" in rviz and
      "Value: /lunabot/terrain/cost_map" in rviz)
check("Phase G RViz uses sensor-compatible QoS", rviz.count("Reliability Policy: Best Effort") >= 4)
for phrase, name in [
    ("Terrain cost map", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g", "automated command"),
    ("COST_MAP_PASS", "cost acceptance"),
    ("/lunabot/terrain/cost_map", "cost map output"),
    ("semantic map", "semantic input"),
    ("inflation", "obstacle inflation"),
    ("clean relaunch", "relaunch"),
    ("Phase H", "Phase H gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_g.py") and
      "workstation" in read("tools/validate_phase_g.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE G STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-g-launch-g/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
