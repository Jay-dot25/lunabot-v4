#!/usr/bin/env python3
"""Static Phase E gate: RGB-D terrain perception.

This validator checks the independent launch-e contract and the approved
Phase A-D baseline. It never claims that a real camera frame was segmented;
that requires the workstation command documented in docs/phase-5-launch-e.md.
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
    "launch-a", "launch-b", "launch-c", "launch-d", "launch-e",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/launch-e.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py",
    "scripts/odometry_monitor.py", "scripts/astar_navigation.py",
    "scripts/terrain_segmentation.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "rviz/phase_d.rviz", "rviz/phase_e.rviz",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py",
    "tools/validate_phase_c.py", "tools/validate_phase_d.py",
    "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md",
    "docs/phase-5-launch-e.md", "evidence/phase-d-launch-d/README.md",
    "evidence/phase-e-launch-e/README.md",
    "evidence/phase-e-launch-e/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-e", "scripts/launch-e.sh", "scripts/terrain_segmentation.py",
            "tools/validate_phase_e.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/terrain_segmentation.py", "scripts/astar_navigation.py",
            "scripts/control_odometry.py", "scripts/odometry_monitor.py",
            "tools/validate_phase_a.py", "tools/validate_phase_b.py",
            "tools/validate_phase_c.py", "tools/validate_phase_d.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d", "launch-e",
            "scripts/launch-a.sh", "scripts/launch-b.sh",
            "scripts/launch-c.sh", "scripts/launch-d.sh", "scripts/launch-e.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

# The approved baseline remains a required static dependency.
for rel, expected in [("tools/validate_phase_a.py", "76/76"),
                      ("tools/validate_phase_b.py", "103/103"),
                      ("tools/validate_phase_c.py", "133/133"),
                      ("tools/validate_phase_d.py", "149/149")]:
    r = run([sys.executable, rel])
    check(f"approved baseline remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-e.sh")
wrapper = read("launch-e")
perception = read("scripts/terrain_segmentation.py")
rviz = read("rviz/phase_e.rviz")
docs = read("docs/phase-5-launch-e.md")
noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))

# Independent launch and inherited safety.
check("root launch-e resolves its real path", "readlink -f" in wrapper and
      "SCRIPT_PATH" in wrapper)
check("root launch-e execs scripts/launch-e.sh", "scripts/launch-e.sh" in wrapper)
check("Phase E does not invoke an earlier launcher", "launch-d" not in noncomment and
      "launch-c" not in noncomment and "launch-b" not in noncomment and
      "launch-a" not in noncomment)
check("Phase E has its own evidence directory", "phase-e-launch-e" in launch and
      "phase-d-launch-d" not in noncomment)
check("Phase E is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase E uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase E has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase E cleans perception process", "PERCEPTION_PID" in launch and
      'stop_group "$PERCEPTION_PID"' in launch)
check("Phase E has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase E preserves the Phase D planner", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase E preserves one SLAM system", "ros2 launch slam_toolbox online_async_launch.py" in launch and
      "cartographer" not in (launch + perception).lower() and
      "nav2_amcl" not in (launch + perception).lower())

# RGB-D segmentation implementation.
check("perception is a ROS node", "class TerrainSegmentation(Node)" in perception)
check("perception consumes RGB images", "Image, self.image_topic" in perception and
      "_image_callback" in perception)
check("perception consumes depth images", "Image, self.depth_topic" in perception and
      "_depth_callback" in perception)
check("perception decodes supported depth encodings", "16uc1" in perception and
      "32fc1" in perception and "_depth_at" in perception)
check("perception publishes a mono8 mask", '"mono8"' in perception and
      "self.mask_pub.publish" in perception)
check("perception publishes an RGB overlay", '"rgb8"' in perception and
      "self.overlay_pub.publish" in perception)
check("perception defines explicit labels", "UNKNOWN = 0" in perception and
      "TERRAIN = 1" in perception and "OBSTACLE = 2" in perception)
check("perception handles invalid depth", "math.isfinite(distance)" in perception and
      "unknown_count" in perception)
check("perception uses a conservative ground ROI", "ground_roi_start" in perception and
      "saturation_threshold" in perception)
check("perception publishes auditable status", "SEGMENTATION_PASS" in perception and
      "self.status_pub" in perception)
check("perception status is late-join safe", "TRANSIENT_LOCAL" in perception and
      "status_qos" in perception)
check("perception does not alter navigation commands", "/cmd_vel" not in perception)

# Phase E launcher carries the sensor and runtime contracts.
check("Phase E resolves the perception path", 'PERCEPTION_PATH="$REPO_DIR/scripts/terrain_segmentation.py"' in launch)
check("Phase E starts perception directly", 'python3 "$PERCEPTION_PATH" --ros-args' in launch)
check("Phase E bridges the existing depth topic", '"/lunabot/depth/image_raw@sensor_msgs/msg/Image' in launch)
check("Phase E keeps the existing RGB topic", '"/lunabot/camera/image_raw@sensor_msgs/msg/Image' in launch)
check("Phase E validates depth at runtime", 'topic_ok "topic /lunabot/depth/image_raw"' in launch)
check("Phase E validates mask type", 'type_ok "type terrain mask sensor_msgs/Image"' in launch)
check("Phase E validates overlay type", 'type_ok "type terrain overlay sensor_msgs/Image"' in launch)
check("Phase E validates status type", 'type_ok "type terrain status std_msgs/String"' in launch)
check("Phase E validates real segmentation messages", 'topic_ok "topic /lunabot/terrain/segmentation"' in launch and
      'topic_ok "topic /lunabot/terrain/overlay"' in launch)
check("Phase E validates segmentation content", "SEGMENTATION_PASS" in launch and
      "terrain segmentation content: PASS" in launch)
check("Phase E records segmentation evidence", "segmentation_sample.txt" in launch and
      "segmentation_status.txt" in launch and "segmentation.log" in launch)
check("Phase E retains the A* goal gate", "wait_for_goal" in launch and
      "A* goal reached" in launch)
check("Phase E retains map motion evidence", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch)

# RViz and documentation.
check("Phase E RViz keeps map fixed frame", "Fixed Frame: map" in rviz)
check("Phase E RViz keeps A* path", "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase E RViz shows segmentation overlay", "Terrain Segmentation Overlay" in rviz and
      "Value: /lunabot/terrain/overlay" in rviz)
check("Phase E RViz includes mask display", "Terrain Segmentation Mask" in rviz and
      "Value: /lunabot/terrain/segmentation" in rviz)
check("Phase E RViz uses sensor-compatible QoS", rviz.count("Reliability Policy: Best Effort") >= 4)
for phrase, name in [
    ("Terrain perception", "scope"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e", "automated command"),
    ("SEGMENTATION_PASS", "segmentation acceptance"),
    ("/lunabot/terrain/segmentation", "mask output"),
    ("/lunabot/terrain/overlay", "overlay output"),
    ("RGB-D", "RGB-D inputs"),
    ("clean relaunch", "relaunch"),
    ("Phase F", "Phase F gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "never claims" in read("tools/validate_phase_e.py") and
      "workstation" in read("tools/validate_phase_e.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE E STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-e-launch-e/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
