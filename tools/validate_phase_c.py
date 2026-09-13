#!/usr/bin/env python3
"""Static Phase C gate: SLAM & Localization.

This validator checks repository structure and the independent launch contract.
It deliberately does not claim that ROS 2, Gazebo, slam_toolbox, /map, TF, or
IMU runtime data work; those checks belong to the workstation run of
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c.
"""

from pathlib import Path
import ast
import re
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


# Required Phase C deliverables and inherited interfaces.
required = [
    "launch-a", "launch-b", "launch-c", "scripts/launch-a.sh",
    "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py",
    "scripts/odometry_monitor.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "docs/phase-b-launch-b.md", "docs/phase-3-launch-c.md",
    "evidence/phase-b-launch-b/README.md",
    "evidence/phase-c-launch-c/README.md",
    "evidence/phase-c-launch-c/verification_checklist.md",
    "evidence/phase-c-launch-c/collect_evidence.sh",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-c", "scripts/launch-c.sh", "tools/validate_phase_c.py",
            "evidence/phase-c-launch-c/collect_evidence.sh"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

# Baseline syntax/structure remains valid.
for rel in ["scripts/wasd_teleop.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "tools/validate_phase_a.py",
            "tools/validate_phase_b.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c",
            "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

# The existing Phase A and Phase B gates are still present. Run the baseline
# validator when this repository's Python environment permits it.
for rel in ["tools/validate_phase_a.py", "tools/validate_phase_b.py"]:
    r = run([sys.executable, rel])
    expected = "76/76" if rel.endswith("phase_a.py") else "103/103"
    check(f"existing static gate remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-c.sh")
wrapper = read("launch-c")
config = read("config/slam_toolbox_phase_c.yaml")
rviz = read("rviz/phase_c.rviz")
world = read("src/lunabot_gazebo/worlds/lunar_world.sdf")
model = read("src/lunabot_gazebo/models/lunabot_v4/model.sdf")
docs = read("docs/phase-3-launch-c.md")

# Independent, symlink-safe entry point.
check("root launch-c resolves its real path", "readlink -f" in wrapper)
check("root launch-c execs scripts/launch-c.sh", "scripts/launch-c.sh" in wrapper)
noncomment_launch = "\n".join(
    line for line in launch.splitlines() if not line.lstrip().startswith("#"))
check("Phase C script does not invoke launch-a",
      not re.search(r"(^|[;&|()\s])(?:bash\s+)?[^\n]*launch-a", noncomment_launch))
check("Phase C script does not invoke launch-b", "launch-b" not in noncomment_launch)
check("Phase C script uses its own evidence directory",
      "phase-c-launch-c" in launch and "phase-b-launch-b" not in noncomment_launch)
check("Phase C has symlink-safe script directory resolution",
      'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
collector = read("evidence/phase-c-launch-c/collect_evidence.sh")
check("Phase C evidence collector resolves the repository", "readlink -f" in collector and
      "scripts/launch-c.sh" in collector)
check("Phase C evidence collector enables evidence and demo", "DEMO=1 EVIDENCE=1" in collector)
check("Phase C uses clean process-group startup", "setsid" in launch)
check("Phase C has trap-controlled shutdown", "trap 'shutdown 130' INT TERM" in launch)
check("Phase C shuts down slam_toolbox first", 'kill -TERM "$SLAM_PID"' in launch)
check("Phase C shuts down control and odometry", 'kill -TERM "$CONTROL_PID"' in launch and
      'kill -TERM "$ODOM_PID"' in launch)
check("Phase C shuts down Gazebo process group", 'kill -TERM -"$GAZEBO_PID"' in launch)
check("Phase C cleans stale SLAM processes", "slam_toolbox.*online_async" in launch)

# Exactly one SLAM system and the explicit slam_toolbox wiring.
check("slam_toolbox is the selected SLAM system", "slam_toolbox" in config and
      "slam_toolbox" in launch)
check("no Cartographer configuration", "cartographer" not in config.lower() and
      "cartographer" not in launch.lower())
check("no AMCL configuration", "amcl" not in config.lower() and
      "amcl" not in launch.lower())
check("no second mapping/localization stack", not any(x in config.lower() for x in
      ["hector_slam", "karto_slam", "rtabmap"]))
check("slam toolbox launch executable is explicit",
      "ros2 launch slam_toolbox online_async_launch.py" in launch)
check("SLAM config path is explicit", 'slam_params_file:="$SLAM_CONFIG"' in launch)
check("slam toolbox uses simulation time", "use_sim_time:=true" in launch and
      "use_sim_time: true" in config)
check("mapping mode is explicit", "mode: mapping" in config)
check("scan topic is explicit in config", "scan_topic: /lunabot/lidar/scan" in config)
check("map frame is explicit", "map_frame: map" in config)
check("odom frame is explicit", "odom_frame: odom" in config)
check("base frame is explicit", "base_frame: chassis" in config)
check("SLAM publishes at a map update interval", "map_update_interval:" in config)
check("SLAM loop closure remains enabled", "do_loop_closing: true" in config)
check("SLAM transform publish period is configured", "transform_publish_period:" in config)

# Inherited Phase B startup/control/sensor contract.
check("world and rover paths are inherited", 'WORLD_PATH="$WORLD_DIR/lunar_world.sdf"' in launch and
      'MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"' in launch)
check("Phase C keeps the validated spawn height", 'SPAWN_Z="-2.308"' in launch)
check("headless mode is supported", 'HEADLESS="${HEADLESS:-0}"' in launch and
      'gazebo -s' in launch)
check("Gazebo is explicitly unpaused", "WorldControl" in launch and
      "--req 'pause: false'" in launch)
check("Gazebo waits for readiness", 'world/$WORLD_NAME/info' in launch and
      'for _ in $(seq 1 90)' in launch)
check("bridge detects ros_ign or ros_gz", "ros_ign_bridge" in launch and
      "ros_gz_bridge" in launch)
check("bridge includes simulation clock", '"/clock@rosgraph_msgs/msg/Clock' in launch)
check("bridge includes explicit LaserScan", '"/lunabot/lidar/scan@sensor_msgs/msg/LaserScan' in launch)
check("bridge includes explicit camera", '"/lunabot/camera/image_raw@sensor_msgs/msg/Image' in launch)
check("bridge includes explicit IMU", '"/lunabot/imu@sensor_msgs/msg/Imu' in launch)
check("bridge includes preserved odometry", '"/lunabot/odom@nav_msgs/msg/Odometry' in launch)
check("static TF includes chassis to sensor head", '"chassis|sensor_head|' in launch)
check("static TF includes sensor head to lidar", '"sensor_head|lidar|' in launch)
check("control input boundary is preserved", "--input-topic /cmd_vel_in" in launch)
check("control output boundary is preserved", "--output-topic /cmd_vel" in launch)
check("watchdog remains configured", "--watchdog-sec 0.5" in launch)
check("odometry monitor is started", "odometry_monitor.py" in launch and "ODOM_PID" in launch)
check("IMU Fortress plugin remains enabled", "ignition-gazebo-imu-system" in world and
      "ignition::gazebo::systems::Imu" in world)
check("model publishes the required odometry topic", "<odom_topic>/lunabot/odom</odom_topic>" in model)
check("model odometry frame pair is unchanged", "<frame_id>odom</frame_id>" in model and
      "<child_frame_id>chassis</child_frame_id>" in model)

# Runtime validation must wait for actual messages, including the IMU/map/scan.
check("runtime validation uses topic echo once", 'sample="$(timeout 30 ros2 topic echo' in launch and
      "--once" in launch)
check("map message is runtime-validated", 'topic_ok "topic /map"' in launch)
check("map type is OccupancyGrid-validated", "nav_msgs/msg/OccupancyGrid" in launch and
      'topic type /map' in launch)
check("map sample structure is OccupancyGrid-shaped", '[[ "$sample" != *"info:"* ]]' in launch and
      '[[ "$sample" != *"data:"* ]]' in launch)
check("map to odom TF is runtime-validated", 'tf_ok "TF map -> odom" "map" "odom"' in launch)
check("odom topic is runtime-validated", 'topic_ok "topic /lunabot/odom"' in launch)
check("odom to chassis TF is runtime-validated", 'tf_ok "TF odom -> chassis" "odom" "chassis"' in launch)
check("LaserScan topic is runtime-validated", 'topic_ok "topic /lunabot/lidar/scan"' in launch)
check("camera topic is runtime-validated", 'topic_ok "topic /lunabot/camera/image_raw"' in launch)
check("IMU topic is runtime-validated", 'topic_ok "topic /lunabot/imu"' in launch)
check("IMU failure diagnostics are preserved", "imu_gazebo_topics.txt" in launch and
      "imu_topic_info.txt" in launch and "imu_diagnostics.txt" in launch)
check("sensor-frame TF is runtime-validated", 'tf_ok "TF chassis -> sensor_head"' in launch)
check("map motion has before and after samples", "map_before_motion.txt" in launch and
      "map_after_motion.txt" in launch)
check("map motion gate compares real samples", "cmp -s" in launch and
      "live map updates during rover motion: PASS" in launch)
check("demo failures propagate to process exit", "demo_rc" in launch and
      'shutdown "$EXIT_CODE"' in launch)
check("clean shutdown message is explicit", "Launch C environment cleanly closed." in launch)

# Map saving is optional only when the package is unavailable; if available,
# the launcher tries the official map_saver_cli and leaves a log.
check("map saver package is detected without a false pass",
      "MAP_SAVER_AVAILABLE" in launch and "nav2_map_server" in launch)
check("official map_saver_cli is used", "map_saver_cli -f" in launch)
check("map saver result is recorded", "map_saver.log" in launch)
check("map evidence path is documented", "phase_c_map.yaml" in docs and
      "phase_c_map.pgm" in docs)

# Phase C RViz must retain the Phase B QoS/topic fixes and add map display.
check("Phase C RViz fixed frame is odom", "Fixed Frame: odom" in rviz)
check("Phase C RViz has a map display", "rviz_default_plugins/Map" in rviz and
      "Name: SLAM Map" in rviz)
check("Phase C RViz map topic is explicit", "Value: /map" in rviz)
check("Phase C RViz LaserScan topic is explicit", "Value: /lunabot/lidar/scan" in rviz)
check("Phase C RViz camera topic is explicit", "Value: /lunabot/camera/image_raw" in rviz)
check("Phase C RViz sensor QoS remains best effort", rviz.count("Reliability Policy: Best Effort") >= 2)
check("Phase C RViz odometry topic is explicit", "Value: /lunabot/odom" in rviz)
check("Phase C RViz TF tree includes map and odom", "map:\n          odom:" in rviz)

# Documentation must make the workstation gate and recovery steps clear.
for phrase, name in [
    ("sudo apt install -y", "Ubuntu installation commands"),
    ("ros-humble-slam-toolbox", "slam toolbox package instruction"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c", "automated launch instruction"),
    ("live map updates during rover motion: PASS", "motion-map acceptance"),
    ("map -> odom", "map TF acceptance"),
    ("/lunabot/imu", "IMU acceptance"),
    ("clean relaunch", "relaunch instruction"),
    ("before Phase D", "Phase D gate"),
]:
    check(f"docs contain {name}", phrase in docs)

# The validator itself is deliberately honest about static versus runtime.
check("validator states that static checks are not runtime acceptance",
      "does not claim" in read("tools/validate_phase_c.py") and
      "runtime" in read("tools/validate_phase_c.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE C STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-c-launch-c/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
