#!/usr/bin/env python3
"""Static Phase B gate: Control & Odometry.

This validator never claims Gazebo runtime success. It checks that the
independent launch, ROS control boundary, odometry monitor, docs and Phase A
baseline are internally consistent. Runtime evidence must come from the user's
ROS 2/Gazebo workstation via evidence/phase-b-launch-b/collect_evidence.sh.
"""

from pathlib import Path
import ast
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []

def check(name, ok, detail=''):
    checks.append((name, bool(ok), detail))


def text(path):
    return path.read_text(encoding='utf-8')


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)

# Required files and permissions.
required = [
    'launch-a', 'launch-b', 'scripts/launch-a.sh', 'scripts/launch-b.sh',
    'scripts/wasd_teleop.py', 'scripts/control_odometry.py',
    'scripts/odometry_monitor.py', 'rviz/phase_a.rviz', 'rviz/phase_b.rviz',
    'src/lunabot_gazebo/worlds/lunar_world.sdf',
    'src/lunabot_gazebo/models/lunabot_v4/model.sdf',
    'tools/validate_phase_a.py', 'evidence/phase-a-launch-a/README.md',
    'evidence/phase-b-launch-b/README.md',
    'evidence/phase-b-launch-b/collect_evidence.sh',
]
for rel in required:
    p = ROOT / rel
    check(f'file exists: {rel}', p.is_file())
for rel in ['launch-b', 'scripts/launch-b.sh', 'scripts/control_odometry.py',
            'scripts/odometry_monitor.py', 'evidence/phase-b-launch-b/collect_evidence.sh']:
    check(f'executable: {rel}', (ROOT / rel).stat().st_mode & 0o111)

# XML baseline.
for rel, kind in [('src/lunabot_gazebo/worlds/lunar_world.sdf', 'world'),
                  ('src/lunabot_gazebo/models/lunabot_v4/model.sdf', 'model')]:
    try:
        root = ET.parse(ROOT / rel).getroot()
        check(f'{kind} SDF well-formed: {rel}', True)
    except Exception as exc:
        check(f'{kind} SDF well-formed: {rel}', False, str(exc))

# Python syntax and imports represented structurally (no ROS import in sandbox).
for rel in ['scripts/wasd_teleop.py', 'scripts/control_odometry.py',
            'scripts/odometry_monitor.py', 'tools/validate_phase_a.py',
            'tools/plot_lidar_scan.py']:
    try:
        ast.parse(text(ROOT / rel))
        check(f'Python syntax: {rel}', True)
    except Exception as exc:
        check(f'Python syntax: {rel}', False, str(exc))

for rel in ['scripts/launch-a.sh', 'scripts/launch-b.sh', 'launch-a', 'launch-b']:
    r = run(['bash', '-n', rel])
    check(f'bash syntax: {rel}', r.returncode == 0, r.stderr.strip())

# Phase B controller contract.
control = text(ROOT / 'scripts/control_odometry.py')
check('control node class exists', 'class ControlNode(Node)' in control)
check('control input defaults to /cmd_vel_in', "default='/cmd_vel_in'" in control)
check('control output defaults to /cmd_vel', "default='/cmd_vel'" in control)
check('control clamps linear and angular values',
      '_clamp(msg.linear.x' in control and '_clamp(msg.angular.z' in control)
check('control applies acceleration limits',
      'max_linear_accel' in control and 'max_angular_accel' in control and
      '_approach(' in control)
check('control watchdog stops stale input',
      'WATCHDOG_STOP' in control and 'watchdog_sec' in control)
check('control status topic', '/lunabot/control/status' in control)
check('control publishes Twist output', 'self.cmd_pub.publish(out)' in control)
check('control guards shutdown publish',
      'if rclpy.ok()' in control and 'publisher context is still valid' in control)

# Odometry monitor contract.
monitor = text(ROOT / 'scripts/odometry_monitor.py')
check('monitor subscribes to DiffDrive odometry',
      "'/lunabot/odom'" in monitor and 'Odometry' in monitor)
check('monitor imports Gazebo-compatible QoS',
      'qos_profile_sensor_data' in monitor and
      'ExternalShutdownException' in monitor)
check('monitor publishes quality status', '/lunabot/odometry/status' in monitor)
check('monitor records CSV samples', 'odometry_samples.csv' in monitor and
      'csv.writer' in monitor)
check('monitor writes report', 'odometry_report.txt' in monitor and
      'quality_result' in monitor)
check('monitor checks odom and chassis frames',
      "'odom' in self.frame_ids" in monitor and
      "'chassis' in self.child_frame_ids" in monitor)
check('monitor checks continuity', 'max_step' in monitor and 'self.max_step < 0.5' in monitor)

# Teleop remains Phase A compatible and supports B input topic.
teleop = text(ROOT / 'scripts/wasd_teleop.py')
check('teleop default preserves Phase A /cmd_vel', "topic='/cmd_vel'" in teleop)
check('teleop supports --topic override', 'ap.add_argument("--topic"' in teleop)
check('teleop publishes selected topic', 'create_publisher(Twist, topic' in teleop)
check('teleop demo supports sim time', 'Clock' in teleop and 'node.sim_t' in teleop)
check('teleop writes failure evidence',
      'failure_reason' in teleop and 'demo_drive_result.txt' in teleop)
check('teleop uses Gazebo-compatible QoS',
      'qos_profile_sensor_data' in teleop and
      'ExternalShutdownException' not in teleop)

# Independent launch contract.
launch = text(ROOT / 'scripts/launch-b.sh')
check('launch-b has Phase B banner', 'PHASE B' in launch and 'CONTROL & ODOMETRY' in launch)
launch_commands = '\n'.join(
    line for line in launch.splitlines() if not line.lstrip().startswith('#'))
check('launch-b does not invoke launch-a',
      not re.search(r'(^|\s)(bash\s+)?[^#\n]*launch-a', launch_commands))
check('launch-b uses Phase B evidence directory', 'phase-b-launch-b' in launch)
check('launch-b starts controller', 'control_odometry.py' in launch and 'CONTROL_PID' in launch)
check('launch-b starts odometry monitor', 'odometry_monitor.py' in launch and 'ODOM_PID' in launch)
check('launch-b separates control input/output',
      '--input-topic /cmd_vel_in' in launch and '--output-topic /cmd_vel' in launch)
check('launch-b explicitly unpauses headless Gazebo',
      'WorldControl' in launch and "--req 'pause: false'" in launch)
check('launch-b uses real topic-message validation',
      'sample="$(timeout 30 ros2 topic echo' in launch and
      '| head -n 3' not in launch)
check('launch-b cleans stale Phase B nodes',
      'ros_ign_bridge.*parameter_bridge' in launch and
      'scripts/control_odometry.py' in launch)
check('launch-b bridges simulation clock', '"/clock@rosgraph_msgs/msg/Clock' in launch)
check('launch-b retries dynamic TF validation',
      'for _ in 1 2 3' in launch and 'tf2_echo' in launch)
check('launch-b has watchdog status runtime check',
      '/lunabot/control/status' in launch)
check('launch-b has odometry status runtime check',
      '/lunabot/odometry/status' in launch)
check('launch-b has clean shutdown',
      'kill -TERM "$CONTROL_PID"' in launch and
      'kill -TERM "$ODOM_PID"' in launch and 'shutdown()' in launch)
check('launch-b has demo nonzero exit propagation',
      'shutdown "$EXIT_CODE"' in launch and 'demo_rc' in launch)

# RViz and evidence/docs.
rviz_b = text(ROOT / 'rviz/phase_b.rviz')
check('Phase B RViz fixed frame odom', 'Fixed Frame: odom' in rviz_b)
check('Phase B RViz displays odometry',
      'rviz_default_plugins/Odometry' in rviz_b and '/lunabot/odom' in rviz_b)
for rel in ['docs/phase-b-launch-b.md']:
    p = ROOT / rel
    check(f'document exists: {rel}', p.is_file())
    if p.is_file():
        d = text(p)
        for i in range(1, 24):
            check(f'doc {rel} section {i}',
                  re.search(rf'^## {i}\. ', d, re.MULTILINE) is not None)

# Evidence collector points at the independent Phase B launch.
collector = text(ROOT / 'evidence/phase-b-launch-b/collect_evidence.sh')
check('collector launches Phase B directly',
      'scripts/launch-b.sh' in collector and 'DEMO=1' in collector)

# Baseline static validator remains present; run it when dependencies are available.
r = run([sys.executable, 'tools/validate_phase_a.py'])
check('Phase A static baseline remains 76/76',
      r.returncode == 0 and '76/76' in r.stdout,
      r.stdout[-200:].strip())

passed = sum(ok for _, ok, _ in checks)
print('=' * 60)
print('LUNABOT V4 PHASE B STATIC VALIDATION')
print('=' * 60)
for name, ok, detail in checks:
    suffix = f' - {detail}' if detail else ''
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{suffix}")
print('=' * 60)
print(f'RESULT: {passed}/{len(checks)} checks passed - ' +
      ('ALL PASS' if passed == len(checks) else 'FAIL'))
print('=' * 60)
report = ROOT / 'evidence/phase-b-launch-b/static_validation.txt'
report.write_text('\n'.join(
    f"[{'PASS' if ok else 'FAIL'}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding='utf-8')
print(f'report: {report}')
sys.exit(0 if passed == len(checks) else 1)
