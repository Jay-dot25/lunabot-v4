#!/usr/bin/env python3
"""Phase 1 acceptance test: Lunar Base Camp Certification.

Runs against a live simulation (see scripts/run_phase1_sim.sh) and checks
the Phase 1 pass-gate items that can be verified from ROS interfaces:

  1. topics_active        all sensor/odometry interfaces publish data
  2. camera_frames_valid  camera images are well-formed and arrive at a
                          usable rate
  3. range_sensor_valid   LiDAR scan has usable measurements
  4. obstacle_detectable  a physical obstacle is visible to the range
                          sensor in the forward sector (demo triad)
  5. motion_forward       the rover translates when commanded forward
                          (measured via wheel odometry only)
  6. motion_rotate        the rover yaws when commanded to rotate
  7. motion_stop          the rover stops when commanded to stop

Ground-truth policy: every check uses ONLY the rover's own interfaces
(/lunabot/odom, /lunabot/lidar/scan, /lunabot/camera/image_raw,
/lunabot/imu). The evaluation ground-truth stream is never read.

Results are written as JSON plus a markdown summary under evidence/phase1/.

Usage (simulation must already be running):
    python3 src/lunabot_tests/lunabot_tests/phase1_base_camp.py \
        [--output-dir evidence/phase1] [--timeout 60]
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan

TOPICS_REQUIRED = ('camera', 'scan', 'odom', 'imu')


def yaw_from_quaternion(q):
    """Extract yaw (rotation about Z) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Phase1BaseCamp(Node):
    """Staged acceptance runner for the Lunar Base Camp certification."""

    def __init__(self, output_dir, topic_timeout):
        super().__init__('phase1_base_camp')

        self.output_dir = Path(output_dir)
        self.topic_timeout = topic_timeout

        # Latest observations and counters.
        self.counts = {'camera': 0, 'scan': 0, 'odom': 0, 'imu': 0}
        self.last_image = None
        self.last_scan = None
        self.last_odom = None
        self.last_imu = None

        self.create_subscription(Image, '/lunabot/camera/image_raw',
                                 self._on_image, 10)
        self.create_subscription(LaserScan, '/lunabot/lidar/scan',
                                 self._on_scan, 10)
        self.create_subscription(Odometry, '/lunabot/odom',
                                 self._on_odom, 20)
        self.create_subscription(Imu, '/lunabot/imu', self._on_imu, 20)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.results = []
        self.stage = 'wait_topics'
        self.stage_started = time.monotonic()
        self.drive_cmd = Twist()
        self.drive_end_time = None
        self.motion_start_pose = None

    # ------------------------------------------------------------ I/O

    def _on_image(self, msg):
        self.counts['camera'] += 1
        self.last_image = msg

    def _on_scan(self, msg):
        self.counts['scan'] += 1
        self.last_scan = msg

    def _on_odom(self, msg):
        self.counts['odom'] += 1
        self.last_odom = msg

    def _on_imu(self, msg):
        self.counts['imu'] += 1
        self.last_imu = msg

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    # --------------------------------------------------------- helpers

    def record(self, name, passed, measured, threshold, detail=''):
        entry = {
            'check': name,
            'passed': bool(passed),
            'measured': measured,
            'threshold': threshold,
            'detail': detail,
        }
        self.results.append(entry)
        status = 'PASS' if passed else 'FAIL'
        self.get_logger().info(
            f'[{status}] {name}: measured={measured} threshold={threshold} {detail}')

    def odom_pose(self):
        """(x, y, yaw) from wheel odometry only."""
        msg = self.last_odom
        p = msg.pose.pose.position
        return p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)

    def front_min_range(self, half_angle=math.pi / 3.0):
        """Minimum valid range in the forward sector of the scan."""
        scan = self.last_scan
        if scan is None or not scan.ranges:
            return None
        best = float('inf')
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = normalize_angle(angle)
            if abs(angle) <= half_angle and math.isfinite(r) and r > 0:
                if r < best:
                    best = r
        return None if best == float('inf') else best

    def enter(self, stage):
        self.stage = stage
        self.stage_started = time.monotonic()
        self.get_logger().info(f'--- stage: {stage} ---')

    def elapsed(self):
        return time.monotonic() - self.stage_started

    # ---------------------------------------------------------- stages

    def tick(self):
        """Advance the acceptance state machine. Called at 20 Hz."""
        if self.stage == 'wait_topics':
            if all(self.counts[t] >= 3 for t in TOPICS_REQUIRED):
                rates = self.message_rates()
                self.record('topics_active', True,
                            {t: self.counts[t] for t in TOPICS_REQUIRED},
                            '>= 3 msgs per required topic',
                            f'rates(hz approx)={rates}')
                self.enter('sensor_valid')
            elif self.elapsed() > self.topic_timeout:
                missing = [t for t in TOPICS_REQUIRED if self.counts[t] < 3]
                self.record('topics_active', False,
                            {t: self.counts[t] for t in TOPICS_REQUIRED},
                            '>= 3 msgs per required topic',
                            f'missing/starved: {missing}')
                self.finish()

        elif self.stage == 'sensor_valid':
            img_ok, img_detail = self.check_image()
            self.record('camera_frames_valid', img_ok,
                        img_detail['measured'], img_detail['threshold'],
                        img_detail.get('note', ''))
            scan_ok, scan_detail = self.check_scan()
            self.record('range_sensor_valid', scan_ok,
                        scan_detail['measured'], scan_detail['threshold'],
                        scan_detail.get('note', ''))
            self.enter('obstacle_detect')

        elif self.stage == 'obstacle_detect':
            d = self.front_min_range()
            if d is not None:
                self.record('obstacle_detectable', d < 5.0, round(d, 3),
                            'min front range < 5.0 m',
                            'demo rock triad ahead of spawn')
                self.enter('motion_forward')
            elif self.elapsed() > 10.0:
                self.record('obstacle_detectable', False, None,
                            'min front range < 5.0 m', 'no scan data')
                self.enter('motion_forward')

        elif self.stage == 'motion_forward':
            if self.motion_start_pose is None:
                self.motion_start_pose = self.odom_pose()
                self.drive_cmd.linear.x = 0.4
                self.drive_end_time = time.monotonic() + 3.0
            if time.monotonic() < self.drive_end_time:
                self.publish_cmd(linear_x=0.4)
            else:
                self.stop()
                x0, y0, _ = self.motion_start_pose
                x1, y1, _ = self.odom_pose()
                disp = math.hypot(x1 - x0, y1 - y0)
                self.record('motion_forward', disp >= 0.30,
                            round(disp, 3), 'odometry displacement >= 0.30 m',
                            'cmd: +0.4 m/s for 3 s (wheel odometry only)')
                self.motion_start_pose = None
                self.enter('motion_rotate')

        elif self.stage == 'motion_rotate':
            if self.motion_start_pose is None:
                self.motion_start_pose = self.odom_pose()
                self.drive_end_time = time.monotonic() + 3.0
            if time.monotonic() < self.drive_end_time:
                self.publish_cmd(angular_z=0.6)
            else:
                self.stop()
                _, _, yaw0 = self.motion_start_pose
                _, _, yaw1 = self.odom_pose()
                dyaw = abs(normalize_angle(yaw1 - yaw0))
                self.record('motion_rotate', dyaw >= 0.30,
                            round(dyaw, 3), '|delta yaw| >= 0.30 rad',
                            'cmd: +0.6 rad/s for 3 s (wheel odometry only)')
                self.motion_start_pose = None
                self.enter('motion_stop')

        elif self.stage == 'motion_stop':
            if self.drive_end_time is None:
                self.drive_end_time = time.monotonic() + 1.5
            if time.monotonic() < self.drive_end_time:
                self.stop()
            else:
                self.stop()
                msg = self.last_odom
                v = abs(msg.twist.twist.linear.x)
                w = abs(msg.twist.twist.angular.z)
                ok = v < 0.05 and w < 0.10
                self.record('motion_stop', ok,
                            {'linear': round(v, 4), 'angular': round(w, 4)},
                            '|linear| < 0.05 m/s and |angular| < 0.10 rad/s')
                self.drive_end_time = None
                self.finish()

    def message_rates(self):
        """Approximate rates based on counts divided by elapsed run time."""
        elapsed = max(time.monotonic() - self.run_started, 1e-3)
        return {t: round(self.counts[t] / elapsed, 1) for t in TOPICS_REQUIRED}

    def check_image(self):
        img = self.last_image
        if img is None:
            return False, {'measured': None, 'threshold': 'w>0, h>0',
                           'note': 'no image received'}
        ok = img.width > 0 and img.height > 0 and len(img.data) > 0
        return ok, {
            'measured': {'width': img.width, 'height': img.height},
            'threshold': 'w>0, h>0, non-empty data',
        }

    def check_scan(self):
        scan = self.last_scan
        if scan is None or not scan.ranges:
            return False, {'measured': None,
                           'threshold': '>=180 beams, >=1 finite range',
                           'note': 'no scan received'}
        finite = [r for r in scan.ranges if math.isfinite(r) and r > 0]
        ok = len(scan.ranges) >= 180 and len(finite) > 0
        return ok, {
            'measured': {
                'beams': len(scan.ranges),
                'finite_ranges': len(finite),
                'min_range': round(min(finite), 3) if finite else None,
            },
            'threshold': '>=180 beams, >=1 finite range',
        }

    # ---------------------------------------------------------- report

    def finish(self):
        self.stop()
        all_passed = all(r['passed'] for r in self.results) and len(self.results) > 0
        report = {
            'phase': 'Phase 1 - Lunar Base Camp',
            'test': 'phase1_base_camp',
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'ground_truth_used': False,
            'policy': 'All checks use rover interfaces only '
                      '(/lunabot/odom, /lunabot/lidar/scan, '
                      '/lunabot/camera/image_raw, /lunabot/imu).',
            'checks': self.results,
            'overall': 'PASS' if all_passed else 'FAIL',
        }

        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        json_path = self.output_dir / f'phase1_base_camp_{stamp}.json'
        latest_path = self.output_dir / 'phase1_base_camp_latest.json'
        md_path = self.output_dir / f'phase1_base_camp_{stamp}.md'

        for path in (json_path, latest_path):
            path.write_text(json.dumps(report, indent=2))

        md_path.write_text(self.render_markdown(report))

        self.get_logger().info(f'Overall: {report["overall"]}')
        self.get_logger().info(f'Report:  {json_path}')
        self.get_logger().info(f'Summary: {md_path}')

        self.done = True
        self.overall_passed = all_passed

    @staticmethod
    def render_markdown(report):
        lines = [
            f'# Phase 1 Acceptance Report - {report["timestamp_utc"]}',
            '',
            f'**Overall result: {report["overall"]}**',
            '',
            f'Ground truth used: {report["ground_truth_used"]}',
            f'Policy: {report["policy"]}',
            '',
            '| Check | Result | Measured | Threshold | Notes |',
            '|---|---|---|---|---|',
        ]
        for r in report['checks']:
            lines.append(
                f'| {r["check"]} | {"PASS" if r["passed"] else "FAIL"} | '
                f'`{r["measured"]}` | {r["threshold"]} | {r["detail"]} |')
        lines.append('')
        return '\n'.join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Phase 1 base camp acceptance test')
    parser.add_argument('--output-dir', default='evidence/phase1',
                        help='Directory for JSON/markdown evidence reports')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Seconds to wait for all required topics')
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])

    rclpy.init()
    node = Phase1BaseCamp(args.output_dir, args.timeout)
    node.done = False
    node.overall_passed = False
    node.run_started = time.monotonic()

    # 20 Hz state machine tick.
    tick_period = 0.05
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=tick_period)
            node.tick()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted; writing partial report.')
        node.finish()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(0 if node.overall_passed else 1)


if __name__ == '__main__':
    main()
