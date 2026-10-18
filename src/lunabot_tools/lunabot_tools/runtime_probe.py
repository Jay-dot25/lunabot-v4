"""Live sensor/motion acceptance probe. Run against an already launched rover."""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__('lunabot_phase1_runtime_probe')
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_count = self.scan_count = self.odom_count = 0
        self.image_meta = None
        self.scan_meta = None
        self.odom = None
        self.obstacle = None
        self.status = None
        self.dynamic_frames = set()
        self.static_frames = set()
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/lunabot/obstacle_ahead', self.on_obstacle, 10)
        self.create_subscription(String, '/lunabot/obstacle_status', self.on_status, 10)
        self.create_subscription(TFMessage, '/tf', self.on_dynamic_tf, 10)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self.on_static_tf, static_qos)

    def on_dynamic_tf(self, msg):
        self.dynamic_frames.update((tf.header.frame_id, tf.child_frame_id)
                                   for tf in msg.transforms)

    def on_static_tf(self, msg):
        self.static_frames.update((tf.header.frame_id, tf.child_frame_id)
                                  for tf in msg.transforms)

    def on_image(self, msg):
        self.image_count += 1
        self.image_meta = {'width': msg.width, 'height': msg.height,
                           'encoding': msg.encoding, 'frame_id': msg.header.frame_id,
                           'data_bytes': len(msg.data)}

    def on_scan(self, msg):
        self.scan_count += 1
        finite = [r for r in msg.ranges if math.isfinite(r)
                  and msg.range_min <= r <= msg.range_max]
        self.scan_meta = {'frame_id': msg.header.frame_id, 'ranges': len(msg.ranges),
                          'finite_returns': len(finite),
                          'nearest': min(finite) if finite else None,
                          'range_min': msg.range_min, 'range_max': msg.range_max}

    def on_odom(self, msg):
        self.odom_count += 1
        self.odom = msg

    def on_obstacle(self, msg):
        self.obstacle = msg.data

    def on_status(self, msg):
        self.status = msg.data

    def spin_for(self, duration, linear=0.0, angular=0.0):
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                msg = Twist()
                msg.linear.x = linear
                msg.angular.z = angular
                self.command_pub.publish(msg)
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        self.command_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def spin_silent(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))

    def snapshot(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y, yaw_from_quaternion(self.odom.pose.pose.orientation))


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeProbe()
    result = {'status': 'FAIL', 'checks': {}, 'observations': {}}
    started = time.monotonic()
    try:
        node.spin_for(4.0)
        sensors_ready = node.image_count > 0 and node.scan_count > 0 and node.odom_count > 0
        result['checks']['live_camera_range_and_odometry'] = sensors_ready
        result['observations']['camera'] = node.image_meta
        result['observations']['scan_initial'] = node.scan_meta
        result['observations']['odom_frames'] = (
            {'frame_id': node.odom.header.frame_id,
             'child_frame_id': node.odom.child_frame_id} if node.odom else None)
        if not sensors_ready:
            raise RuntimeError('Required sensor/odometry messages were not received')

        image_ok = bool(node.image_meta and node.image_meta['width'] >= 320 and
                        node.image_meta['height'] >= 240 and node.image_meta['data_bytes'] > 0 and
                        node.image_meta['encoding'] in ('rgb8', 'bgr8'))
        scan_ok = bool(node.scan_meta and node.scan_meta['ranges'] >= 100 and
                       node.scan_meta['frame_id'] == 'lidar_link' and
                       node.scan_meta['finite_returns'] > 0)
        result['checks']['camera_frame'] = bool(
            node.image_meta and node.image_meta['frame_id'] == 'camera_link')
        odom_ok = (node.odom.header.frame_id == 'odom' and
                   node.odom.child_frame_id == 'base_link')
        tf_ok = (('odom', 'base_link') in node.dynamic_frames and
                 ('base_link', 'camera_link') in node.static_frames and
                 ('base_link', 'lidar_link') in node.static_frames)
        result['observations']['tf_dynamic'] = sorted(node.dynamic_frames)
        result['observations']['tf_static'] = sorted(node.static_frames)
        result['checks'].update(camera_payload=image_ok, range_frame=scan_ok,
                                odometry_frames=odom_ok, coordinate_frames=tf_ok)

        # Confirm true drive/odometry motion on each axis with low bounded speed.
        before = node.snapshot()
        node.spin_for(1.0, linear=0.15)
        forward = node.snapshot()
        forward_ok = forward and before and forward[0] - before[0] > 0.02
        result['checks']['forward_motion'] = bool(forward_ok)
        before_reverse = node.snapshot()
        node.spin_for(1.0, linear=-0.15)
        reverse = node.snapshot()
        reverse_ok = reverse and before_reverse and reverse[0] - before_reverse[0] < -0.02
        result['checks']['reverse_motion'] = bool(reverse_ok)

        before_left = node.snapshot()
        node.spin_for(1.0, angular=0.45)
        left = node.snapshot()
        left_ok = left and before_left and angle_delta(left[2], before_left[2]) > 0.08
        result['checks']['left_rotation'] = bool(left_ok)
        before_right = node.snapshot()
        node.spin_for(1.0, angular=-0.45)
        right = node.snapshot()
        right_ok = right and before_right and angle_delta(right[2], before_right[2]) < -0.08
        result['checks']['right_rotation'] = bool(right_ok)

        # Approach the predefined rock slowly until the real scan monitor asserts.
        node.spin_for(8.0, linear=0.15)
        saw_obstacle = node.obstacle is True
        node.spin_for(0.5)
        result['checks']['sensor_obstacle_detected'] = saw_obstacle
        result['observations']['obstacle_status_near'] = node.status
        # Retreat to restore a clear, repeatable initial test position.
        node.spin_for(8.0, linear=-0.15)
        node.spin_for(0.5)
        result['checks']['sensor_obstacle_cleared'] = node.obstacle is False
        result['observations']['obstacle_status_after_retreat'] = node.status

        # Publish one nonzero command, then deliberately stop renewing it; the
        # gate must time out. Measure drift only after that timeout interval.
        one_shot = Twist()
        one_shot.linear.x = 0.15
        node.command_pub.publish(one_shot)
        node.spin_silent(1.0)
        stationary_before = node.snapshot()
        node.spin_silent(1.0)
        stationary_after = node.snapshot()
        drift = (math.hypot(stationary_after[0] - stationary_before[0],
                            stationary_after[1] - stationary_before[1])
                 if stationary_before and stationary_after else float('inf'))
        result['observations']['stationary_odom_drift_m'] = drift
        result['checks']['stale_command_watchdog'] = drift < 0.15
        result['checks']['stops_without_stale_motion'] = drift < 0.15
        elapsed = max(time.monotonic() - started, 0.01)
        rates = {
            'camera': node.image_count / elapsed,
            'scan': node.scan_count / elapsed,
            'odom': node.odom_count / elapsed}
        result['observations']['rates_hz'] = rates
        result['checks']['camera_rate'] = rates['camera'] >= 5.0
        result['checks']['scan_rate'] = rates['scan'] >= 4.0
        result['checks']['odometry_rate'] = rates['odom'] >= 10.0
        result['observations']['message_counts'] = {
            'camera': node.image_count, 'scan': node.scan_count, 'odom': node.odom_count}
        result['observations']['duration_sec'] = elapsed
        passed = all(result['checks'].values())
        result['status'] = 'PASS' if passed else 'FAIL'
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        node.command_pub.publish(Twist())
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.03)
            node.command_pub.publish(Twist())
        print(json.dumps(result, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result['status'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
