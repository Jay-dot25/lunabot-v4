#!/usr/bin/env python3
"""
LunaBot V4 Phase B control layer.

INPUT
  /cmd_vel_in  geometry_msgs/msg/Twist from teleoperation or a future planner

PROCESS
  Clamp commands to the rover's configured limits, apply acceleration limits,
  and stop automatically when the input watchdog expires.

OUTPUT
  /cmd_vel             geometry_msgs/msg/Twist for the Gazebo DiffDrive plugin
  /lunabot/control/status  std_msgs/msg/String health and watchdog status

This node deliberately does not own the Gazebo bridge or the wheel model. It
is the reusable ROS-side control boundary introduced by Phase B.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


class ControlNode(Node):
    WHEELBASE_HALF = 0.62
    TRACK_HALF = 0.52
    MAX_STEER_RAD = 0.75

    def __init__(self, input_topic, output_topic, max_linear, max_angular,
                 max_linear_accel, max_angular_accel, watchdog_sec, rate,
                 enable_corner_steering=False):
        super().__init__('lunabot_control')
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.max_linear = abs(max_linear)
        self.max_angular = abs(max_angular)
        self.max_linear_accel = abs(max_linear_accel)
        self.max_angular_accel = abs(max_angular_accel)
        self.watchdog_sec = max(0.05, watchdog_sec)
        self.target_v = 0.0
        self.target_w = 0.0
        self.output_v = 0.0
        self.output_w = 0.0
        self.imu_wz = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_count = 0
        self.enable_corner_steering = bool(enable_corner_steering)
        self._last_steer = (0.0, 0.0, 0.0, 0.0)
        self.last_input = 0.0
        self.last_tick = time.monotonic()
        self.last_status = 0.0
        self.input_count = 0
        self.output_count = 0

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.steer_fl_pub = self.create_publisher(
            Float64, '/lunabot/steer/front_left', 10)
        self.steer_fr_pub = self.create_publisher(
            Float64, '/lunabot/steer/front_right', 10)
        self.steer_rl_pub = self.create_publisher(
            Float64, '/lunabot/steer/rear_left', 10)
        self.steer_rr_pub = self.create_publisher(
            Float64, '/lunabot/steer/rear_right', 10)
        self.status_pub = self.create_publisher(
            String, '/lunabot/control/status', 10)
        self.create_subscription(Twist, input_topic, self._input_cb, 10)
        self.create_subscription(
            Imu, '/lunabot/imu', self._imu_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'control active: {input_topic} -> {output_topic}; '
            f'watchdog={self.watchdog_sec:.2f}s, '
            f'limits={self.max_linear:.2f}m/s {self.max_angular:.2f}rad/s')

    @staticmethod
    def _clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    @staticmethod
    def _approach(current, target, rate, dt):
        step = max(0.0, rate) * max(0.0, min(dt, 0.25))
        if target > current:
            return min(target, current + step)
        return max(target, current - step)

    def _input_cb(self, msg):
        self.target_v = self._clamp(msg.linear.x, self.max_linear)
        self.target_w = self._clamp(msg.angular.z, self.max_angular)
        self.last_input = time.monotonic()
        self.input_count += 1

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.imu_pitch = math.asin(sinp)
        if math.isfinite(msg.angular_velocity.z):
            self.imu_wz = float(msg.angular_velocity.z)
        self.imu_count += 1

    @classmethod
    def _compute_steering_angles(cls, linear_v: float, angular_w: float):
        """Compute 4-wheel double-Ackermann or spot-turn steering angles."""
        if abs(linear_v) < 0.01 and abs(angular_w) < 0.02:
            return 0.0, 0.0, 0.0, 0.0
        if abs(linear_v) < 0.03 and abs(angular_w) >= 0.02:
            spot = min(cls.MAX_STEER_RAD,
                       math.atan2(cls.WHEELBASE_HALF, cls.TRACK_HALF))
            return -spot, spot, spot, -spot
        if abs(angular_w) < 1e-3:
            return 0.0, 0.0, 0.0, 0.0
        radius = linear_v / angular_w
        r_left = radius - cls.TRACK_HALF
        r_right = radius + cls.TRACK_HALF
        if abs(r_left) < 0.05:
            r_left = 0.05 if r_left >= 0.0 else -0.05
        if abs(r_right) < 0.05:
            r_right = 0.05 if r_right >= 0.0 else -0.05
        fl = cls._clamp(math.atan(cls.WHEELBASE_HALF / r_left), cls.MAX_STEER_RAD)
        fr = cls._clamp(math.atan(cls.WHEELBASE_HALF / r_right), cls.MAX_STEER_RAD)
        return fl, fr, -fl, -fr

    def _publish_steering(self, fl: float, fr: float, rl: float, rr: float):
        for pub, angle in ((self.steer_fl_pub, fl), (self.steer_fr_pub, fr),
                           (self.steer_rl_pub, rl), (self.steer_rr_pub, rr)):
            msg = Float64()
            msg.data = float(angle)
            pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        age = now - self.last_input if self.last_input else float('inf')
        watchdog = age > self.watchdog_sec
        tilt_mag = max(abs(self.imu_roll), abs(self.imu_pitch))
        slope_scale = 0.65 if tilt_mag > 0.35 else 1.0
        target_v = 0.0 if watchdog else self.target_v * slope_scale
        target_w = 0.0 if watchdog else self.target_w
        self.output_v = self._approach(
            self.output_v, target_v, self.max_linear_accel, dt)
        self.output_w = self._approach(
            self.output_w, target_w, self.max_angular_accel, dt)

        out = Twist()
        out.linear.x = self.output_v
        out.angular.z = self.output_w
        self.cmd_pub.publish(out)
        if self.enable_corner_steering:
            fl, fr, rl, rr = self._compute_steering_angles(self.output_v, self.output_w)
            scaled = (0.20 * fl, 0.20 * fr, 0.20 * rl, 0.20 * rr)
            if any(abs(a - b) > 1e-3 for a, b in zip(scaled, self._last_steer)):
                self._publish_steering(*scaled)
                self._last_steer = scaled
        self.output_count += 1

        if now - self.last_status >= 1.0:
            status = String()
            state = 'WATCHDOG_STOP' if watchdog else 'ACTIVE'
            status.data = (
                f'{state} input={self.input_topic} '
                f'age={age:.3f}s target=({target_v:.3f},{target_w:.3f}) '
                f'output=({self.output_v:.3f},{self.output_w:.3f}) '
                f'clamps=({self.max_linear:.3f},{self.max_angular:.3f}) '
                f'counts=({self.input_count},{self.output_count})')
            self.status_pub.publish(status)
            self.last_status = now


def main():
    parser = argparse.ArgumentParser(description='LunaBot Phase B control node')
    parser.add_argument('--input-topic', default='/cmd_vel_in')
    parser.add_argument('--output-topic', default='/cmd_vel')
    parser.add_argument('--max-linear', type=float, default=0.45)
    parser.add_argument('--max-angular', type=float, default=1.0)
    parser.add_argument('--max-linear-accel', type=float, default=0.4)
    parser.add_argument('--max-angular-accel', type=float, default=0.8)
    parser.add_argument('--watchdog-sec', type=float, default=0.5)
    parser.add_argument('--rate', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ControlNode(
        args.input_topic, args.output_topic, args.max_linear, args.max_angular,
        args.max_linear_accel, args.max_angular_accel, args.watchdog_sec,
        args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # rclpy installs a SIGTERM handler which may invalidate the context
        # before this finally block runs. Only publish the final zero command
        # while the publisher context is still valid.
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
