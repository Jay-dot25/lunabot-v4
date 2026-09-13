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
from std_msgs.msg import String


class ControlNode(Node):
    def __init__(self, input_topic, output_topic, max_linear, max_angular,
                 max_linear_accel, max_angular_accel, watchdog_sec, rate):
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
        self.last_input = 0.0
        self.last_tick = time.monotonic()
        self.last_status = 0.0
        self.input_count = 0
        self.output_count = 0

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/lunabot/control/status', 10)
        self.create_subscription(Twist, input_topic, self._input_cb, 10)
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

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        age = now - self.last_input if self.last_input else float('inf')
        watchdog = age > self.watchdog_sec
        target_v = 0.0 if watchdog else self.target_v
        target_w = 0.0 if watchdog else self.target_w
        self.output_v = self._approach(
            self.output_v, target_v, self.max_linear_accel, dt)
        self.output_w = self._approach(
            self.output_w, target_w, self.max_angular_accel, dt)

        out = Twist()
        out.linear.x = self.output_v
        out.angular.z = self.output_w
        self.cmd_pub.publish(out)
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
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
