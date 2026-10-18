"""Bounded /cmd_vel relay with a stale-command watchdog."""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .obstacle_logic import clamp_velocity


class CommandGate(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_command_gate")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/lunabot/gz_cmd_vel")
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("timeout_sec", 0.30)
        self.declare_parameter("publish_rate", 20.0)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        rate = float(self.get_parameter("publish_rate").value)
        if min(self.max_linear, self.max_angular, self.timeout, rate) <= 0:
            raise ValueError("command limits, timeout, and publish rate must be positive")
        self.last_command = (0.0, 0.0)
        self.last_command_time = None
        self.was_stale = True
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.subscription = self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self.on_command, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(
            f"Bounded drive relay {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}; timeout={self.timeout:.2f}s")

    def on_command(self, command: Twist) -> None:
        self.last_command = clamp_velocity(
            command.linear.x, command.angular.z, self.max_linear, self.max_angular)
        self.last_command_time = time.monotonic()
        self.was_stale = False

    def publish_command(self) -> None:
        stale = (self.last_command_time is None or
                 time.monotonic() - self.last_command_time > self.timeout)
        linear, angular = (0.0, 0.0) if stale else self.last_command
        output = Twist()
        output.linear.x = linear
        output.angular.z = angular
        self.publisher.publish(output)
        if stale and not self.was_stale:
            self.get_logger().info("No fresh /cmd_vel received; sending stop")
        self.was_stale = stale


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A launch-managed SIGINT may shut the rclpy context down before this
        # finally block runs. Do not publish through an invalid context then.
        if rclpy.ok():
            stop = Twist()
            for _ in range(3):
                try:
                    node.publisher.publish(stop)
                except Exception:
                    break
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
