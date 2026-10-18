"""Sensor-driven forward obstacle status node."""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .obstacle_logic import evaluate_forward_scan


class ObstacleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_obstacle_monitor")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacle_ahead")
        self.declare_parameter("status_topic", "/lunabot/obstacle_status")
        self.declare_parameter("threshold", 1.5)
        self.declare_parameter("forward_half_angle_deg", 15.0)
        self.declare_parameter("stale_timeout_sec", 0.75)
        self.declare_parameter("status_period_sec", 0.2)
        self.threshold = float(self.get_parameter("threshold").value)
        half_angle_deg = float(self.get_parameter("forward_half_angle_deg").value)
        self.half_angle = math.radians(half_angle_deg)
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        if self.threshold <= 0 or not 0 < half_angle_deg <= 90 or self.stale_timeout <= 0:
            raise ValueError("threshold/stale timeout must be positive; forward angle must be in (0, 90]")

        self.state = "STALE"
        self.nearest = None
        self.last_message_wall_time = None
        self.last_logged_state = None
        self.obstacle_pub = self.create_publisher(
            Bool, str(self.get_parameter("obstacle_topic").value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.on_scan, 10)
        self.timer = self.create_timer(
            float(self.get_parameter("status_period_sec").value), self.publish_status)
        self.get_logger().info(
            f"Monitoring {self.get_parameter('scan_topic').value}; "
            f"front +/-{half_angle_deg:.1f} deg, threshold {self.threshold:.2f} m")

    def on_scan(self, scan: LaserScan) -> None:
        self.last_message_wall_time = time.monotonic()
        self.state, self.nearest = evaluate_forward_scan(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, self.threshold, self.half_angle)

    def publish_status(self) -> None:
        if (self.last_message_wall_time is None or
                time.monotonic() - self.last_message_wall_time > self.stale_timeout):
            self.state = "STALE"
            self.nearest = None
        status = String()
        status.data = self.state if self.nearest is None else f"{self.state}; nearest={self.nearest:.3f} m"
        self.status_pub.publish(status)
        # A Bool is emitted only when a valid scan exists. No message means
        # unavailable/stale; false therefore unambiguously means observed clear.
        if self.state in ("CLEAR", "OBSTACLE"):
            obstacle = Bool()
            obstacle.data = self.state == "OBSTACLE"
            self.obstacle_pub.publish(obstacle)
        if self.state != self.last_logged_state:
            self.get_logger().info(f"Obstacle monitor: {status.data}")
            self.last_logged_state = self.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
