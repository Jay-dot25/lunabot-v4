"""Report the nearest valid LiDAR return in a configurable forward cone."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from lunabot_phase1_tools.obstacle_monitor_core import (
    CLEAR,
    ObstacleObservation,
    NO_VALID_MEASUREMENTS,
    OBSTACLE_DETECTED,
    classify_obstacle,
    closest_forward_range,
    scan_is_stale,
)


class ObstacleMonitor(Node):
    """Diagnostics-only consumer; it never publishes velocity commands."""

    def __init__(self) -> None:
        super().__init__("obstacle_monitor")
        self.declare_parameter("warning_distance_m", 1.50)
        self.declare_parameter("forward_half_angle_rad", 0.5235987756)
        self.declare_parameter("scan_timeout_sec", 1.00)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("status_topic", "/obstacle_monitor/status")
        self.declare_parameter("range_topic", "/obstacle_monitor/closest_range")

        self._warning_distance = float(self.get_parameter("warning_distance_m").value)
        self._forward_half_angle = float(
            self.get_parameter("forward_half_angle_rad").value
        )
        self._scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        range_topic = str(self.get_parameter("range_topic").value)
        if not math.isfinite(self._warning_distance) or self._warning_distance <= 0.0:
            raise ValueError("warning_distance_m must be finite and positive")
        if (
            not math.isfinite(self._forward_half_angle)
            or not 0.0 <= self._forward_half_angle <= math.pi
        ):
            raise ValueError("forward_half_angle_rad must be in [0, pi]")
        if not math.isfinite(self._scan_timeout) or self._scan_timeout <= 0.0:
            raise ValueError("scan_timeout_sec must be finite and positive")

        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self._range_publisher = self.create_publisher(Float32, range_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._last_status: str | None = None
        self._last_scan_monotonic: float | None = None
        # If the sensor stops publishing entirely, report unknown rather than
        # leaving a stale CLEAR status on the diagnostic topic.
        poll_period = min(0.25, self._scan_timeout / 2.0)
        self._watchdog = self.create_timer(poll_period, self._check_scan_timeout)
        self.get_logger().info(
            f"Monitoring {self._scan_topic} within +/-{self._forward_half_angle:.2f} rad; "
            f"warning distance {self._warning_distance:.2f} m, "
            f"scan timeout {self._scan_timeout:.2f} s"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        self._last_scan_monotonic = time.monotonic()
        nearest = closest_forward_range(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._forward_half_angle,
        )
        self._publish_observation(classify_obstacle(nearest, self._warning_distance))

    def _check_scan_timeout(self) -> None:
        if not scan_is_stale(
            self._last_scan_monotonic,
            time.monotonic(),
            self._scan_timeout,
        ):
            return
        if self._last_status != NO_VALID_MEASUREMENTS:
            self._publish_observation(
                classify_obstacle(None, self._warning_distance)
            )

    def _publish_observation(self, observation: ObstacleObservation) -> None:
        status_message = String()
        status_message.data = observation.status
        self._status_publisher.publish(status_message)

        range_message = Float32()
        range_message.data = (
            float(observation.closest_range_m)
            if observation.closest_range_m is not None
            else float("nan")
        )
        self._range_publisher.publish(range_message)

        if observation.status != self._last_status:
            if observation.status == OBSTACLE_DETECTED:
                self.get_logger().warning(
                    f"Forward obstacle at {observation.closest_range_m:.2f} m"
                )
            elif observation.status == CLEAR:
                self.get_logger().info(
                    f"Forward sector clear; nearest valid return "
                    f"{observation.closest_range_m:.2f} m"
                )
            elif observation.status == NO_VALID_MEASUREMENTS:
                self.get_logger().warning(
                    "No valid LiDAR returns in the configured forward sector "
                    "or no scan received before timeout"
                )
            self._last_status = observation.status


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ObstacleMonitor | None = None
    try:
        node = ObstacleMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
