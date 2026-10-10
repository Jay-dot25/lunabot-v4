"""Clamp /cmd_vel and stop forwarding it after a wall-clock deadman timeout."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from lunabot_phase1_tools.command_guard_core import resolve_command


class CommandGuard(Node):
    """Single-owner safety boundary between ROS teleop and Gazebo DiffDrive."""

    def __init__(self) -> None:
        super().__init__("command_guard")
        self.declare_parameter("linear_velocity_limit_mps", 0.35)
        self.declare_parameter("angular_velocity_limit_radps", 0.80)
        self.declare_parameter("command_timeout_sec", 0.50)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_sim")

        self._linear_limit = float(self.get_parameter("linear_velocity_limit_mps").value)
        self._angular_limit = float(self.get_parameter("angular_velocity_limit_radps").value)
        self._timeout = float(self.get_parameter("command_timeout_sec").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        if self._rate <= 0.0 or not math.isfinite(self._rate):
            raise ValueError("publish_rate_hz must be finite and positive")
        if not math.isfinite(self._linear_limit) or self._linear_limit <= 0.0:
            raise ValueError("linear_velocity_limit_mps must be finite and positive")
        if not math.isfinite(self._angular_limit) or self._angular_limit <= 0.0:
            raise ValueError("angular_velocity_limit_radps must be finite and positive")
        if not math.isfinite(self._timeout) or self._timeout <= 0.0:
            raise ValueError("command_timeout_sec must be finite and positive")

        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_received_at: float | None = None
        self._reported_stale = False

        self._publisher = self.create_publisher(Twist, self._output_topic, 10)
        self._subscription = self.create_subscription(
            Twist, self._input_topic, self._on_command, 10
        )
        self._timer = self.create_timer(1.0 / self._rate, self._publish_guarded)
        self.get_logger().info(
            f"Guarding {self._input_topic} -> {self._output_topic}; "
            f"limits +/-{self._linear_limit:.2f} m/s, "
            f"+/-{self._angular_limit:.2f} rad/s, timeout {self._timeout:.2f} s"
        )

    def _on_command(self, message: Twist) -> None:
        self._last_linear = float(message.linear.x)
        self._last_angular = float(message.angular.z)
        self._last_received_at = time.monotonic()

    def _publish_guarded(self) -> None:
        decision = resolve_command(
            self._last_linear,
            self._last_angular,
            self._last_received_at,
            time.monotonic(),
            self._timeout,
            self._linear_limit,
            self._angular_limit,
        )
        output = Twist()
        output.linear.x = decision.linear_x
        output.angular.z = decision.angular_z
        self._publisher.publish(output)

        if decision.stale and not self._reported_stale:
            self.get_logger().info("No fresh velocity command; publishing a safe stop")
        self._reported_stale = decision.stale

    def publish_stop_burst(self) -> None:
        """Best-effort stop on orderly shutdown; watchdog covers lost publishers."""
        stop = Twist()
        for _ in range(3):
            self._publisher.publish(stop)
            time.sleep(0.03)


def main(args: list[str] | None = None) -> None:
    # Leave SIGINT as Python's KeyboardInterrupt so finally can publish the
    # best-effort zero burst before shutting down the ROS context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: CommandGuard | None = None
    try:
        node = CommandGuard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                try:
                    node.publish_stop_burst()
                except Exception as exc:
                    node.get_logger().warning(
                        f"Could not publish the shutdown stop burst: {exc}"
                    )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
