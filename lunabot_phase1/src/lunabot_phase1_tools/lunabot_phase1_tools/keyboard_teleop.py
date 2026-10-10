"""Small terminal keyboard driver for Phase 1; not an autonomous controller."""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.exceptions import RCLError
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")
        self.declare_parameter("linear_speed_mps", 0.25)
        self.declare_parameter("angular_speed_radps", 0.60)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self._linear_speed = abs(float(self.get_parameter("linear_speed_mps").value))
        self._angular_speed = abs(float(self.get_parameter("angular_speed_radps").value))
        topic = str(self.get_parameter("cmd_vel_topic").value)
        self._publisher = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f"Publishing manual velocity commands on {topic}")

    def publish(self, linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self._publisher.publish(message)


def _command_for_key(key: str, linear_speed: float, angular_speed: float) -> tuple[float, float] | None:
    """Return a planar command for a movement/stop key, otherwise None."""
    commands = {
        "w": (linear_speed, 0.0),
        "s": (-linear_speed, 0.0),
        "a": (0.0, angular_speed),
        "d": (0.0, -angular_speed),
        "x": (0.0, 0.0),
        " ": (0.0, 0.0),
    }
    return commands.get(key.lower())


def main(args: list[str] | None = None) -> int:
    if not sys.stdin.isatty():
        print("keyboard_teleop needs an interactive terminal (TTY).", file=sys.stderr)
        return 2

    # Preserve Python's Ctrl+C handler so the explicit zero burst is sent
    # before the ROS context is shut down in finally.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: KeyboardTeleop | None = None
    original_settings = termios.tcgetattr(sys.stdin)
    linear = angular = 0.0
    should_exit = False
    try:
        node = KeyboardTeleop()
        tty.setraw(sys.stdin.fileno())
        print(
            "LunaBot manual control: W forward, S reverse, A/D rotate, "
            "Space or X stop, Q quit. Commands persist until changed; "
            "the command guard stops after 0.5 s if this process disappears.",
            flush=True,
        )
        period = 1.0 / 20.0
        while rclpy.ok() and not should_exit:
            started = time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                key = sys.stdin.read(1)
                if key.lower() in ("q", "\x03"):
                    should_exit = True
                else:
                    command = _command_for_key(
                        key, node._linear_speed, node._angular_speed
                    )
                    if command is not None:
                        linear, angular = command
            if not should_exit:
                node.publish(linear, angular)
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        if node is not None:
            # A brief stop burst gives the command guard an explicit zero before exit.
            if rclpy.ok():
                try:
                    for _ in range(3):
                        node.publish(0.0, 0.0)
                        time.sleep(0.03)
                except RCLError as exc:
                    node.get_logger().warning(
                        f"Could not publish the teleop stop burst: {exc}"
                    )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Manual control stopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
