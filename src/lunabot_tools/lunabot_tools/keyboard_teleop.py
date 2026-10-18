"""Minimal terminal WASD teleop; press/repeat keys to renew bounded motion."""
from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('lunabot_keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, linear=0.0, angular=0.0):
        if not rclpy.ok():
            return
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('Keyboard teleop requires an interactive terminal (TTY).')
    rclpy.init(args=args)
    node = KeyboardTeleop()
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    print('W/S: forward/reverse, A/D: rotate left/right, SPACE: stop, Q: quit')
    print('Commands are renewed while the key is repeated; the drive watchdog stops stale input.')
    mapping = {'w': (0.25, 0.0), 's': (-0.25, 0.0),
               'a': (0.0, 0.55), 'd': (0.0, -0.55)}
    active = (0.0, 0.0)
    last_key_time = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == 'q' or key == '\x03':
                    break
                if key == ' ':
                    active = (0.0, 0.0)
                    node.send()
                    print('STOP')
                elif key in mapping:
                    active = mapping[key]
                    last_key_time = time.monotonic()
                    print(f'cmd_vel linear.x={active[0]:.2f}, angular.z={active[1]:.2f}')
            if active != (0.0, 0.0):
                if time.monotonic() - last_key_time <= 0.30:
                    node.send(*active)
                else:
                    active = (0.0, 0.0)
                    node.send()
    except KeyboardInterrupt:
        pass
    finally:
        node.send()
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.02)
            node.send()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
