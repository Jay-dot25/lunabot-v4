#!/usr/bin/env python3
"""WASD keyboard teleoperation for LunaBot.

Publishes geometry_msgs/Twist on /cmd_vel.

Controls:
    w / s : drive forward / backward
    a / d : turn left / right
    space : stop
    + / - : increase / decrease speed
    q     : quit (rover is stopped on exit)

Safety: if no key is pressed for a short timeout the command decays to
zero, so the rover never keeps driving because a key event was lost.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

KEY_TIMEOUT = 0.6      # seconds without a key before commands decay to zero
PUBLISH_PERIOD = 0.05  # 20 Hz command stream


class WasdTeleop(Node):

    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5    # m/s
        self.angular_speed = 1.0   # rad/s
        self.speed_step = 0.1

    def publish(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


def get_keys(timeout):
    """Return any characters typed within `timeout` seconds."""
    keys = []
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rlist, _, _ = select.select([sys.stdin], [], [], remaining)
        if rlist:
            keys.append(sys.stdin.read(1))
        else:
            break
    return keys


def main():
    rclpy.init()
    node = WasdTeleop()

    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print('============================================================')
    print(' LunaBot WASD teleop')
    print('   w / s : forward / backward     a / d : turn left / right')
    print('   space : stop                   + / - : speed up / down')
    print('   q     : quit')
    print('============================================================')

    linear = 0.0
    angular = 0.0
    last_key_time = time.monotonic()

    try:
        while rclpy.ok():
            keys = get_keys(PUBLISH_PERIOD)
            now = time.monotonic()

            for key in keys:
                last_key_time = now
                if key == 'w':
                    linear = node.linear_speed
                    angular = 0.0
                elif key == 's':
                    linear = -node.linear_speed
                    angular = 0.0
                elif key == 'a':
                    linear = 0.0
                    angular = node.angular_speed
                elif key == 'd':
                    linear = 0.0
                    angular = -node.angular_speed
                elif key == ' ':
                    linear = 0.0
                    angular = 0.0
                elif key in ('+', '='):
                    node.linear_speed += node.speed_step
                    node.angular_speed += node.speed_step
                    print(f'--> speed: {node.linear_speed:.2f} m/s, '
                          f'{node.angular_speed:.2f} rad/s')
                elif key in ('-', '_'):
                    node.linear_speed = max(0.1, node.linear_speed - node.speed_step)
                    node.angular_speed = max(0.2, node.angular_speed - node.speed_step)
                    print(f'--> speed: {node.linear_speed:.2f} m/s, '
                          f'{node.angular_speed:.2f} rad/s')
                elif key in ('q', '\x03'):
                    raise KeyboardInterrupt

            # Safety decay: stop if no recent key activity.
            if now - last_key_time > KEY_TIMEOUT:
                linear = 0.0
                angular = 0.0

            node.publish(linear, angular)
            node.get_logger().debug(f'cmd_vel linear={linear} angular={angular}')

    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)   # guaranteed stop
        time.sleep(0.1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Teleop stopped; rover commanded to zero velocity.')


if __name__ == '__main__':
    main()
