#!/usr/bin/env python3
"""
LunaBot V4 - Phase A - WASD teleoperation + automated demo drive test
=======================================================================

Interactive mode (default):
    W forward | S reverse | A turn left | D turn right | Space stop | Q quit
    Publishes geometry_msgs/msg/Twist on /cmd_vel.

Demo mode (headless / automated validation):
    python3 wasd_teleop.py --demo [evidence_dir]
    Sequence: forward 3 s -> stop -> turn left 3 s -> stop
    Reports the odometry delta (distance driven, yaw change) and writes
    <evidence_dir>/demo_drive_result.txt. Exit code 0 = movement verified.

Speeds respect the DiffDrive plugin limits in model.sdf
(max_linear_velocity 0.45 m/s, max_angular_velocity 1.0 rad/s).
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

MAX_LINEAR = 0.45   # m/s  (DiffDrive plugin limit in model.sdf)
MAX_ANGULAR = 1.0   # rad/s


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/lunabot/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.odom = msg

    def snapshot(self):
        """Return (x, y, yaw, t) from the latest odometry, or None."""
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return (p.x, p.y, yaw_from_quaternion(q), time.time())


def drive(node, lin, ang, duration):
    msg = Twist()
    msg.linear.x = lin
    msg.angular.z = ang
    t0 = time.time()
    while time.time() - t0 < duration:
        node.pub.publish(msg)
        time.sleep(0.05)
    node.pub.publish(Twist())          # brake
    time.sleep(0.3)


def run_demo(node, evidence_dir):
    import threading

    def spinner():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

    threading.Thread(target=spinner, daemon=True).start()

    print("Waiting for first /lunabot/odom message ...")
    t0 = time.time()
    while node.odom is None and time.time() - t0 < 15:
        time.sleep(0.1)
    if node.odom is None:
        print("DEMO FAIL: no odometry received within 15 s")
        return 1
    print("Odometry received. Starting drive test.")

    s0 = node.snapshot()
    drive(node, MAX_LINEAR, 0.0, 3.0)
    s1 = node.snapshot()
    drive(node, 0.0, 0.6, 3.0)
    s2 = node.snapshot()

    dist = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
    dyaw = math.atan2(math.sin(s2[2] - s1[2]), math.cos(s2[2] - s1[2]))
    ok = dist > 0.3 and abs(dyaw) > 0.3

    lines = [
        "LunaBot V4 - Phase A - automated demo drive result",
        "====================================================",
        f"time                 : {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
        f"forward 3 s @ {MAX_LINEAR} m/s : distance = {dist:.3f} m (expect > 0.3 m)",
        f"turn 3 s @ 0.6 rad/s  : yaw delta  = {dyaw:+.3f} rad (expect > 0.3 rad)",
        f"start pose (x,y,yaw)  : {s0[0]:.3f}, {s0[1]:.3f}, {s0[2]:.3f}",
        f"after forward         : {s1[0]:.3f}, {s1[1]:.3f}, {s1[2]:.3f}",
        f"after turn            : {s2[0]:.3f}, {s2[1]:.3f}, {s2[2]:.3f}",
        f"result                : {'PASS' if ok else 'FAIL'}",
    ]
    for line in lines:
        print(line)
    if evidence_dir:
        os.makedirs(evidence_dir, exist_ok=True)
        with open(os.path.join(evidence_dir, "demo_drive_result.txt"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
    return 0 if ok else 1


def run_interactive(node):
    import select
    import termios
    import tty

    if not sys.stdin.isatty():
        print("Interactive teleop needs a terminal. Use --demo for headless runs.")
        return 1

    settings = termios.tcgetattr(sys.stdin)
    speed = MAX_LINEAR
    turn = 0.6

    def get_key():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    print("WASD to drive, Space to stop, Q to quit")
    try:
        while True:
            key = get_key()
            msg = Twist()
            if key == 'w':
                msg.linear.x = speed
                print("Moving forward")
            elif key == 's':
                msg.linear.x = -speed
                print("Moving backward")
            elif key == 'a':
                msg.angular.z = turn
                print("Turning left")
            elif key == 'd':
                msg.angular.z = -turn
                print("Turning right")
            elif key == ' ':
                print("Stopping")
            elif key in ('q', 'Q', '\x03'):
                break
            node.pub.publish(msg)
    except Exception as e:
        print(e)
    finally:
        node.pub.publish(Twist())     # always stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


def main():
    ap = argparse.ArgumentParser(description="LunaBot V4 WASD teleop / demo drive")
    ap.add_argument("--demo", action="store_true",
                    help="run the automated drive test instead of interactive WASD")
    ap.add_argument("evidence_dir", nargs="?", default="",
                    help="optional evidence dir for --demo results")
    args = ap.parse_args()

    rclpy.init()
    node = TeleopNode()
    rc = 1
    try:
        if args.demo:
            rc = run_demo(node, args.evidence_dir)
        else:
            rc = run_interactive(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
