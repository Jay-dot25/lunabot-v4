#!/usr/bin/env python3
"""
LunaBot V4 - Phase A - WASD teleoperation + automated demo drive test
=======================================================================

Interactive mode (default):
    W forward | S reverse | A turn left | D turn right | Space stop | Q quit
    Publishes geometry_msgs/msg/Twist on /cmd_vel.

Demo mode (headless / automated validation):
    python3 wasd_teleop.py --demo [evidence_dir]
    Sequence (SIMULATION time, via /clock - robust to slow rendering):
        forward 3 s -> stop 0.5 s -> turn left 3 s -> stop 0.5 s
    Reports odometry deltas and writes to <evidence_dir>:
        demo_drive_result.txt   summary + PASS/FAIL
        diag_drive.csv          per-sample diagnostics:
                                sim time, commanded vs measured velocity,
                                rover pose, actual wheel joint velocities
                                (used to separate wheel slip / traction
                                issues from simulation-time lag)
    Exit code 0 = movement verified.

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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

MAX_LINEAR = 0.45   # m/s  (DiffDrive plugin limit in model.sdf)
MAX_ANGULAR = 1.0   # rad/s
WHEEL_R = 0.17      # m (model.sdf)
WHEEL_JOINTS = (
    'left_front_wheel_joint', 'left_middle_wheel_joint', 'left_rear_wheel_joint',
    'right_front_wheel_joint', 'right_middle_wheel_joint', 'right_rear_wheel_joint',
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TeleopNode(Node):
    def __init__(self, topic='/cmd_vel'):
        super().__init__('wasd_teleop')
        self.topic = topic
        self.pub = self.create_publisher(Twist, topic, 10)
        self.odom = None
        self.joint_vel = {}
        self.sim_t = None
        self.cmd = (0.0, 0.0)          # last commanded (v, wz)
        # Gazebo bridge sensor/clock publishers use best-effort QoS.
        # Use the ROS 2 sensor profile so Python subscribers match them.
        self.create_subscription(Odometry, '/lunabot/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/lunabot/joint_states',
                                 self._js_cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._clock_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, msg):
        self.odom = msg

    def _js_cb(self, msg):
        for name, vel in zip(msg.name, msg.velocity):
            if name in WHEEL_JOINTS:
                self.joint_vel[name] = vel

    def _clock_cb(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def cmd_of(self, v, wz):
        self.cmd = (v, wz)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = wz
        self.pub.publish(msg)

    def snapshot(self):
        """(x, y, yaw, odom_v, odom_wz, mean left wheel omega, mean right)"""
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        t = self.odom.twist.twist
        lv = [self.joint_vel[j] for j in WHEEL_JOINTS[:3] if j in self.joint_vel]
        rv = [self.joint_vel[j] for j in WHEEL_JOINTS[3:] if j in self.joint_vel]
        return (p.x, p.y, yaw_from_quaternion(q),
                t.linear.x, t.angular.z,
                sum(lv) / len(lv) if lv else 0.0,
                sum(rv) / len(rv) if rv else 0.0)


def run_demo(node, evidence_dir, phase_label=None):
    """Run the controlled drive test while spinning rclpy in this thread.

    rclpy's global executor is not safe to lazily construct from a background
    thread on ROS 2 Humble. Synchronous spin_once keeps command publication and
    subscription callbacks in one executor context.
    """
    # Later phases preserve the Phase B command boundary, so the topic alone
    # cannot identify the active phase. Launchers may provide an evidence label.
    phase_label = phase_label or ('Phase B' if node.topic == '/cmd_vel_in' else 'Phase A')

    def write_failure(reason):
        lines = [
            f"LunaBot V4 - {phase_label} - automated demo drive result",
            "====================================================",
            f"time                 : {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
            f"failure_reason       : {reason}",
            "result                : FAIL",
        ]
        for line in lines:
            print(line, flush=True)
        if evidence_dir:
            os.makedirs(evidence_dir, exist_ok=True)
            with open(os.path.join(evidence_dir, "demo_drive_result.txt"), "w") as fh:
                fh.write("\n".join(lines) + "\n")
        return 1

    print("Waiting for /lunabot/odom and /clock ...", flush=True)
    deadline = time.monotonic() + 15.0
    while (node.odom is None or node.sim_t is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.odom is None or node.sim_t is None:
        return write_failure("odom/clock not received within 15 s")
    print("Odometry + sim time received. Starting drive test (sim-time based).",
          flush=True)

    csv_path = os.path.join(evidence_dir, "diag_drive.csv") if evidence_dir else None
    csv = open(csv_path, "w") if csv_path else None
    if csv:
        csv.write("t_wall,t_sim,cmd_v,cmd_wz,odom_v,odom_wz,"
                  "x,y,yaw,wheel_omega_left,wheel_omega_right\n")

    def sample():
        s = node.snapshot()
        if s and csv:
            csv.write(f"{time.time():.2f},{node.sim_t:.3f},"
                      f"{node.cmd[0]:.3f},{node.cmd[1]:.3f},"
                      f"{s[3]:.3f},{s[4]:.3f},{s[0]:.3f},{s[1]:.3f},{s[2]:.3f},"
                      f"{s[5]:.3f},{s[6]:.3f}\n")
            csv.flush()

    def drive(v, wz, sim_dur):
        """Drive until SIMULATION time advances by sim_dur seconds."""
        start = node.sim_t
        deadline = time.monotonic() + 120.0
        while (node.sim_t - start < sim_dur and
               time.monotonic() < deadline):
            node.cmd_of(v, wz)
            rclpy.spin_once(node, timeout_sec=0.02)
            sample()
        node.cmd_of(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        return node.sim_t - start >= sim_dur

    s0 = node.snapshot()
    if s0 is None:
        if csv:
            csv.close()
        return write_failure("odometry snapshot unavailable after startup")
    if not drive(MAX_LINEAR, 0.0, 3.0):
        if csv:
            csv.close()
        return write_failure("simulation time stopped during forward phase")
    s1 = node.snapshot()
    drive(0.0, 0.0, 0.5)
    if not drive(0.0, 0.6, 3.0):
        if csv:
            csv.close()
        return write_failure("simulation time stopped during turn phase")
    s2 = node.snapshot()
    drive(0.0, 0.0, 0.5)
    if csv:
        csv.close()

    dist = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
    dyaw = math.atan2(math.sin(s2[2] - s1[2]), math.cos(s2[2] - s1[2]))
    # efficiency vs ideal (accel ramp included: ~1.10 m and ~1.54 rad)
    eff_lin = dist / 1.10
    eff_ang = abs(dyaw) / 1.54
    ok = dist > 0.6 and abs(dyaw) > 0.8

    lines = [
        f"LunaBot V4 - {phase_label} - automated demo drive result",
        "====================================================",
        f"time                 : {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
        f"forward 3 s(sim) @ {MAX_LINEAR} m/s : distance = {dist:.3f} m (expect > 0.6 m)",
        f"turn 3 s(sim) @ 0.6 rad/s          : yaw delta  = {dyaw:+.3f} rad (expect > 0.8 rad)",
        f"forward efficiency : {eff_lin * 100:.0f}% of ideal",
        f"turn efficiency    : {eff_ang * 100:.0f}% of ideal",
        f"start pose (x,y,yaw)  : {s0[0]:.3f}, {s0[1]:.3f}, {s0[2]:.3f}",
        f"after forward         : {s1[0]:.3f}, {s1[1]:.3f}, {s1[2]:.3f}",
        f"after turn            : {s2[0]:.3f}, {s2[1]:.3f}, {s2[2]:.3f}",
        f"diag csv             : {csv_path or '(none)'}",
        f"result                : {'PASS' if ok else 'FAIL'}",
    ]
    for line in lines:
        print(line, flush=True)
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

    print("WASD to drive, Space to stop, Q to quit", flush=True)
    try:
        while True:
            key = get_key()
            msg = Twist()
            if key == 'w':
                msg.linear.x = speed
                print("Moving forward", flush=True)
            elif key == 's':
                msg.linear.x = -speed
                print("Moving backward", flush=True)
            elif key == 'a':
                msg.angular.z = turn
                print("Turning left", flush=True)
            elif key == 'd':
                msg.angular.z = -turn
                print("Turning right", flush=True)
            elif key == ' ':
                print("Stopping", flush=True)
            elif key in ('q', 'Q', '\x03'):
                break
            node.pub.publish(msg)
    except Exception as e:
        print(e, flush=True)
    finally:
        node.pub.publish(Twist())     # always stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


def main():
    ap = argparse.ArgumentParser(description="LunaBot V4 WASD teleop / demo drive")
    ap.add_argument("--demo", action="store_true",
                    help="run the automated drive test instead of interactive WASD")
    ap.add_argument("--topic", default="/cmd_vel",
                    help="Twist command topic (Phase A: /cmd_vel; Phase B: /cmd_vel_in)")
    ap.add_argument("--phase-label", default=None,
                    help="phase name written to demo evidence (for example 'Phase C')")
    ap.add_argument("evidence_dir", nargs="?", default="",
                    help="optional evidence dir for --demo results")
    args = ap.parse_args()

    rclpy.init()
    node = TeleopNode(args.topic)
    rc = 1
    try:
        if args.demo:
            rc = run_demo(node, args.evidence_dir, args.phase_label)
        else:
            rc = run_interactive(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
