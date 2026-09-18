#!/usr/bin/env python3
"""
LunaBot V4 Phase B odometry monitor.

Subscribes to the DiffDrive odometry stream and records an auditable report.
It does not replace the simulator's odometry source; it measures that source.

OUTPUT FILES (when --evidence-dir is supplied)
  odometry_samples.csv  every received odometry sample
  odometry_report.txt   rates, duration, travelled distance, yaw, continuity,
                        frame IDs, and PASS/FAIL quality result
  /lunabot/odometry/status  std_msgs/msg/String live quality status
"""

import argparse
import csv
import math
import os
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class OdometryMonitor(Node):
    def __init__(self, evidence_dir, phase_label='Phase B'):
        super().__init__('lunabot_odometry_monitor')
        self.evidence_dir = evidence_dir
        self.phase_label = phase_label
        if evidence_dir:
            os.makedirs(evidence_dir, exist_ok=True)
        self.status_pub = self.create_publisher(
            String, '/lunabot/odometry/status', 10)
        # Gazebo bridge publishes odometry with sensor-style best-effort QoS.
        self.create_subscription(Odometry, '/lunabot/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.samples = 0
        self.first_wall = None
        self.last_wall = None
        self.first_sim = None
        self.last_sim = None
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.total_distance = 0.0
        self.total_yaw = 0.0
        self.max_step = 0.0
        self.max_speed = 0.0
        self.sum_speed = 0.0
        self.frame_ids = set()
        self.child_frame_ids = set()
        self._closed = False
        self.csv_file = None
        self.csv_writer = None
        if evidence_dir:
            self.csv_file = open(
                os.path.join(evidence_dir, 'odometry_samples.csv'),
                'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'wall_time', 'sim_time', 'x', 'y', 'yaw',
                'linear_x', 'angular_z', 'step_distance', 'frame_id',
                'child_frame_id'])
            self.csv_file.flush()
        self.create_timer(1.0, self._publish_status)

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _unwrap(delta):
        return math.atan2(math.sin(delta), math.cos(delta))

    def _odom_cb(self, msg):
        now = time.time()
        sim_time = self._stamp_seconds(msg.header.stamp)
        p = msg.pose.pose.position
        yaw = self._yaw(msg.pose.pose.orientation)
        step = 0.0
        yaw_step = 0.0
        if self.last_x is not None:
            step = math.hypot(p.x - self.last_x, p.y - self.last_y)
            yaw_step = self._unwrap(yaw - self.last_yaw)
            self.total_distance += step
            self.total_yaw += yaw_step
        self.max_step = max(self.max_step, step)
        speed = abs(msg.twist.twist.linear.x)
        self.max_speed = max(self.max_speed, speed)
        self.sum_speed += speed
        self.samples += 1
        self.first_wall = now if self.first_wall is None else self.first_wall
        self.last_wall = now
        self.first_sim = sim_time if self.first_sim is None else self.first_sim
        self.last_sim = sim_time
        self.last_x, self.last_y, self.last_yaw = p.x, p.y, yaw
        if msg.header.frame_id:
            self.frame_ids.add(msg.header.frame_id)
        if msg.child_frame_id:
            self.child_frame_ids.add(msg.child_frame_id)
        if self.csv_writer:
            self.csv_writer.writerow([
                f'{now:.3f}', f'{sim_time:.6f}', f'{p.x:.6f}', f'{p.y:.6f}',
                f'{yaw:.6f}', f'{msg.twist.twist.linear.x:.6f}',
                f'{msg.twist.twist.angular.z:.6f}', f'{step:.6f}',
                msg.header.frame_id, msg.child_frame_id])
            self.csv_file.flush()

    def _quality(self):
        duration = ((self.last_sim - self.first_sim)
                    if self.first_sim is not None and self.last_sim is not None
                    else 0.0)
        rate = (self.samples / duration) if duration > 0 else 0.0
        frames_ok = ('odom' in self.frame_ids and 'chassis' in self.child_frame_ids)
        passed = (self.samples >= 10 and duration >= 1.0 and
                  self.max_step < 0.5 and frames_ok)
        return passed, duration, rate, frames_ok

    def _publish_status(self):
        passed, duration, rate, frames_ok = self._quality()
        msg = String()
        msg.data = (f"{'PASS' if passed else 'WARMING'} samples={self.samples} "
                    f"sim_duration={duration:.2f}s rate={rate:.1f}Hz "
                    f"frames={'OK' if frames_ok else 'CHECK'} "
                    f"max_step={self.max_step:.3f}m")
        self.status_pub.publish(msg)

    def write_report(self):
        if self._closed:
            return
        self._closed = True
        passed, duration, rate, frames_ok = self._quality()
        avg_speed = self.sum_speed / self.samples if self.samples else 0.0
        lines = [
            f'LunaBot V4 - {self.phase_label} - odometry monitor report',
            '================================================',
            f'generated_utc       : {time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}',
            f'samples             : {self.samples}',
            f'simulation_duration : {duration:.3f} s',
            f'odometry_rate       : {rate:.3f} Hz',
            f'total_distance      : {self.total_distance:.6f} m',
            f'total_yaw_change    : {self.total_yaw:.6f} rad',
            f'max_pose_step       : {self.max_step:.6f} m',
            f'max_abs_linear_speed: {self.max_speed:.6f} m/s',
            f'avg_abs_linear_speed: {avg_speed:.6f} m/s',
            f'frame_ids           : {sorted(self.frame_ids)}',
            f'child_frame_ids     : {sorted(self.child_frame_ids)}',
            f'continuity_frames   : {"PASS" if frames_ok else "FAIL"}',
            'quality_result      : ' + ('PASS' if passed else 'FAIL'),
        ]
        for line in lines:
            self.get_logger().info(line)
        if self.evidence_dir:
            with open(os.path.join(self.evidence_dir, 'odometry_report.txt'), 'w') as fh:
                fh.write('\n'.join(lines) + '\n')
            if self.csv_file:
                self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description='LunaBot odometry monitor')
    parser.add_argument('--evidence-dir', default='')
    parser.add_argument('--phase-label', default='Phase B',
                        help='phase name written to the evidence report')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = OdometryMonitor(args.evidence_dir, args.phase_label)

    def stop(_signum, _frame):
        node.write_report()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
