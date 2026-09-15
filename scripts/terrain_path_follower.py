#!/usr/bin/env python3
"""LunaBot V4 Phase I: terrain-aware autonomous path follower.

The follower consumes the Phase H terrain-aware path and sends conservative
commands through the approved Phase B `/cmd_vel_in` boundary. It is the first
phase that integrates the terrain-aware plan with rover motion. The existing
controller/watchdog remains responsible for clamping, acceleration limiting,
and safe `/cmd_vel` output.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros


def angle_error(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TerrainPathFollower(Node):
    """Follow the terrain-aware path while preserving the controller boundary."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_path_follower")
        self.declare_parameter("path_topic", "/lunabot/terrain/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_in")
        self.declare_parameter(
            "status_topic", "/lunabot/autonomy/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.60)
        self.declare_parameter("lookahead_cells", 2)
        self.declare_parameter("control_rate", 10.0)

        get = self.get_parameter
        self.path_topic = str(get("path_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.cmd_topic = str(get("cmd_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.goal_tolerance = max(0.05, float(get("goal_tolerance").value))
        self.max_linear = max(0.02, float(get("max_linear").value))
        self.max_angular = max(0.1, float(get("max_angular").value))
        self.lookahead_cells = max(0, int(get("lookahead_cells").value))

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self._path_callback, path_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path: list[tuple[float, float]] = []
        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        self.reached = False
        self.last_waypoint = -1
        self.last_status = ""
        self.publish_status("INTEGRATION_WAITING_FOR_TERRAIN_PLAN")
        self.timer = self.create_timer(
            1.0 / max(1.0, float(get("control_rate").value)), self._tick)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.publish_status("INTEGRATION_EMPTY_TERRAIN_PLAN")
            return
        self.path = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        # The terrain planner republishes its latched/replanned path. Do not
        # turn a completed goal back into an active one just because an
        # unchanged path arrived on the next planning tick.
        was_reached = self.reached
        self.last_waypoint = -1
        if not was_reached:
            self.publish_status(
                f"INTEGRATION_PATH_RECEIVED cells={len(self.path)}")

    @staticmethod
    def _same_goal(first: PoseStamped, second: PoseStamped) -> bool:
        if (first.header.frame_id or "") != (second.header.frame_id or ""):
            return False
        a, b = first.pose, second.pose
        return (
            abs(a.position.x - b.position.x) < 1e-6 and
            abs(a.position.y - b.position.y) < 1e-6 and
            abs(a.position.z - b.position.z) < 1e-6 and
            abs(a.orientation.x - b.orientation.x) < 1e-6 and
            abs(a.orientation.y - b.orientation.y) < 1e-6 and
            abs(a.orientation.z - b.orientation.z) < 1e-6 and
            abs(a.orientation.w - b.orientation.w) < 1e-6
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        # A* republishes its active goal for late observers. Treat an identical
        # message as state, not as a new command, so it cannot reset progress.
        if self.goal is not None and self._same_goal(msg, self.goal):
            return
        self.goal = msg
        self.reached = False
        self.last_waypoint = -1
        self.publish_status(
            f"INTEGRATION_GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * q.w * q.z,
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _pose(self):
        if self.odom is not None:
            pose = self.odom.pose.pose
            x, y, yaw = pose.position.x, pose.position.y, self._yaw(pose.orientation)
            source = self.odom.header.frame_id or "odom"
        else:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
            t = transform.transform.translation
            return t.x, t.y, self._yaw(transform.transform.rotation)
        if source == self.map_frame:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, angle_error(yaw + tf_yaw)

    def _goal_map(self):
        if self.goal is None:
            return None
        pose = self.goal.pose
        yaw = self._yaw(pose.orientation)
        source = self.goal.header.frame_id or self.map_frame
        if source == self.map_frame:
            return pose.position.x, pose.position.y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * pose.position.x - s * pose.position.y, \
            t.y + s * pose.position.x + c * pose.position.y, \
            angle_error(yaw + tf_yaw)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if not self.path:
            self._publish_stop()
            return
        try:
            current_x, current_y, current_yaw = self._pose()
            goal = self._goal_map()
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f"INTEGRATION_WAITING_FOR_TF {exc}")
            self._publish_stop()
            return
        if goal is None:
            self.publish_status("INTEGRATION_WAITING_FOR_GOAL")
            self._publish_stop()
            return
        goal_distance = math.hypot(goal[0] - current_x, goal[1] - current_y)
        if goal_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(
                    f"INTEGRATION_GOAL_REACHED distance={goal_distance:.2f}")
            return
        nearest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - current_x,
                                               self.path[i][1] - current_y))
        target_index = min(len(self.path) - 1, nearest + self.lookahead_cells)
        target_x, target_y = self.path[target_index]
        heading = math.atan2(target_y - current_y, target_x - current_x)
        error = angle_error(heading - current_yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.0 * error))
        if abs(error) <= 0.9:
            cmd.linear.x = min(self.max_linear, 0.45 * goal_distance)
            cmd.linear.x *= max(0.2, math.cos(error))
        self.cmd_pub.publish(cmd)
        if target_index != self.last_waypoint:
            self.last_waypoint = target_index
            self.publish_status(
                f"INTEGRATION_FOLLOWING waypoint={target_index}/{len(self.path)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
