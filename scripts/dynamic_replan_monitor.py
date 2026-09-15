#!/usr/bin/env python3
"""LunaBot V4 Phase J: monitor live terrain-plan replanning.

The Phase H terrain-aware planner already replans from the live cost map and
odometry. Phase J makes that behavior an explicit, auditable contract: this
node observes the active terrain path, shared goal, and real odometry, counts
changed path revisions after rover motion, and publishes a latched status.
It never publishes velocity and cannot compete with the Phase I follower.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class DynamicReplanMonitor(Node):
    """Prove that live motion produces changed terrain-aware path revisions."""

    def __init__(self) -> None:
        super().__init__("lunabot_dynamic_replan_monitor")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("planner_status_topic", "/lunabot/terrain/planner/status")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/status")
        self.declare_parameter("status_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("require_obstacle", False)
        self.declare_parameter("minimum_revisions", 2)
        self.declare_parameter("minimum_motion", 0.05)

        get = self.get_parameter
        self.plan_topic = str(get("plan_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.planner_status_topic = str(get("planner_status_topic").value)
        self.obstacle_topic = str(get("obstacle_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.require_obstacle = bool(get("require_obstacle").value)
        self.minimum_revisions = max(1, int(get("minimum_revisions").value))
        self.minimum_motion = max(0.01, float(get("minimum_motion").value))

        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.plan_sub = self.create_subscription(
            Path, self.plan_topic, self._plan_callback, plan_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.planner_status_sub = self.create_subscription(
            String, self.planner_status_topic, self._planner_status_callback,
            plan_qos)
        self.obstacle_sub = self.create_subscription(
            String, self.obstacle_topic, self._obstacle_callback, plan_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)

        self.goal_seen = False
        self.last_signature: Optional[tuple] = None
        self.revisions = 0
        self.total_motion = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_planner_status = ""
        self.obstacle_seen = False
        self.last_status = ""
        self.passed = False
        self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_PLAN")
        self.timer = self.create_timer(0.2, self._evaluate)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _goal_callback(self, _msg: PoseStamped) -> None:
        self.goal_seen = True
        self._evaluate()

    def _planner_status_callback(self, msg: String) -> None:
        self.last_planner_status = msg.data

    def _obstacle_callback(self, msg: String) -> None:
        self.obstacle_seen = self.obstacle_seen or "OBSTACLE_DETECTED" in msg.data
        self._evaluate()

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_x is not None and self.last_y is not None:
            self.total_motion += math.hypot(x - self.last_x, y - self.last_y)
        self.last_x, self.last_y = x, y
        self._evaluate()

    @staticmethod
    def _signature(msg: Path) -> tuple:
        return tuple(
            (round(pose.pose.position.x, 2), round(pose.pose.position.y, 2))
            for pose in msg.poses
        )

    def _plan_callback(self, msg: Path) -> None:
        # Keep the successful marker latched. The planner may continue
        # publishing later revisions while the follower settles at its goal,
        # but those updates must not overwrite DYNAMIC_REPLAN_PASS before the
        # launcher can retrieve the retained evidence.
        if self.passed:
            return
        if not msg.poses:
            self.publish_status("DYNAMIC_REPLAN_EMPTY_PLAN")
            return
        signature = self._signature(msg)
        if self.last_signature is None:
            self.last_signature = signature
            self.publish_status(
                f"DYNAMIC_REPLAN_PLAN_RECEIVED cells={len(signature)}")
            return
        if signature != self.last_signature:
            self.revisions += 1
            self.last_signature = signature
            self.publish_status(
                f"DYNAMIC_REPLAN_UPDATE revision={self.revisions} "
                f"cells={len(signature)} motion={self.total_motion:.2f}")
        self._evaluate()

    def _evaluate(self) -> None:
        if self.passed:
            return
        if self.last_signature is None:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_PLAN")
            return
        if not self.goal_seen:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_GOAL")
            return
        if self.require_obstacle and not self.obstacle_seen:
            self.publish_status("DYNAMIC_REPLAN_WAITING_FOR_OBSTACLE")
            return
        if self.revisions < self.minimum_revisions:
            self.publish_status(
                f"DYNAMIC_REPLAN_WAITING_FOR_UPDATE revisions={self.revisions}/"
                f"{self.minimum_revisions}")
            return
        if self.total_motion < self.minimum_motion:
            self.publish_status(
                f"DYNAMIC_REPLAN_WAITING_FOR_MOTION motion={self.total_motion:.2f}")
            return
        self.passed = True
        self.publish_status(
            f"DYNAMIC_REPLAN_PASS revisions={self.revisions} "
            f"motion={self.total_motion:.2f} obstacle={int(self.obstacle_seen)} "
            f"planner={self.last_planner_status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicReplanMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
