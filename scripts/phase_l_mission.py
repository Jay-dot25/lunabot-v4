#!/usr/bin/env python3
"""LunaBot V4 Phase L: final mission demonstration monitor.

This node is deliberately observation-only. It watches the already validated
Phase K outputs and publishes a retained mission result; it never publishes
velocity, goals, or navigation state.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class PhaseLMission(Node):
    """Aggregate the final autonomous mission's retained runtime evidence."""

    def __init__(self) -> None:
        super().__init__("lunabot_phase_l_mission")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("replan_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("autonomy_topic", "/lunabot/autonomy/status")
        self.declare_parameter("evaluation_topic", "/lunabot/evaluation/status")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("status_topic", "/lunabot/mission/status")

        get = self.get_parameter
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained_qos = QoSProfile(depth=1)
        retained_qos.reliability = ReliabilityPolicy.RELIABLE
        retained_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plan_sub = self.create_subscription(
            Path, str(get("plan_topic").value), self._plan_callback, retained_qos)
        self.replan_sub = self.create_subscription(
            String, str(get("replan_topic").value), self._replan_callback,
            retained_qos)
        self.autonomy_sub = self.create_subscription(
            String, str(get("autonomy_topic").value), self._autonomy_callback,
            retained_qos)
        self.evaluation_sub = self.create_subscription(
            String, str(get("evaluation_topic").value),
            self._evaluation_callback, retained_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, str(get("goal_topic").value), self._goal_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, str(get("map_topic").value), self._map_callback,
            retained_qos)
        self.status_pub = self.create_publisher(
            String, str(get("status_topic").value), status_qos)

        self.plan_seen = False
        self.replan_pass = False
        self.goal_reached = False
        self.evaluation_pass = False
        self.goal_seen = False
        self.map_seen = False
        self.map_updates = 0
        self.last_status = ""
        self.passed = False
        self.publish_status("MISSION_WAITING_FOR_DATA")
        self.timer = self.create_timer(0.2, self._evaluate)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _plan_callback(self, msg: Path) -> None:
        self.plan_seen = self.plan_seen or bool(msg.poses)
        self._evaluate()

    def _replan_callback(self, msg: String) -> None:
        self.replan_pass = self.replan_pass or "DYNAMIC_REPLAN_PASS" in msg.data
        self._evaluate()

    def _autonomy_callback(self, msg: String) -> None:
        self.goal_reached = self.goal_reached or "INTEGRATION_GOAL_REACHED" in msg.data
        self._evaluate()

    def _evaluation_callback(self, msg: String) -> None:
        self.evaluation_pass = self.evaluation_pass or "EVALUATION_PASS" in msg.data
        self._evaluate()

    def _goal_callback(self, msg: PoseStamped) -> None:
        # Receiving a real PoseStamped is the goal-publication criterion; the
        # integrated status separately proves that the goal was reached.
        self.goal_seen = True
        self._evaluate()

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width > 0 and msg.info.height > 0 and msg.data:
            self.map_seen = True
            self.map_updates += 1
        self._evaluate()

    def _evaluate(self) -> None:
        if self.passed:
            return
        criteria = (
            self.plan_seen,
            self.replan_pass,
            self.goal_reached,
            self.evaluation_pass,
            self.goal_seen,
            self.map_seen,
        )
        if all(criteria):
            self.passed = True
            self.publish_status(
                "MISSION_DEMO_PASS plan=1 replan=1 goal=1 evaluation=1 "
                f"goal_pose=1 map=1 map_updates={self.map_updates}")
            return
        names = ("plan", "replan", "goal", "evaluation", "goal_pose", "map")
        missing = [name for name, valid in zip(names, criteria) if not valid]
        self.publish_status("MISSION_WAITING_FOR_" + ",".join(missing).upper())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhaseLMission()
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
