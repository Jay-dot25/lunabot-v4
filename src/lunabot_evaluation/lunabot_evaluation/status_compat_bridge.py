"""Bridge retained legacy status strings to typed Phase 3 interfaces.

This optional node does not publish motion, goals, maps, or legacy topics. It
can run beside the A–L baseline without changing existing consumers.
"""

from __future__ import annotations

import rclpy
from lunabot_common.status_compat import mission_fields, planner_fields, replan_fields
from lunabot_msgs.msg import MissionMetrics, PlannerStatus, ReplanEvent
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class StatusCompatibilityBridge(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_status_compat_bridge")
        self.declare_parameter("planner_legacy_topic", "/lunabot/terrain/planner/status")
        self.declare_parameter("planner_typed_topic", "/lunabot/terrain/planner/status_typed")
        self.declare_parameter("replan_legacy_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("replan_typed_topic", "/lunabot/autonomy/replan_event")
        self.declare_parameter("mission_legacy_topic", "/lunabot/mission/status")
        self.declare_parameter("mission_typed_topic", "/lunabot/mission/metrics")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        parameter = lambda name: str(self.get_parameter(name).value)
        self.planner_pub = self.create_publisher(
            PlannerStatus, parameter("planner_typed_topic"), qos)
        self.replan_pub = self.create_publisher(
            ReplanEvent, parameter("replan_typed_topic"), qos)
        self.mission_pub = self.create_publisher(
            MissionMetrics, parameter("mission_typed_topic"), qos)
        self.create_subscription(
            String, parameter("planner_legacy_topic"), self._planner_callback, qos)
        self.create_subscription(
            String, parameter("replan_legacy_topic"), self._replan_callback, qos)
        self.create_subscription(
            String, parameter("mission_legacy_topic"), self._mission_callback, qos)
        self.get_logger().info("Typed status compatibility bridge ready")

    def _stamp(self, message) -> None:
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ""

    @staticmethod
    def _assign(message, values: dict) -> None:
        for name, value in values.items():
            if hasattr(message, name):
                setattr(message, name, value)

    def _planner_callback(self, legacy: String) -> None:
        message = PlannerStatus()
        self._stamp(message)
        self._assign(message, planner_fields(legacy.data))
        self.planner_pub.publish(message)

    def _replan_callback(self, legacy: String) -> None:
        message = ReplanEvent()
        self._stamp(message)
        self._assign(message, replan_fields(legacy.data))
        self.replan_pub.publish(message)

    def _mission_callback(self, legacy: String) -> None:
        message = MissionMetrics()
        self._stamp(message)
        self._assign(message, mission_fields(legacy.data))
        self.mission_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusCompatibilityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
