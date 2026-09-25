#!/usr/bin/env python3
"""LunaBot V4 Phase K: runtime testing and evaluation monitor.

This node evaluates the already-approved Phase J autonomous run without
publishing velocity or modifying navigation state. It collects real plan,
replanning, goal, command-boundary, controller, and odometry evidence and
publishes a latched aggregate result.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PhaseKEvaluator(Node):
    """Aggregate real runtime evidence into an auditable evaluation result."""

    def __init__(self) -> None:
        super().__init__("lunabot_phase_k_evaluator")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("geometric_plan_topic", "/plan")
        self.declare_parameter("replan_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("autonomy_topic", "/lunabot/autonomy/status")
        self.declare_parameter("control_topic", "/lunabot/control/status")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("cmd_input_topic", "/cmd_vel_in")
        self.declare_parameter("cmd_output_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/lunabot/evaluation/status")
        self.declare_parameter("minimum_motion", 0.05)

        get = self.get_parameter
        self.status_topic = str(get("status_topic").value)
        self.minimum_motion = max(0.01, float(get("minimum_motion").value))

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.plan_sub = self.create_subscription(
            Path, str(get("plan_topic").value), self._plan_callback, latched_qos)
        self.geometric_plan_sub = self.create_subscription(
            Path, str(get("geometric_plan_topic").value),
            self._geometric_plan_callback, latched_qos)
        self.replan_sub = self.create_subscription(
            String, str(get("replan_topic").value), self._replan_callback,
            latched_qos)
        self.autonomy_sub = self.create_subscription(
            String, str(get("autonomy_topic").value), self._autonomy_callback,
            latched_qos)
        self.control_sub = self.create_subscription(
            String, str(get("control_topic").value), self._control_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, str(get("odom_topic").value), self._odom_callback,
            qos_profile_sensor_data)
        self.input_sub = self.create_subscription(
            Twist, str(get("cmd_input_topic").value), self._input_callback, 10)
        self.output_sub = self.create_subscription(
            Twist, str(get("cmd_output_topic").value), self._output_callback, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.plan_seen = False
        self.geometric_baseline_seen = False
        self.terrain_length = 0.0
        self.geometric_length = 0.0
        self.replan_pass = False
        self.goal_reached = False
        self.controller_active = False
        self.input_motion = False
        self.output_motion = False
        self.total_motion = 0.0
        self.last_x = None
        self.last_y = None
        self.input_samples = 0
        self.output_samples = 0
        self.last_replan = ""
        self.last_autonomy = ""
        self.last_status = ""
        self.passed = False
        self.publish_status("EVALUATION_WAITING_FOR_DATA")
        self.timer = self.create_timer(0.2, self._evaluate)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _path_length(msg: Path) -> float:
        return sum(math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y)
            for first, second in zip(msg.poses, msg.poses[1:]))

    def _plan_callback(self, msg: Path) -> None:
        if msg.poses:
            self.plan_seen = True
            self.terrain_length = self._path_length(msg)
        self._evaluate()

    def _geometric_plan_callback(self, msg: Path) -> None:
        if msg.poses:
            self.geometric_baseline_seen = True
            self.geometric_length = self._path_length(msg)
        self._evaluate()

    def _replan_callback(self, msg: String) -> None:
        self.last_replan = msg.data
        if "DYNAMIC_REPLAN_PASS" in msg.data:
            self.replan_pass = True
        self._evaluate()

    def _autonomy_callback(self, msg: String) -> None:
        self.last_autonomy = msg.data
        if "INTEGRATION_GOAL_REACHED" in msg.data:
            self.goal_reached = True
        self._evaluate()

    def _control_callback(self, msg: String) -> None:
        if "ACTIVE" in msg.data:
            self.controller_active = True
        self._evaluate()

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_x is not None and self.last_y is not None:
            self.total_motion += math.hypot(x - self.last_x, y - self.last_y)
        self.last_x, self.last_y = x, y
        self._evaluate()

    @staticmethod
    def _moving(msg: Twist) -> bool:
        return (abs(float(msg.linear.x)) > 0.001 or
                abs(float(msg.angular.z)) > 0.001)

    def _input_callback(self, msg: Twist) -> None:
        self.input_samples += 1
        self.input_motion = self.input_motion or self._moving(msg)
        self._evaluate()

    def _output_callback(self, msg: Twist) -> None:
        self.output_samples += 1
        self.output_motion = self.output_motion or self._moving(msg)
        self._evaluate()

    def _evaluate(self) -> None:
        if self.passed:
            return
        criteria = (
            self.plan_seen,
            self.geometric_baseline_seen,
            self.replan_pass,
            self.goal_reached,
            self.controller_active,
            self.input_motion,
            self.output_motion,
            self.total_motion >= self.minimum_motion,
        )
        if all(criteria):
            self.passed = True
            self.publish_status(
                f"EVALUATION_PASS plan=1 geometric_baseline=1 replan=1 goal=1 "
                f"controller=1 input_motion=1 output_motion=1 "
                f"motion={self.total_motion:.2f} geometric_length={self.geometric_length:.2f} "
                f"terrain_length={self.terrain_length:.2f} "
                f"input_samples={self.input_samples} output_samples={self.output_samples}")
            return
        missing = []
        names = ("plan", "geometric_baseline", "replan", "goal", "controller",
                 "input_motion", "output_motion", "motion")
        for name, valid in zip(names, criteria):
            if not valid:
                missing.append(name)
        self.publish_status("EVALUATION_WAITING_FOR_" + ",".join(missing).upper())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhaseKEvaluator()
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
