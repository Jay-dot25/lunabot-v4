#!/usr/bin/env python3
"""LunaBot V4 Phase L: final mission demonstration monitor (real metrics).

Combines:
  Phase J (dynamic replanning)
  Phase K (quantitative evaluation)
  goal completion
  obstacle detection
  semantic navigation

Final status contains actual measured values, not just topic existence.

Example final report:
========================================
          LUNABOT MISSION
========================================
MISSION SUCCESS
Goal reached       : YES
Path length        : XX.XX m
Collisions         : X
Replans            : X
Replanning time    : X.XX s
Hazardous terrain  : XX.X %
Avg terrain cost   : XX.X
Evaluation         : PASS
========================================

Only reports success when underlying conditions actually occurred.
Observation-only, no velocity publishing.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class PhaseLMission(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_phase_l_mission")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("replan_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("autonomy_topic", "/lunabot/autonomy/status")
        self.declare_parameter("evaluation_topic", "/lunabot/evaluation/status")
        self.declare_parameter("navigation_topic", "/lunabot/navigation/status")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/status")
        self.declare_parameter("dynamic_obstacle_topic", "/lunabot/dynamic_obstacle/status")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cost_map_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter("semantic_map_topic", "/lunabot/terrain/semantic_map")
        self.declare_parameter("status_topic", "/lunabot/mission/status")
        self.declare_parameter("require_manual_goal", True)

        get = self.get_parameter
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        retained_qos = QoSProfile(depth=1)
        retained_qos.reliability = ReliabilityPolicy.RELIABLE
        retained_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plan_sub = self.create_subscription(Path, str(get("plan_topic").value), self._plan_callback, retained_qos)
        self.replan_sub = self.create_subscription(String, str(get("replan_topic").value), self._replan_callback, retained_qos)
        self.autonomy_sub = self.create_subscription(String, str(get("autonomy_topic").value), self._autonomy_callback, retained_qos)
        self.evaluation_sub = self.create_subscription(String, str(get("evaluation_topic").value), self._evaluation_callback, retained_qos)
        self.navigation_sub = self.create_subscription(String, str(get("navigation_topic").value), self._navigation_callback, retained_qos)
        self.obstacle_sub = self.create_subscription(String, str(get("obstacle_topic").value), self._obstacle_callback, retained_qos)
        self.dynamic_sub = self.create_subscription(String, str(get("dynamic_obstacle_topic").value), self._dynamic_callback, retained_qos)
        self.goal_sub = self.create_subscription(PoseStamped, str(get("goal_topic").value), self._goal_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, str(get("map_topic").value), self._map_callback, retained_qos)
        self.cost_sub = self.create_subscription(OccupancyGrid, str(get("cost_map_topic").value), self._cost_callback, retained_qos)
        self.semantic_sub = self.create_subscription(OccupancyGrid, str(get("semantic_map_topic").value), self._semantic_callback, retained_qos)

        self.require_manual_goal = bool(get("require_manual_goal").value)
        self.status_pub = self.create_publisher(String, str(get("status_topic").value), status_qos)

        # State
        self.plan_seen = False
        self.replan_pass = False
        self.goal_reached = False
        self.evaluation_pass = False
        self.goal_seen = False
        self.manual_goal_seen = False
        self.obstacle_detected = False
        self.map_seen = False
        self.cost_map_seen = False
        self.semantic_map_seen = False
        self.dynamic_replan_complete = False

        self.map_updates = 0

        # Metrics from evaluation
        self.path_length = 0.0
        self.collisions = 0
        self.replans = 0
        self.replanning_time = 0.0
        self.hazard_pct = 0.0
        self.avg_cost = 0.0
        self.success = False
        self.evaluation_metrics_received = False

        self.last_status = ""
        self.passed = False

        self.publish_status("MISSION_WAITING_FOR_DATA")
        self.timer = self.create_timer(0.5, self._evaluate)

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

    def _replan_callback(self, msg: String) -> None:
        if "DYNAMIC_REPLAN_PASS" in msg.data:
            self.replan_pass = True
        # Parse replans if available
        if "revisions=" in msg.data:
            try:
                self.replans = max(self.replans, int(msg.data.split("revisions=")[1].split()[0]))
            except Exception:
                pass

    def _autonomy_callback(self, msg: String) -> None:
        if "INTEGRATION_GOAL_REACHED" in msg.data:
            self.goal_reached = True
            self.success = True

    def _evaluation_callback(self, msg: String) -> None:
        # Parse metrics from new evaluator format
        data = msg.data
        if "EVALUATION_PASS" in data or "EVALUATION_METRICS" in data:
            self.evaluation_metrics_received = True
            if "EVALUATION_PASS" in data:
                self.evaluation_pass = True
            # Parse metrics
            try:
                # Example: EVALUATION_PASS path_length=12.34 success=1 collisions=0 replans=2 replanning_time=1.23s hazard_pct=5.1% avg_cost=22.3 motion=12.34 goal_reached=1 distance_to_goal=0.12
                for part in data.split():
                    if part.startswith("path_length="):
                        self.path_length = float(part.split("=")[1].rstrip("m"))
                    elif part.startswith("collisions="):
                        self.collisions = int(part.split("=")[1])
                    elif part.startswith("replans="):
                        self.replans = max(self.replans, int(part.split("=")[1]))
                    elif part.startswith("replanning_time="):
                        self.replanning_time = float(part.split("=")[1].rstrip("s"))
                    elif part.startswith("hazard_pct="):
                        self.hazard_pct = float(part.split("=")[1].rstrip("%"))
                    elif part.startswith("avg_cost="):
                        self.avg_cost = float(part.split("=")[1])
                    elif part.startswith("success="):
                        self.success = bool(int(part.split("=")[1]))
            except Exception as e:
                self.get_logger().warn(f"Failed to parse evaluation metrics: {e} data={data}")

    def _navigation_callback(self, msg: String) -> None:
        if "MANUAL_GOAL_SELECTED" in msg.data:
            self.manual_goal_seen = True

    def _obstacle_callback(self, msg: String) -> None:
        if "OBSTACLE_DETECTED" in msg.data:
            self.obstacle_detected = True

    def _dynamic_callback(self, msg: String) -> None:
        if "DYNAMIC_OBSTACLE_REPLAN_COMPLETE" in msg.data or "REPLAN_COMPLETE" in msg.data:
            self.dynamic_replan_complete = True
            # Parse replanning time if present
            try:
                if "replanning_time=" in msg.data:
                    rt = msg.data.split("replanning_time=")[1].split()[0].rstrip("s")
                    self.replanning_time = float(rt)
            except Exception:
                pass

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal_seen = True

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width > 0 and msg.info.height > 0 and msg.data:
            self.map_seen = True
            self.map_updates += 1

    def _cost_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width > 0 and msg.info.height > 0 and msg.data:
            self.cost_map_seen = True

    def _semantic_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width > 0 and msg.info.height > 0 and msg.data:
            self.semantic_map_seen = True

    def _evaluate(self) -> None:
        if self.passed:
            return

        criteria = (
            self.plan_seen,
            self.replan_pass or self.dynamic_replan_complete,
            self.goal_reached,
            self.evaluation_pass,
            self.goal_seen,
            self.map_seen,
            self.cost_map_seen,
            self.semantic_map_seen,
            self.obstacle_detected,
            (self.manual_goal_seen or not self.require_manual_goal),
        )

        if all(criteria):
            self.passed = True
            # Final mission report with real metrics
            report = (
                f"\n========================================\n"
                f"          LUNABOT MISSION\n"
                f"========================================\n"
                f"MISSION SUCCESS\n\n"
                f"Goal reached       : {'YES' if self.goal_reached else 'NO'}\n"
                f"Path length        : {self.path_length:.2f} m\n"
                f"Collisions         : {self.collisions}\n"
                f"Replans            : {self.replans}\n"
                f"Replanning time    : {self.replanning_time:.2f} s\n"
                f"Hazardous terrain  : {self.hazard_pct:.1f} %\n"
                f"Avg terrain cost   : {self.avg_cost:.1f}\n"
                f"Evaluation         : {'PASS' if self.evaluation_pass else 'FAIL'}\n"
                f"Semantic map       : {'YES' if self.semantic_map_seen else 'NO'}\n"
                f"Cost map           : {'YES' if self.cost_map_seen else 'NO'}\n"
                f"Obstacle detected  : {'YES' if self.obstacle_detected else 'NO'}\n"
                f"Manual goal        : {'YES' if self.manual_goal_seen else 'NO'}\n"
                f"Map updates        : {self.map_updates}\n"
                f"========================================\n"
            )
            self.get_logger().info(report)

            self.publish_status(
                f"MISSION_DEMO_PASS plan=1 replan=1 goal=1 evaluation=1 "
                f"goal_pose=1 map=1 obstacle=1 manual_goal={int(self.manual_goal_seen)} "
                f"map_updates={self.map_updates} "
                f"path_length={self.path_length:.2f} collisions={self.collisions} "
                f"replans={self.replans} replanning_time={self.replanning_time:.2f}s "
                f"hazard_pct={self.hazard_pct:.1f}% avg_cost={self.avg_cost:.1f} "
                f"success={int(self.success)}"
            )
            return

        names = ("plan", "replan", "goal", "evaluation", "goal_pose", "map", "cost_map", "semantic_map", "obstacle", "manual_goal")
        missing = [name for name, valid in zip(names, criteria) if not valid]
        self.publish_status(
            f"MISSION_WAITING_FOR_{','.join(missing).upper()} "
            f"path={self.path_length:.2f}m replans={self.replans} collisions={self.collisions} "
            f"hazard={self.hazard_pct:.1f}% avg_cost={self.avg_cost:.1f}"
        )


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
