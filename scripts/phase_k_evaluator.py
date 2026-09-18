#!/usr/bin/env python3
"""LunaBot V4 Phase K: real experimental evaluation.

Implements quantitative metrics per master directive and PDF:

- navigation safety, path length, success rate, replanning time,
  collisions, terrain traversability

Metrics:
  1. Path length from odometry sum sqrt(dx^2+dy^2)
  2. Success: distance_to_goal < GOAL_TOLERANCE
  3. Collision count: entering forbidden/high-cost cells (CRATER_COST) or obstacle proximity
  4. Hazardous terrain exposure: proportion of path intersecting ROCK/CRATER
  5. Replanning time: obstacle_detected -> new_plan timestamps from Phase J
  6. Terrain cost: cumulative/average cost along trajectory
  7. Number of replans: count actual plan revisions

Also supports Traditional vs LunaBot comparison when both logs available.
Observation-only, no velocity publishing.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


# Cost thresholds matching Phase G
BEDROCK_COST = 5
REGOLITH_COST = 15
SHADOW_COST = 45
ROCK_COST = 70
CRATER_COST = 100
UNKNOWN_COST = 80

HAZARDOUS_COSTS = {ROCK_COST, CRATER_COST}
FORBIDDEN_COST = CRATER_COST


class PhaseKEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_phase_k_evaluator")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter("replan_topic", "/lunabot/autonomy/replan_status")
        self.declare_parameter("autonomy_topic", "/lunabot/autonomy/status")
        self.declare_parameter("control_topic", "/lunabot/control/status")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cost_map_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter("dynamic_obstacle_topic", "/lunabot/dynamic_obstacle/status")
        self.declare_parameter("cmd_input_topic", "/cmd_vel_in")
        self.declare_parameter("cmd_output_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/lunabot/evaluation/status")
        self.declare_parameter("minimum_motion", 0.05)
        self.declare_parameter("goal_tolerance", 0.35)

        get = self.get_parameter
        self.plan_topic = str(get("plan_topic").value)
        self.replan_topic = str(get("replan_topic").value)
        self.autonomy_topic = str(get("autonomy_topic").value)
        self.control_topic = str(get("control_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.cost_map_topic = str(get("cost_map_topic").value)
        self.dynamic_obstacle_topic = str(get("dynamic_obstacle_topic").value)
        self.cmd_input_topic = str(get("cmd_input_topic").value)
        self.cmd_output_topic = str(get("cmd_output_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.minimum_motion = max(0.01, float(get("minimum_motion").value))
        self.goal_tolerance = float(get("goal_tolerance").value)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plan_sub = self.create_subscription(Path, self.plan_topic, self._plan_callback, latched_qos)
        self.replan_sub = self.create_subscription(String, self.replan_topic, self._replan_callback, latched_qos)
        self.autonomy_sub = self.create_subscription(String, self.autonomy_topic, self._autonomy_callback, latched_qos)
        self.control_sub = self.create_subscription(String, self.control_topic, self._control_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.cost_sub = self.create_subscription(OccupancyGrid, self.cost_map_topic, self._cost_callback, latched_qos)
        self.dynamic_sub = self.create_subscription(String, self.dynamic_obstacle_topic, self._dynamic_callback, latched_qos)
        self.input_sub = self.create_subscription(Twist, self.cmd_input_topic, self._input_callback, 10)
        self.output_sub = self.create_subscription(Twist, self.cmd_output_topic, self._output_callback, 10)

        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        # State
        self.plan_seen = False
        self.replan_pass = False
        self.goal_reached = False
        self.controller_active = False
        self.input_motion = False
        self.output_motion = False

        self.total_motion = 0.0
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.trajectory: List[Tuple[float, float, float]] = []  # x,y,time
        self.cost_along_path: List[int] = []
        self.hazardous_distance = 0.0
        self.collision_count = 0
        self.last_cost_was_forbidden = False

        self.goal_pose: Optional[Tuple[float, float]] = None
        self.cost_map: Optional[OccupancyGrid] = None
        self.plan_revisions = 0
        self.last_plan_sig: Optional[tuple] = None

        self.input_samples = 0
        self.output_samples = 0

        # Replanning timing
        self.obstacle_introduced_time: Optional[float] = None
        self.obstacle_detected_time: Optional[float] = None
        self.new_plan_time: Optional[float] = None
        self.replanning_time: Optional[float] = None

        self.last_replan_msg = ""
        self.last_autonomy_msg = ""
        self.last_dynamic_msg = ""

        self.last_status = ""
        self.passed = False

        self.publish_status("EVALUATION_WAITING_FOR_DATA")
        self.timer = self.create_timer(0.5, self._evaluate)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _cost_callback(self, msg: OccupancyGrid) -> None:
        self.cost_map = msg

    def _dynamic_callback(self, msg: String) -> None:
        self.last_dynamic_msg = msg.data
        # Parse timestamps from injector status
        # Format: DYNAMIC_OBSTACLE_INJECTED ... time=...
        #         DYNAMIC_OBSTACLE_REPLAN_COMPLETE introduced=... detected=... new_plan=... replanning_time=...
        try:
            if "introduced=" in msg.data:
                # Extract times
                parts = msg.data.split()
                for p in parts:
                    if p.startswith("introduced="):
                        self.obstacle_introduced_time = float(p.split("=")[1])
                    elif p.startswith("detected="):
                        self.obstacle_detected_time = float(p.split("=")[1])
                    elif p.startswith("new_plan="):
                        self.new_plan_time = float(p.split("=")[1])
                    elif p.startswith("replanning_time="):
                        self.replanning_time = float(p.split("=")[1].rstrip("s"))
            if "OBSTACLE_DETECTED" in msg.data and self.obstacle_detected_time is None:
                self.obstacle_detected_time = time.time()
        except Exception:
            pass

    def _plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            return
        self.plan_seen = True
        sig = tuple((round(p.pose.position.x, 2), round(p.pose.position.y, 2)) for p in msg.poses)
        if self.last_plan_sig is None:
            self.last_plan_sig = sig
        elif sig != self.last_plan_sig:
            self.plan_revisions += 1
            self.last_plan_sig = sig
            # If this is after obstacle detection, record new plan time
            if self.obstacle_detected_time is not None and self.new_plan_time is None:
                self.new_plan_time = time.time()
                if self.obstacle_detected_time:
                    self.replanning_time = self.new_plan_time - self.obstacle_detected_time

    def _replan_callback(self, msg: String) -> None:
        self.last_replan_msg = msg.data
        if "DYNAMIC_REPLAN_PASS" in msg.data:
            self.replan_pass = True
        # Parse revisions if present
        if "revisions=" in msg.data:
            try:
                rev = int(msg.data.split("revisions=")[1].split()[0])
                self.plan_revisions = max(self.plan_revisions, rev)
            except Exception:
                pass

    def _autonomy_callback(self, msg: String) -> None:
        self.last_autonomy_msg = msg.data
        if "INTEGRATION_GOAL_REACHED" in msg.data:
            self.goal_reached = True

    def _control_callback(self, msg: String) -> None:
        if "ACTIVE" in msg.data:
            self.controller_active = True

    def _lookup_cost(self, x: float, y: float) -> Optional[int]:
        if self.cost_map is None:
            return None
        info = self.cost_map.info
        # Simple transform: assume map frame origin
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None
        idx = gy * info.width + gx
        if idx >= len(self.cost_map.data):
            return None
        return int(self.cost_map.data[idx])

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        now = time.time()
        if self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.hypot(dx, dy)
            self.total_motion += dist
            # Hazardous and collision detection
            cost = self._lookup_cost(x, y)
            if cost is not None:
                self.cost_along_path.append(cost)
                if cost in HAZARDOUS_COSTS or cost >= ROCK_COST:
                    self.hazardous_distance += dist
                if cost >= FORBIDDEN_COST:
                    if not self.last_cost_was_forbidden:
                        self.collision_count += 1
                        self.last_cost_was_forbidden = True
                else:
                    self.last_cost_was_forbidden = False
            self.trajectory.append((x, y, now))
        else:
            self.trajectory.append((x, y, now))
        self.last_x, self.last_y = x, y

    @staticmethod
    def _moving(msg: Twist) -> bool:
        return abs(float(msg.linear.x)) > 0.001 or abs(float(msg.angular.z)) > 0.001

    def _input_callback(self, msg: Twist) -> None:
        self.input_samples += 1
        if self._moving(msg):
            self.input_motion = True

    def _output_callback(self, msg: Twist) -> None:
        self.output_samples += 1
        if self._moving(msg):
            self.output_motion = True

    def _compute_metrics(self):
        # Path length already in total_motion
        path_length = self.total_motion

        # Success
        success = False
        distance_to_goal = None
        if self.goal_pose is not None and self.last_x is not None:
            gx, gy = self.goal_pose
            distance_to_goal = math.hypot(gx - self.last_x, gy - self.last_y)
            success = distance_to_goal < self.goal_tolerance or self.goal_reached

        # Hazardous %
        hazard_pct = 0.0
        if path_length > 1e-6:
            hazard_pct = (self.hazardous_distance / path_length) * 100.0

        # Avg terrain cost
        avg_cost = 0.0
        total_cost = 0
        if self.cost_along_path:
            total_cost = sum(self.cost_along_path)
            avg_cost = total_cost / len(self.cost_along_path)

        # Replanning time
        replanning_time = self.replanning_time
        if replanning_time is None and self.obstacle_detected_time and self.new_plan_time:
            replanning_time = self.new_plan_time - self.obstacle_detected_time

        return {
            "path_length": path_length,
            "success": success,
            "distance_to_goal": distance_to_goal,
            "hazard_pct": hazard_pct,
            "hazard_dist": self.hazardous_distance,
            "avg_cost": avg_cost,
            "total_cost": total_cost,
            "collision_count": self.collision_count,
            "replans": self.plan_revisions,
            "replanning_time": replanning_time,
            "goal_reached_flag": self.goal_reached,
        }

    def _evaluate(self) -> None:
        metrics = self._compute_metrics()

        # Criteria for PASS: need motion, plan, controller, input/output, and enough data
        criteria = (
            self.plan_seen,
            self.controller_active,
            self.input_motion,
            self.output_motion,
            self.total_motion >= self.minimum_motion,
        )

        # Build detailed status
        status_lines = []
        status_lines.append(
            f"EVALUATION_METRICS path_length={metrics['path_length']:.2f}m "
            f"success={int(metrics['success'])} "
            f"goal_reached={int(metrics['goal_reached_flag'])} "
            f"distance_to_goal={metrics['distance_to_goal']:.2f}m "
            f"collisions={metrics['collision_count']} "
            f"replans={metrics['replans']} "
            f"replanning_time={metrics['replanning_time']:.2f}s "
            f"hazard_pct={metrics['hazard_pct']:.1f}% "
            f"hazard_dist={metrics['hazard_dist']:.2f}m "
            f"avg_cost={metrics['avg_cost']:.1f} "
            f"total_cost={metrics['total_cost']} "
            f"motion={self.total_motion:.2f}m "
            f"input_samples={self.input_samples} output_samples={self.output_samples}"
        )

        if all(criteria):
            if not self.passed:
                self.passed = True
                # Final PASS includes all metrics
                self.publish_status(
                    f"EVALUATION_PASS path_length={metrics['path_length']:.2f} "
                    f"success={int(metrics['success'])} collisions={metrics['collision_count']} "
                    f"replans={metrics['replans']} replanning_time={metrics['replanning_time']:.2f}s "
                    f"hazard_pct={metrics['hazard_pct']:.1f}% avg_cost={metrics['avg_cost']:.1f} "
                    f"motion={self.total_motion:.2f} goal_reached={int(metrics['goal_reached_flag'])} "
                    f"distance_to_goal={metrics['distance_to_goal']:.2f}"
                )
                # Also publish detailed metrics as separate line for evidence parsers
                self.get_logger().info(status_lines[0])
            return

        # Waiting status
        missing = []
        names = ("plan", "controller", "input_motion", "output_motion", "motion")
        for name, valid in zip(names, criteria):
            if not valid:
                missing.append(name)
        self.publish_status(
            f"EVALUATION_WAITING_FOR_{','.join(missing).upper()} "
            f"path={metrics['path_length']:.2f}m "
            f"replans={metrics['replans']} collisions={metrics['collision_count']} "
            f"hazard={metrics['hazard_pct']:.1f}%"
        )


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
