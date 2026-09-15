#!/usr/bin/env python3
"""LunaBot V4 Phase H: terrain-aware weighted A* path planning.

This node consumes the Phase G terrain cost map and the existing navigation goal
and publishes a cost-aware path. It deliberately does not publish velocity:
Phase I will integrate terrain-aware planning into autonomous motion after this
path-planning contract is independently validated.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros


class TerrainAwarePlanner(Node):
    """Plan through traversable cells while minimizing terrain cost."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_aware_planner")
        self.declare_parameter("cost_map_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("plan_topic", "/lunabot/terrain/plan")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/planner/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("blocked_cost", 100)
        self.declare_parameter("cost_weight", 2.0)
        self.declare_parameter("unknown_cost", 80)
        self.declare_parameter("replan_period", 1.0)

        get = self.get_parameter
        self.cost_map_topic = str(get("cost_map_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.plan_topic = str(get("plan_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.blocked_cost = int(get("blocked_cost").value)
        self.cost_weight = max(0.0, float(get("cost_weight").value))
        self.unknown_cost = max(0, min(100, int(get("unknown_cost").value)))
        self.replan_period = max(0.2, float(get("replan_period").value))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cost_sub = self.create_subscription(
            OccupancyGrid, self.cost_map_topic, self._cost_callback, map_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.plan_pub = self.create_publisher(Path, self.plan_topic, map_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, map_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cost_map: Optional[OccupancyGrid] = None
        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        self.last_plan_time = self.get_clock().now() - Duration(seconds=10.0)
        self.last_status = ""
        self.publish_status("TERRAIN_PLANNER_WAITING_FOR_COST_MAP")
        self.timer = self.create_timer(0.1, self._tick)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _cost_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width <= 0 or msg.info.height <= 0 or not msg.data:
            self.publish_status("TERRAIN_PLANNER_INVALID_COST_MAP")
            return
        self.cost_map = msg

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal = msg
        self.publish_status(
            f"TERRAIN_GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _transform_xy(self, x: float, y: float, yaw: float,
                      source: str, target: str):
        source = source or target
        if source == target:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(target, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, yaw + tf_yaw

    def _current_pose(self):
        if self.odom is not None:
            pose = self.odom.pose.pose
            return self._transform_xy(
                pose.position.x, pose.position.y, self._yaw(pose.orientation),
                self.odom.header.frame_id or "odom", self.map_frame)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time())
        t = transform.transform.translation
        return t.x, t.y, self._yaw(transform.transform.rotation)

    def _world_to_grid(self, x: float, y: float):
        info = self.cost_map.info
        origin = info.origin
        oyaw = self._yaw(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        c, s = math.cos(oyaw), math.sin(oyaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(mx / info.resolution)), int(math.floor(my / info.resolution))

    def _grid_to_world(self, cell):
        info = self.cost_map.info
        origin = info.origin
        oyaw = self._yaw(origin.orientation)
        mx = (cell[0] + 0.5) * info.resolution
        my = (cell[1] + 0.5) * info.resolution
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (origin.position.x + c * mx - s * my,
                origin.position.y + s * mx + c * my)

    def _cell_cost(self, cell) -> Optional[int]:
        info = self.cost_map.info
        x, y = cell
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return None
        index = y * info.width + x
        if index >= len(self.cost_map.data):
            return None
        value = int(self.cost_map.data[index])
        if value < 0:
            value = self.unknown_cost
        if value >= self.blocked_cost:
            return None
        return max(0, min(100, value))

    def _nearest_traversable(self, cell):
        if self._cell_cost(cell) is not None:
            return cell
        for radius in range(1, 20):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self._cell_cost(candidate) is not None:
                        return candidate
        return None

    def _weighted_astar(self, start, goal):
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                     (1, 1), (1, -1), (-1, 1), (-1, -1))
        open_set = [(0.0, start)]
        came_from = {}
        scores = {start: 0.0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path)), scores[goal]
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                cell_cost = self._cell_cost(nxt)
                if cell_cost is None:
                    continue
                distance = math.sqrt(2.0) if dx and dy else 1.0
                step = distance * (1.0 + self.cost_weight * cell_cost / 100.0)
                tentative = scores[current] + step
                if tentative >= scores.get(nxt, float("inf")):
                    continue
                came_from[nxt] = current
                scores[nxt] = tentative
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_set, (tentative + heuristic, nxt))
        return [], float("inf")

    def _plan(self) -> None:
        if self.cost_map is None or self.goal is None:
            return
        try:
            current_x, current_y, current_yaw = self._current_pose()
            goal_pose = self.goal.pose
            goal_yaw = self._yaw(goal_pose.orientation)
            goal_x, goal_y, goal_yaw = self._transform_xy(
                goal_pose.position.x, goal_pose.position.y, goal_yaw,
                self.goal.header.frame_id or self.map_frame, self.map_frame)
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f"TERRAIN_PLANNER_WAITING_FOR_TF {exc}")
            return
        start_cell = self._nearest_traversable(
            self._world_to_grid(current_x, current_y))
        goal_cell = self._nearest_traversable(
            self._world_to_grid(goal_x, goal_y))
        if start_cell is None or goal_cell is None:
            self.publish_status("TERRAIN_PLANNER_NO_SAFE_START_OR_GOAL")
            return
        cells, total_cost = self._weighted_astar(start_cell, goal_cell)
        if not cells:
            self.publish_status("TERRAIN_PLANNER_NO_PATH")
            return
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, cell in enumerate(cells):
            x, y = self._grid_to_world(cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(cells):
                nx, ny = self._grid_to_world(cells[index + 1])
                heading = math.atan2(ny - y, nx - x)
            else:
                heading = goal_yaw
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)
        self.plan_pub.publish(path)
        self.last_plan_time = self.get_clock().now()
        self.publish_status(
            f"TERRAIN_PLAN_PASS cells={len(cells)} weighted_cost={total_cost:.2f} "
            f"frame={self.map_frame} cost_weight={self.cost_weight:.2f}")

    def _tick(self) -> None:
        if (self.get_clock().now() - self.last_plan_time).nanoseconds < int(self.replan_period * 1e9):
            return
        self._plan()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainAwarePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
