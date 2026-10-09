#!/usr/bin/env python3
"""LunaBot V4 Phase G: semantic terrain cost-map generation.

This node converts the approved Phase F semantic terrain grid into a numeric
traversability cost grid. It is intentionally not connected to A* yet; Phase H
will consume this output for terrain-aware planning.

Cost values are 0..100:
  20 = observed terrain candidate
  80 = unknown / caution
  100 = obstacle or inflated obstacle

Obstacle inflation adds a conservative cost halo without changing the source
semantic map. The node publishes no velocity or localization output.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


UNKNOWN = -1
TERRAIN_VALUE = 25
OBSTACLE_VALUE = 100


class TerrainCostMapper(Node):
    """Convert semantic terrain labels into an inflated cost grid."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_cost_mapper")
        self.declare_parameter("semantic_topic", "/lunabot/terrain/semantic_map")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/map")
        self.declare_parameter("slam_map_topic", "/map")
        self.declare_parameter("cost_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/cost_map/status")
        self.declare_parameter("terrain_cost", 20)
        self.declare_parameter("unknown_cost", 80)
        self.declare_parameter("obstacle_cost", 100)
        self.declare_parameter("slam_obstacle_threshold", 65)
        self.declare_parameter("fuse_slam_map", False)
        self.declare_parameter("inflation_radius", 0.50)

        get = self.get_parameter
        self.semantic_topic = str(get("semantic_topic").value)
        self.obstacle_topic = str(get("obstacle_topic").value)
        self.slam_map_topic = str(get("slam_map_topic").value)
        self.cost_topic = str(get("cost_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.terrain_cost = max(0, min(100, int(get("terrain_cost").value)))
        self.unknown_cost = max(0, min(100, int(get("unknown_cost").value)))
        self.obstacle_cost = max(0, min(100, int(get("obstacle_cost").value)))
        self.slam_obstacle_threshold = max(
            1, min(100, int(get("slam_obstacle_threshold").value)))
        self.fuse_slam_map = bool(get("fuse_slam_map").value)
        self.inflation_radius = max(0.0, float(get("inflation_radius").value))

        input_qos = QoSProfile(depth=1)
        input_qos.reliability = ReliabilityPolicy.RELIABLE
        input_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.semantic_sub = self.create_subscription(
            OccupancyGrid, self.semantic_topic, self._semantic_callback, input_qos)
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, self.obstacle_topic, self._obstacle_callback, input_qos)
        self.slam_sub = self.create_subscription(
            OccupancyGrid, self.slam_map_topic, self._slam_callback, input_qos)
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cost_pub = self.create_publisher(
            OccupancyGrid, self.cost_topic, output_qos)
        self.status_pub = self.create_publisher(
            String, self.status_topic, output_qos)
        self.frame_count = 0
        self.last_status = ""
        self.last_cost_map: Optional[OccupancyGrid] = None
        self.last_semantic_map: Optional[OccupancyGrid] = None
        self.last_obstacle_map: Optional[OccupancyGrid] = None
        self.last_slam_map: Optional[OccupancyGrid] = None
        self.publish_status("COST_MAP_WAITING_FOR_SEMANTIC_MAP")

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _base_cost(self, value: int) -> int:
        if value >= OBSTACLE_VALUE:
            return self.obstacle_cost
        if value == TERRAIN_VALUE or 0 < value < OBSTACLE_VALUE:
            return self.terrain_cost
        return self.unknown_cost

    def _inflate(self, costs: list[int], width: int, height: int,
                 resolution: float) -> tuple[int, int]:
        radius_cells = int(math.ceil(self.inflation_radius / max(resolution, 1e-6)))
        if radius_cells <= 0:
            return 0, 0
        obstacles = [i for i, value in enumerate(costs)
                     if value >= self.obstacle_cost]
        inflated = 0
        for index in obstacles:
            ox, oy = index % width, index // width
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    distance = math.hypot(dx, dy)
                    if distance > radius_cells:
                        continue
                    x, y = ox + dx, oy + dy
                    if x < 0 or y < 0 or x >= width or y >= height:
                        continue
                    target = y * width + x
                    if target == index:
                        continue
                    # The halo remains high-cost but drops with distance; an
                    # existing obstacle or higher cost is never overwritten.
                    if distance * resolution <= 0.46:
                        halo = 96
                    else:
                        halo = max(self.terrain_cost + 1,
                                   int(self.obstacle_cost - 35.0 * distance / radius_cells))
                    if halo > costs[target]:
                        costs[target] = min(self.obstacle_cost, halo)
                        inflated += 1
        return len(obstacles), inflated

    def _obstacle_callback(self, msg: OccupancyGrid) -> None:
        self.last_obstacle_map = msg
        if self.last_semantic_map is not None:
            self._publish_cost(self.last_semantic_map)

    def _slam_callback(self, msg: OccupancyGrid) -> None:
        self.last_slam_map = msg
        if self.fuse_slam_map and self.last_semantic_map is not None:
            self._publish_cost(self.last_semantic_map)

    def _semantic_callback(self, msg: OccupancyGrid) -> None:
        self.last_semantic_map = msg
        self._publish_cost(msg)

    def _fuse_external_grid(self, target_msg: OccupancyGrid, costs: list[int],
                            overlay: Optional[OccupancyGrid],
                            obstacle_threshold: int,
                            fuse_free_space: bool = False) -> int:
        if overlay is None:
            return 0
        width, height = int(target_msg.info.width), int(target_msg.info.height)
        src_w, src_h = int(overlay.info.width), int(overlay.info.height)
        if src_w <= 0 or src_h <= 0 or len(overlay.data) < src_w * src_h:
            return 0
        dst_res = max(float(target_msg.info.resolution), 1e-6)
        src_res = max(float(overlay.info.resolution), 1e-6)
        dst_ox = float(target_msg.info.origin.position.x)
        dst_oy = float(target_msg.info.origin.position.y)
        src_ox = float(overlay.info.origin.position.x)
        src_oy = float(overlay.info.origin.position.y)
        added_obstacles = 0

        same_geometry = (
            src_w == width and
            src_h == height and
            abs(src_res - dst_res) < 1e-6 and
            abs(src_ox - dst_ox) < 1e-6 and
            abs(src_oy - dst_oy) < 1e-6 and
            len(overlay.data) >= width * height)
        if same_geometry:
            for index, raw_val in enumerate(overlay.data[:width * height]):
                val = int(raw_val)
                if val >= obstacle_threshold:
                    if costs[index] < self.obstacle_cost:
                        added_obstacles += 1
                    costs[index] = self.obstacle_cost
                elif fuse_free_space and 0 <= val < TERRAIN_VALUE and costs[index] == self.unknown_cost:
                    costs[index] = self.terrain_cost
            return added_obstacles

        for sy in range(src_h):
            wy = src_oy + (sy + 0.5) * src_res
            ty = int(math.floor((wy - dst_oy) / dst_res))
            if ty < 0 or ty >= height:
                continue
            row_offset = sy * src_w
            dst_row_offset = ty * width
            for sx in range(src_w):
                val = int(overlay.data[row_offset + sx])
                if val < 0:
                    continue
                wx = src_ox + (sx + 0.5) * src_res
                tx = int(math.floor((wx - dst_ox) / dst_res))
                if tx < 0 or tx >= width:
                    continue
                target_idx = dst_row_offset + tx
                if val >= obstacle_threshold:
                    if costs[target_idx] < self.obstacle_cost:
                        added_obstacles += 1
                    costs[target_idx] = self.obstacle_cost
                elif fuse_free_space and val < TERRAIN_VALUE and costs[target_idx] == self.unknown_cost:
                    costs[target_idx] = self.terrain_cost
        return added_obstacles

    def _publish_cost(self, msg: OccupancyGrid) -> None:
        width, height = int(msg.info.width), int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) < width * height:
            self.publish_status("COST_MAP_INVALID_SEMANTIC_MAP")
            return
        costs = [self._base_cost(int(value)) for value in msg.data[:width * height]]
        sensed_obstacles = self._fuse_external_grid(
            msg, costs, self.last_obstacle_map, OBSTACLE_VALUE, fuse_free_space=False)
        if self.fuse_slam_map:
            slam_obstacles = self._fuse_external_grid(
                msg, costs, self.last_slam_map, self.slam_obstacle_threshold, fuse_free_space=True)
            sensed_obstacles += slam_obstacles
        obstacle_count, inflated_count = self._inflate(
            costs, width, height, float(msg.info.resolution))
        output = OccupancyGrid()
        output.header = msg.header
        output.header.stamp = self.get_clock().now().to_msg()
        output.info = msg.info
        output.data = costs
        self.last_cost_map = output
        self.cost_pub.publish(output)
        self.frame_count += 1
        unknown_count = sum(value == self.unknown_cost for value in costs)
        terrain_count = sum(value == self.terrain_cost for value in costs)
        self.publish_status(
            f"COST_MAP_PASS frames={self.frame_count} width={width} height={height} "
            f"terrain_cells={terrain_count} unknown_cells={unknown_count} "
            f"obstacle_cells={obstacle_count} sensed_obstacles={sensed_obstacles} "
            f"inflated_cells={inflated_count} resolution={msg.info.resolution:.2f} "
            f"frame={msg.header.frame_id}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainCostMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
