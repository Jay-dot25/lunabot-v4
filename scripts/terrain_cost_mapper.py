#!/usr/bin/env python3
"""LunaBot V4 Phase G: 5-class semantic terrain cost-map generation.

Converts the Phase F 5-class semantic grid into graded traversability costs
per master directive:

  BEDROCK  (10)  ->  5   safe
  REGOLITH (30)  -> 15   moderate
  SHADOW   (50)  -> 45   less desirable / caution
  ROCK     (70)  -> 70   hazardous
  CRATER   (100) -> 100  hazardous / non-traversable

Unknown/unobserved cells receive conservative treatment (80).

Adds inflation around high-cost terrain (ROCK, CRATER).

Publishes:
  /lunabot/terrain/cost_map
  /lunabot/terrain/cost_map/status

No velocity or localization output.
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

# Semantic grid values from Phase F
UNKNOWN_GRID = -1
BEDROCK_GRID = 10
REGOLITH_GRID = 30
SHADOW_GRID = 50
ROCK_GRID = 70
CRATER_GRID = 100

# Cost values per directive (graded)
BEDROCK_COST = 5
REGOLITH_COST = 15
SHADOW_COST = 45
ROCK_COST = 70
CRATER_COST = 100
UNKNOWN_COST = 80

# Mapping semantic -> cost
SEMANTIC_TO_COST = {
    BEDROCK_GRID: BEDROCK_COST,
    REGOLITH_GRID: REGOLITH_COST,
    SHADOW_GRID: SHADOW_COST,
    ROCK_GRID: ROCK_COST,
    CRATER_GRID: CRATER_COST,
}


class TerrainCostMapper(Node):
    def __init__(self) -> None:
        super().__init__("lunabot_terrain_cost_mapper")
        self.declare_parameter("semantic_topic", "/lunabot/terrain/semantic_map")
        self.declare_parameter("obstacle_topic", "/lunabot/obstacles/map")
        self.declare_parameter("cost_topic", "/lunabot/terrain/cost_map")
        self.declare_parameter("status_topic", "/lunabot/terrain/cost_map/status")
        self.declare_parameter("terrain_cost", BEDROCK_COST)  # kept for compat
        self.declare_parameter("unknown_cost", UNKNOWN_COST)
        self.declare_parameter("obstacle_cost", CRATER_COST)
        self.declare_parameter("inflation_radius", 0.35)

        get = self.get_parameter
        self.semantic_topic = str(get("semantic_topic").value)
        self.obstacle_topic = str(get("obstacle_topic").value)
        self.cost_topic = str(get("cost_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.terrain_cost = max(0, min(100, int(get("terrain_cost").value)))
        self.unknown_cost = max(0, min(100, int(get("unknown_cost").value)))
        self.obstacle_cost = max(0, min(100, int(get("obstacle_cost").value)))
        self.inflation_radius = max(0.0, float(get("inflation_radius").value))

        input_qos = QoSProfile(depth=1)
        input_qos.reliability = ReliabilityPolicy.RELIABLE
        input_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.semantic_sub = self.create_subscription(
            OccupancyGrid, self.semantic_topic, self._semantic_callback, input_qos)
        self.obstacle_sub = self.create_subscription(
            OccupancyGrid, self.obstacle_topic, self._obstacle_callback, input_qos)

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cost_pub = self.create_publisher(OccupancyGrid, self.cost_topic, output_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, output_qos)

        self.frame_count = 0
        self.last_status = ""
        self.last_cost_map: Optional[OccupancyGrid] = None
        self.last_semantic_map: Optional[OccupancyGrid] = None
        self.last_obstacle_map: Optional[OccupancyGrid] = None
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
        # New 5-class mapping
        if value in SEMANTIC_TO_COST:
            return SEMANTIC_TO_COST[value]
        # Backward compat: old 25 -> bedrock, 100 -> crater/rock
        if value == 25:
            return BEDROCK_COST
        if value >= 90:
            return CRATER_COST
        if value > 0:
            return REGOLITH_COST
        return UNKNOWN_COST

    def _inflate(self, costs: list[int], width: int, height: int, resolution: float) -> tuple[int, int]:
        radius_cells = int(math.ceil(self.inflation_radius / max(resolution, 1e-6)))
        if radius_cells <= 0:
            return 0, 0
        # Inflate around ROCK and CRATER (cost >= ROCK_COST)
        obstacles = [i for i, v in enumerate(costs) if v >= ROCK_COST]
        inflated = 0
        for index in obstacles:
            ox, oy = index % width, index // width
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    dist = math.hypot(dx, dy)
                    if dist > radius_cells:
                        continue
                    x, y = ox + dx, oy + dy
                    if x < 0 or y < 0 or x >= width or y >= height:
                        continue
                    target = y * width + x
                    if target == index:
                        continue
                    # Halo cost decreases with distance, never overwrites higher
                    # ROCK_COST=70, CRATER=100 -> halo 45-70 range
                    halo = max(SHADOW_COST, int(ROCK_COST + (CRATER_COST - ROCK_COST) * 0.5 - 25.0 * dist / radius_cells))
                    # Ensure halo at least SHADOW_COST and less than obstacle
                    halo = max(SHADOW_COST, min(ROCK_COST, halo))
                    if halo > costs[target]:
                        costs[target] = halo
                        inflated += 1
        return len(obstacles), inflated

    def _obstacle_callback(self, msg: OccupancyGrid) -> None:
        self.last_obstacle_map = msg
        if self.last_semantic_map is not None:
            self._publish_cost(self.last_semantic_map)

    def _semantic_callback(self, msg: OccupancyGrid) -> None:
        self.last_semantic_map = msg
        self._publish_cost(msg)

    def _publish_cost(self, msg: OccupancyGrid) -> None:
        width, height = int(msg.info.width), int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) < width * height:
            self.publish_status("COST_MAP_INVALID_SEMANTIC_MAP")
            return

        costs = [self._base_cost(int(v)) for v in msg.data[:width * height]]

        sensed_obstacles = 0
        overlay = self.last_obstacle_map
        if overlay is not None:
            same_geometry = (
                int(overlay.info.width) == width and
                int(overlay.info.height) == height and
                abs(float(overlay.info.resolution) - float(msg.info.resolution)) < 1e-6 and
                abs(float(overlay.info.origin.position.x) - float(msg.info.origin.position.x)) < 1e-6 and
                abs(float(overlay.info.origin.position.y) - float(msg.info.origin.position.y)) < 1e-6 and
                len(overlay.data) >= width * height
            )
            if same_geometry:
                for idx, val in enumerate(overlay.data[:width * height]):
                    if int(val) >= 90:  # obstacle detected
                        if costs[idx] < CRATER_COST:
                            sensed_obstacles += 1
                        costs[idx] = CRATER_COST

        obstacle_count, inflated_count = self._inflate(costs, width, height, float(msg.info.resolution))

        output = OccupancyGrid()
        output.header = msg.header
        output.header.stamp = self.get_clock().now().to_msg()
        output.info = msg.info
        output.data = costs

        self.last_cost_map = output
        self.cost_pub.publish(output)
        self.frame_count += 1

        # Counts per cost
        bedrock_cells = sum(v == BEDROCK_COST for v in costs)
        regolith_cells = sum(v == REGOLITH_COST for v in costs)
        shadow_cells = sum(v == SHADOW_COST for v in costs)
        rock_cells = sum(v == ROCK_COST for v in costs)
        crater_cells = sum(v == CRATER_COST for v in costs)
        unknown_cells = sum(v == UNKNOWN_COST for v in costs)

        self.publish_status(
            f"COST_MAP_PASS frames={self.frame_count} width={width} height={height} "
            f"bedrock={bedrock_cells} regolith={regolith_cells} shadow={shadow_cells} "
            f"rock={rock_cells} crater={crater_cells} unknown={unknown_cells} "
            f"obstacle_cells={obstacle_count} sensed_obstacles={sensed_obstacles} "
            f"inflated_cells={inflated_count} resolution={msg.info.resolution:.2f} "
            f"frame={msg.header.frame_id} costs=BEDROCK=5,REGOLITH=15,SHADOW=45,ROCK=70,CRATER=100"
        )


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
