#!/usr/bin/env python3
"""LunaBot V4 Phase F: semantic terrain map fusion.

The node fuses the Phase E RGB-D segmentation mask with depth and the
approved map->chassis localization transform. It accumulates observations in a
fixed map-frame grid and publishes a single auditable OccupancyGrid:

  -1 = unknown
   25 = terrain candidate
  100 = obstacle candidate

This is a semantic terrain map, not a second localization or SLAM system. It
uses the existing slam_toolbox TF and does not publish velocity or modify the
planner/controller path.
"""

from __future__ import annotations

import math
import struct
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros


UNKNOWN = 0
TERRAIN = 1
OBSTACLE = 2


class SemanticTerrainMapper(Node):
    """Accumulate RGB-D segmentation labels in the existing map frame."""

    def __init__(self) -> None:
        super().__init__("lunabot_semantic_terrain_mapper")
        self.declare_parameter("mask_topic", "/lunabot/terrain/segmentation")
        self.declare_parameter("depth_topic", "/lunabot/depth/image_raw")
        self.declare_parameter("map_topic", "/lunabot/terrain/semantic_map")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/semantic_map/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("width", 160)
        self.declare_parameter("height", 160)
        self.declare_parameter("origin_x", -8.0)
        self.declare_parameter("origin_y", -8.0)
        self.declare_parameter("horizontal_fov", 1.047)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("sample_stride", 8)
        self.declare_parameter("update_rate", 2.0)

        get = self.get_parameter
        self.mask_topic = str(get("mask_topic").value)
        self.depth_topic = str(get("depth_topic").value)
        self.map_topic = str(get("map_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.resolution = max(0.02, float(get("resolution").value))
        self.width = max(10, int(get("width").value))
        self.height = max(10, int(get("height").value))
        self.origin_x = float(get("origin_x").value)
        self.origin_y = float(get("origin_y").value)
        self.horizontal_fov = max(0.2, float(get("horizontal_fov").value))
        self.max_depth = max(0.5, float(get("max_depth").value))
        self.sample_stride = max(1, int(get("sample_stride").value))
        self.update_rate = max(0.2, float(get("update_rate").value))

        self.mask_sub = self.create_subscription(
            Image, self.mask_topic, self._mask_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_mask: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.last_mask_stamp = 0
        self.last_process_ns = 0
        self.frame_count = 0
        self.observation_count = 0
        self.last_status = ""
        self.cells = [UNKNOWN] * (self.width * self.height)
        self.timer = self.create_timer(1.0 / self.update_rate, self._process_latest)
        self.publish_status("SEMANTIC_MAP_WAITING_FOR_SEGMENTATION")

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _mask_callback(self, msg: Image) -> None:
        self.latest_mask = msg

    def _depth_callback(self, msg: Image) -> None:
        self.latest_depth = msg

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _depth_at(msg: Image, x: int, y: int) -> float:
        encoding = (msg.encoding or "").lower().replace(" ", "")
        offset = y * msg.step
        try:
            if encoding in ("16uc1", "mono16"):
                raw = struct.unpack_from(
                    ">H" if msg.is_bigendian else "<H", msg.data, offset + x * 2)[0]
                return raw / 1000.0
            if encoding in ("32fc1", "32fc"):
                return float(struct.unpack_from(
                    ">f" if msg.is_bigendian else "<f", msg.data, offset + x * 4)[0])
            if encoding in ("64fc1", "64fc"):
                return float(struct.unpack_from(
                    ">d" if msg.is_bigendian else "<d", msg.data, offset + x * 8)[0])
        except (IndexError, struct.error):
            return math.nan
        return math.nan

    @staticmethod
    def _mask_at(msg: Image, x: int, y: int) -> int:
        if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
            return UNKNOWN
        offset = y * msg.step + x
        if offset >= len(msg.data):
            return UNKNOWN
        value = int(msg.data[offset])
        return value if value in (TERRAIN, OBSTACLE) else UNKNOWN

    def _lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException as exc:
            self.publish_status(f"SEMANTIC_MAP_WAITING_FOR_TF {exc}")
            return None
        translation = transform.transform.translation
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def _grid_index(self, x: float, y: float) -> Optional[int]:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gy * self.width + gx

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        # OccupancyGrid is used as a compact semantic grid: 25 is terrain,
        # 100 is obstacle, and -1 is not yet observed.
        msg.data = [-1 if value == UNKNOWN else 25 if value == TERRAIN else 100
                    for value in self.cells]
        self.map_pub.publish(msg)

    def _process_latest(self) -> None:
        mask = self.latest_mask
        depth = self.latest_depth
        if mask is None or depth is None or mask.width <= 0 or mask.height <= 0:
            return
        stamp_ns = self._stamp_ns(mask)
        if stamp_ns <= self.last_mask_stamp:
            return
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        self.last_mask_stamp = stamp_ns
        robot_x, robot_y, robot_yaw = pose
        fx = mask.width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        cx = (mask.width - 1) / 2.0
        samples = 0
        for y in range(0, mask.height, self.sample_stride):
            depth_y = min(depth.height - 1, int(y * depth.height / mask.height))
            for x in range(0, mask.width, self.sample_stride):
                label = self._mask_at(mask, x, y)
                if label == UNKNOWN:
                    continue
                depth_x = min(depth.width - 1, int(x * depth.width / mask.width))
                distance = self._depth_at(depth, depth_x, depth_y)
                if not math.isfinite(distance) or distance < 0.1 or distance > self.max_depth:
                    continue
                bearing = math.atan2((x - cx), fx)
                local_x = distance * math.cos(bearing)
                local_y = distance * math.sin(bearing)
                map_x = robot_x + math.cos(robot_yaw) * local_x - math.sin(robot_yaw) * local_y
                map_y = robot_y + math.sin(robot_yaw) * local_x + math.cos(robot_yaw) * local_y
                index = self._grid_index(map_x, map_y)
                if index is None:
                    continue
                # Obstacle evidence dominates terrain evidence in a cell.
                if label == OBSTACLE or self.cells[index] == UNKNOWN:
                    self.cells[index] = label
                samples += 1

        self.frame_count += 1
        self.observation_count += samples
        terrain_cells = sum(value == TERRAIN for value in self.cells)
        obstacle_cells = sum(value == OBSTACLE for value in self.cells)
        self._publish_map()
        self.publish_status(
            f"SEMANTIC_MAP_PASS frames={self.frame_count} "
            f"observations={self.observation_count} samples={samples} "
            f"terrain_cells={terrain_cells} obstacle_cells={obstacle_cells} "
            f"resolution={self.resolution:.2f} frame={self.map_frame}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticTerrainMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
