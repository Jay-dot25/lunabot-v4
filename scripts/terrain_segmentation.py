#!/usr/bin/env python3
"""LunaBot V4 Phase E: lightweight RGB-D terrain segmentation.

This node deliberately stays at the perception layer. It consumes the existing
Gazebo RGB and depth camera topics and publishes a pixel-wise mask plus a
visual overlay; it does not modify the map, planner, controller, or rover
model. The first implementation is deterministic and dependency-light so it
can run on the validated ROS 2 Humble workstation without cv_bridge or a
machine-learning model.

Mask labels are encoded as mono8 values:
  0 = unknown / invalid depth
  1 = traversable terrain candidate
  2 = obstacle / non-ground candidate

The lower camera field with valid depth is treated as terrain candidate, while
valid returns above the configurable ground ROI are treated as obstacle
candidate. Saturated RGB pixels in the ground ROI are conservatively marked as
obstacles. This is an auditable baseline for Phase E; semantic terrain
mapping and cost-map fusion are later phases.
"""

from __future__ import annotations

import math
import struct
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


UNKNOWN = 0
TERRAIN = 1
OBSTACLE = 2


class TerrainSegmentation(Node):
    """Segment synchronized RGB-D frames into auditable terrain classes."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_segmentation")
        self.declare_parameter("image_topic", "/lunabot/camera/image_raw")
        self.declare_parameter("depth_topic", "/lunabot/depth/image_raw")
        self.declare_parameter("mask_topic", "/lunabot/terrain/segmentation")
        self.declare_parameter("overlay_topic", "/lunabot/terrain/overlay")
        self.declare_parameter(
            "status_topic", "/lunabot/terrain/segmentation/status")
        self.declare_parameter("ground_roi_start", 0.35)
        self.declare_parameter("max_depth", 8.0)
        self.declare_parameter("max_rate", 5.0)
        self.declare_parameter("saturation_threshold", 0.55)
        self.declare_parameter("horizontal_fov", 1.047)
        self.declare_parameter("camera_height", 1.24)
        self.declare_parameter("obstacle_height_threshold", 0.35)
        self.declare_parameter("vertical_step_threshold", 0.03)
        self.declare_parameter("enable_geometric_obstacles", False)

        get = self.get_parameter
        self.image_topic = str(get("image_topic").value)
        self.depth_topic = str(get("depth_topic").value)
        self.mask_topic = str(get("mask_topic").value)
        self.overlay_topic = str(get("overlay_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.ground_roi_start = max(0.0, min(1.0, float(get("ground_roi_start").value)))
        self.max_depth = max(0.1, float(get("max_depth").value))
        self.max_rate = max(0.1, float(get("max_rate").value))
        self.saturation_threshold = max(
            0.0, min(1.0, float(get("saturation_threshold").value)))
        self.horizontal_fov = max(0.2, float(get("horizontal_fov").value))
        self.camera_height = max(0.1, float(get("camera_height").value))
        self.obstacle_height_threshold = max(
            0.05, float(get("obstacle_height_threshold").value))
        self.vertical_step_threshold = max(
            0.005, float(get("vertical_step_threshold").value))
        self.enable_geometric_obstacles = bool(
            get("enable_geometric_obstacles").value)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self.mask_pub = self.create_publisher(Image, self.mask_topic, image_qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, image_qos)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)

        self.latest_image: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.last_stamp_ns = 0
        self.last_process_ns = 0
        self.frame_count = 0
        self.last_status = ""
        self.timer = self.create_timer(0.05, self._process_latest)
        self.publish_status("SEGMENTATION_WAITING_FOR_RGB_D")

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _image_callback(self, msg: Image) -> None:
        self.latest_image = msg

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
    def _encoding_lower(msg: Image) -> str:
        return (msg.encoding or "").lower().replace(" ", "")

    def _rgb_at(self, msg: Image, x: int, y: int) -> tuple[int, int, int]:
        encoding = self._encoding_lower(msg)
        channels = 4 if encoding in ("rgba8", "bgra8") else 3
        if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            return 128, 128, 128
        offset = y * msg.step + x * channels
        if offset + channels > len(msg.data):
            return 128, 128, 128
        values = msg.data[offset:offset + channels]
        if encoding.startswith("bgr"):
            return int(values[2]), int(values[1]), int(values[0])
        return int(values[0]), int(values[1]), int(values[2])

    def _depth_at(self, msg: Image, x: int, y: int) -> float:
        encoding = self._encoding_lower(msg)
        if msg.width <= 0 or msg.height <= 0:
            return math.nan
        offset = y * msg.step
        try:
            if encoding in ("16uc1", "mono16"):
                offset += x * 2
                raw = struct.unpack_from(
                    ">H" if msg.is_bigendian else "<H", msg.data, offset)[0]
                return raw / 1000.0
            if encoding in ("32fc1", "32fc"):
                offset += x * 4
                return float(struct.unpack_from(
                    ">f" if msg.is_bigendian else "<f", msg.data, offset)[0])
            if encoding in ("64fc1", "64fc"):
                offset += x * 8
                return float(struct.unpack_from(
                    ">d" if msg.is_bigendian else "<d", msg.data, offset)[0])
        except (IndexError, struct.error):
            return math.nan
        return math.nan

    @staticmethod
    def _make_image(header, width: int, height: int, encoding: str,
                    step: int, data: bytes) -> Image:
        msg = Image()
        msg.header = header
        msg.height = height
        msg.width = width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data
        return msg

    def _is_geometric_obstacle(self, depth: Image, depth_x: int, depth_y: int,
                               distance: float, fy: float, cy: float) -> bool:
        """Detect vertical 3D obstacles (such as grey rocks/walls) from depth geometry."""
        if distance > 3.5 or depth.height < 16:
            return False
        height_above_ground = self.camera_height - distance * ((depth_y - cy) / max(1.0, fy))
        if height_above_ground < self.obstacle_height_threshold:
            return False
        step_y = depth_y - 8 if depth_y >= 8 else min(depth.height - 1, depth_y + 8)
        neighbor_dist = self._depth_at(depth, depth_x, step_y)
        if not math.isfinite(neighbor_dist):
            return False
        return abs(neighbor_dist - distance) <= self.vertical_step_threshold

    def _process_latest(self) -> None:
        image = self.latest_image
        depth = self.latest_depth
        if image is None or depth is None or image.width <= 0 or image.height <= 0:
            return
        stamp_ns = max(self._stamp_ns(image), self._stamp_ns(depth))
        now_ns = self.get_clock().now().nanoseconds
        if self.last_process_ns and 0 < (now_ns - self.last_process_ns) < int(1e9 / self.max_rate):
            return
        self.last_stamp_ns = stamp_ns
        self.last_process_ns = now_ns

        width, height = int(image.width), int(image.height)
        fy = max(1.0, depth.width / (2.0 * math.tan(self.horizontal_fov / 2.0)))
        cy = (depth.height - 1) / 2.0
        mask = bytearray(width * height)
        overlay = bytearray(width * height * 3)
        terrain_count = 0
        obstacle_count = 0
        valid_count = 0
        for y in range(height):
            depth_y = min(depth.height - 1, int(y * depth.height / height))
            lower_roi = y / max(1, height - 1) >= self.ground_roi_start
            for x in range(width):
                depth_x = min(depth.width - 1, int(x * depth.width / width))
                distance = self._depth_at(depth, depth_x, depth_y)
                valid = math.isfinite(distance) and 0.05 <= distance <= self.max_depth
                label = UNKNOWN
                if valid:
                    valid_count += 1
                    red, green, blue = self._rgb_at(image, x, y)
                    luminance = max(red, green, blue) / 255.0
                    saturation = (max(red, green, blue) - min(red, green, blue)) / 255.0
                    geom_obstacle = (
                        self.enable_geometric_obstacles and
                        self._is_geometric_obstacle(
                            depth, depth_x, depth_y, distance, fy, cy)
                    )
                    # Lunar terrain is expected to be a low-saturation return in
                    # the lower field without steep vertical obstacle geometry.
                    if (lower_roi and not geom_obstacle and
                            saturation <= self.saturation_threshold and luminance < 0.98):
                        label = TERRAIN
                        terrain_count += 1
                    else:
                        label = OBSTACLE
                        obstacle_count += 1
                mask[y * width + x] = label
                index = (y * width + x) * 3
                if label == TERRAIN:
                    if luminance < 0.08:
                        # Deep South-Pole Shadow (Umbra)
                        overlay[index:index + 3] = bytes((145, 85, 235))
                    elif luminance < 0.21:
                        # Steep Crater Wall / Penumbra
                        overlay[index:index + 3] = bytes((245, 115, 35))
                    elif (red - blue) >= 7 and luminance <= 0.48:
                        # Exposed Warm-Slate Bedrock
                        overlay[index:index + 3] = bytes((235, 175, 40))
                    else:
                        # Traversable Regolith
                        overlay[index:index + 3] = bytes((40, 210, 60))
                elif label == OBSTACLE:
                    overlay[index:index + 3] = bytes((235, 55, 45))
                else:
                    overlay[index:index + 3] = bytes((20, 20, 20))

        header = image.header
        self.mask_pub.publish(self._make_image(
            header, width, height, "mono8", width, bytes(mask)))
        self.overlay_pub.publish(self._make_image(
            header, width, height, "rgb8", width * 3, bytes(overlay)))
        self.frame_count += 1
        unknown_count = width * height - valid_count
        self.publish_status(
            f"SEGMENTATION_PASS frames={self.frame_count} "
            f"width={width} height={height} valid={valid_count} "
            f"terrain={terrain_count} obstacle={obstacle_count} "
            f"unknown={unknown_count}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainSegmentation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
