#!/usr/bin/env python3
"""LunaBot V4: real LiDAR obstacle detection for the final demonstration.

The node observes the bridged LaserScan, marks close returns in front of the
rover, and publishes both a visual marker and a map-frame obstacle overlay.
It is observation/perception only: it never publishes velocity or goals.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf2_ros


class ObstacleDetector(Node):
    """Detect close forward LiDAR returns and publish auditable evidence."""

    def __init__(self) -> None:
        super().__init__("lunabot_obstacle_detector")
        self.declare_parameter("scan_topic", "/lunabot/lidar/scan")
        self.declare_parameter("status_topic", "/lunabot/obstacles/status")
        self.declare_parameter("marker_topic", "/lunabot/obstacles/markers")
        self.declare_parameter("map_topic", "/lunabot/obstacles/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("front_half_angle", 0.60)
        self.declare_parameter("detection_distance", 1.50)
        self.declare_parameter("minimum_distance", 0.25)
        self.declare_parameter("map_resolution", 0.10)
        self.declare_parameter("map_width", 160)
        self.declare_parameter("map_height", 160)
        self.declare_parameter("map_origin_x", -8.0)
        self.declare_parameter("map_origin_y", -8.0)

        get = self.get_parameter
        self.scan_topic = str(get("scan_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.marker_topic = str(get("marker_topic").value)
        self.map_topic = str(get("map_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.front_half_angle = max(0.1, float(get("front_half_angle").value))
        self.detection_distance = max(0.3, float(get("detection_distance").value))
        self.minimum_distance = max(0.05, float(get("minimum_distance").value))
        self.map_resolution = max(0.02, float(get("map_resolution").value))
        self.map_width = max(10, int(get("map_width").value))
        self.map_height = max(10, int(get("map_height").value))
        self.map_origin_x = float(get("map_origin_x").value)
        self.map_origin_y = float(get("map_origin_y").value)

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback,
            qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)
        marker_qos = QoSProfile(depth=5)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.VOLATILE
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, marker_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, status_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_status = ""
        self.last_scan: Optional[LaserScan] = None
        self.last_points: list[tuple[float, float, float]] = []
        self.frame_count = 0
        self.publish_status("OBSTACLE_DETECTOR_WAITING_FOR_LIDAR")

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    @staticmethod
    def _yaw(transform: TransformStamped) -> float:
        q = transform.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _map_transform(self, frame_id: str) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, frame_id, Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None

    def _scan_callback(self, scan: LaserScan) -> None:
        self.frame_count += 1
        points: list[tuple[float, float, float]] = []
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > self.front_half_angle:
                continue
            if not math.isfinite(value):
                continue
            if value < self.minimum_distance or value > self.detection_distance:
                continue
            points.append((value * math.cos(angle), value * math.sin(angle), value))

        self.last_scan = scan
        self.last_points = points
        if points:
            closest = min(points, key=lambda point: point[2])
            self.publish_status(
                f"OBSTACLE_DETECTED distance={closest[2]:.2f} "
                f"bearing={math.atan2(closest[1], closest[0]):.2f} "
                f"returns={len(points)} source=lidar frames={self.frame_count}")
        else:
            self.publish_status(
                f"OBSTACLE_CLEAR returns=0 source=lidar frames={self.frame_count}")
        self._publish_marker(scan, points)
        self._publish_map(scan, points)

    def _publish_marker(self, scan: LaserScan,
                        points: list[tuple[float, float, float]]) -> None:
        marker = Marker()
        marker.header = scan.header
        marker.ns = "lunabot_obstacles"
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.05
        marker.color.a = 0.95
        for x, y, _ in points:
            point = Point()
            point.x, point.y, point.z = x, y, 0.0
            marker.points.append(point)
        self.marker_pub.publish(marker)

    def _publish_map(self, scan: LaserScan,
                     points: list[tuple[float, float, float]]) -> None:
        transform = self._map_transform(scan.header.frame_id)
        if transform is None:
            return
        output = OccupancyGrid()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        output.info.resolution = self.map_resolution
        output.info.width = self.map_width
        output.info.height = self.map_height
        output.info.origin.position.x = self.map_origin_x
        output.info.origin.position.y = self.map_origin_y
        output.info.origin.orientation.w = 1.0
        output.data = [-1] * (self.map_width * self.map_height)
        yaw = self._yaw(transform)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        for x, y, _ in points:
            map_x = tx + math.cos(yaw) * x - math.sin(yaw) * y
            map_y = ty + math.sin(yaw) * x + math.cos(yaw) * y
            cell_x = int((map_x - self.map_origin_x) / self.map_resolution)
            cell_y = int((map_y - self.map_origin_y) / self.map_resolution)
            if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                output.data[cell_y * self.map_width + cell_x] = 100
        self.map_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleDetector()
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
