#!/usr/bin/env python3
"""LunaBot V4 Phase I: terrain-aware autonomous path follower.

The follower consumes the Phase H terrain-aware path and sends conservative
commands through the approved Phase B `/cmd_vel_in` boundary. It is the first
phase that integrates the terrain-aware plan with rover motion. The existing
controller/watchdog remains responsible for clamping, acceleration limiting,
and safe `/cmd_vel` output.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros


def angle_error(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TerrainPathFollower(Node):
    """Follow the terrain-aware path while preserving the controller boundary."""

    def __init__(self) -> None:
        super().__init__("lunabot_terrain_path_follower")
        self.declare_parameter("path_topic", "/lunabot/terrain/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("odom_topic", "/lunabot/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_in")
        self.declare_parameter(
            "status_topic", "/lunabot/autonomy/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis")
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.60)
        self.declare_parameter("lookahead_cells", 5)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("scan_topic", "/lunabot/lidar/scan")
        # Stop before the rover reaches an obstacle. The Phase L detector
        # observes a wider forward sector (0.60 rad); using a narrower sector
        # allowed returns at bearing about 0.48 rad to pass through to the
        # controller until physical contact.
        self.declare_parameter("front_obstacle_distance", 0.42)
        self.declare_parameter("front_obstacle_half_angle", 0.60)

        get = self.get_parameter
        self.path_topic = str(get("path_topic").value)
        self.goal_topic = str(get("goal_topic").value)
        self.odom_topic = str(get("odom_topic").value)
        self.cmd_topic = str(get("cmd_topic").value)
        self.status_topic = str(get("status_topic").value)
        self.map_frame = str(get("map_frame").value)
        self.base_frame = str(get("base_frame").value)
        self.goal_tolerance = max(0.05, float(get("goal_tolerance").value))
        self.max_linear = max(0.02, float(get("max_linear").value))
        self.max_angular = max(0.1, float(get("max_angular").value))
        self.lookahead_cells = max(0, int(get("lookahead_cells").value))

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self._path_callback, path_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, str(get("scan_topic").value), self._scan_callback,
            qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, self.status_topic, status_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path: list[tuple[float, float]] = []
        self.goal: Optional[PoseStamped] = None
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None
        # Hold the chosen avoidance direction while the rover clears an
        # obstacle. Recomputing it every scan made left/right clearance noise
        # alternate the command and trapped the rover in place.
        self.avoid_turn: Optional[float] = None
        self.obstacle_clear_ticks = 0
        # After the front sector clears, keep a short forward escape arc.
        # Releasing avoidance immediately after an in-place turn lets the
        # path controller point back at the same obstacle and repeat forever.
        self.avoid_escape_ticks = 0
        self.reached = False
        self.last_waypoint = -1
        self.last_status = ""
        self.publish_status("INTEGRATION_WAITING_FOR_TERRAIN_PLAN")
        self.timer = self.create_timer(
            1.0 / max(1.0, float(get("control_rate").value)), self._tick)

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.publish_status("INTEGRATION_EMPTY_TERRAIN_PLAN")
            return
        self.path = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        # The terrain planner republishes its latched/replanned path. Do not
        # turn a completed goal back into an active one just because an
        # unchanged path arrived on the next planning tick.
        was_reached = self.reached
        # Preserve progress across replans. Resetting to -1 on every new path
        # forced the follower back to waypoint 2/3 and made it orbit the same
        # region while the planner was correctly producing new paths.
        if not was_reached:
            self.publish_status(
                f"INTEGRATION_PATH_RECEIVED cells={len(self.path)}")

    @staticmethod
    def _same_goal(first: PoseStamped, second: PoseStamped) -> bool:
        if (first.header.frame_id or "") != (second.header.frame_id or ""):
            return False
        a, b = first.pose, second.pose
        return (
            abs(a.position.x - b.position.x) < 1e-6 and
            abs(a.position.y - b.position.y) < 1e-6 and
            abs(a.position.z - b.position.z) < 1e-6 and
            abs(a.orientation.x - b.orientation.x) < 1e-6 and
            abs(a.orientation.y - b.orientation.y) < 1e-6 and
            abs(a.orientation.z - b.orientation.z) < 1e-6 and
            abs(a.orientation.w - b.orientation.w) < 1e-6
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        # A* republishes its active goal for late observers. Treat an identical
        # message as state, not as a new command, so it cannot reset progress.
        if self.goal is not None and self._same_goal(msg, self.goal):
            return
        self.goal = msg
        self.reached = False
        self.last_waypoint = -1
        self.publish_status(
            f"INTEGRATION_GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def _front_obstacle(self):
        if self.scan is None:
            return None
        limit = float(self.get_parameter("front_obstacle_distance").value)
        half_angle = float(self.get_parameter("front_obstacle_half_angle").value)
        front, left, right = [], [], []
        for index, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance):
                continue
            angle = self.scan.angle_min + index * self.scan.angle_increment
            if distance < self.scan.range_min or distance > limit:
                continue
            if abs(angle) <= half_angle:
                front.append((distance, angle))
            elif half_angle < angle <= 1.0:
                left.append(distance)
            elif -1.0 <= angle < -half_angle:
                right.append(distance)
        if not front:
            return None
        # First move away from the closest return's bearing. A return at a
        # positive bearing is on the rover's left, so the safe turn is right;
        # a negative bearing requires a left turn. This prevents the clearance
        # heuristic from choosing a side that is technically open farther away
        # while steering into the nearest face of a paired obstacle.
        closest_distance, closest_angle = min(front, key=lambda item: item[0])
        if closest_angle > 0.15:
            turn = -1.0
        elif closest_angle < -0.15:
            turn = 1.0
        else:
            # Turn toward the side with more measured clearance. In ROS,
            # positive angular.z rotates counter-clockwise, to the left.
            left_clear = min(left) if left else self.scan.range_max
            right_clear = min(right) if right else self.scan.range_max
            turn = 1.0 if left_clear > right_clear else -1.0
        return turn, closest_distance

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * q.w * q.z,
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _pose(self):
        if self.odom is not None:
            pose = self.odom.pose.pose
            x, y, yaw = pose.position.x, pose.position.y, self._yaw(pose.orientation)
            source = self.odom.header.frame_id or "odom"
        else:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
            t = transform.transform.translation
            return t.x, t.y, self._yaw(transform.transform.rotation)
        if source == self.map_frame:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, angle_error(yaw + tf_yaw)

    def _goal_map(self):
        if self.goal is None:
            return None
        pose = self.goal.pose
        yaw = self._yaw(pose.orientation)
        source = self.goal.header.frame_id or self.map_frame
        if source == self.map_frame:
            return pose.position.x, pose.position.y, yaw
        transform = self.tf_buffer.lookup_transform(self.map_frame, source, Time())
        t = transform.transform.translation
        tf_yaw = self._yaw(transform.transform.rotation)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        return t.x + c * pose.position.x - s * pose.position.y, \
            t.y + s * pose.position.x + c * pose.position.y, \
            angle_error(yaw + tf_yaw)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if not self.path:
            self._publish_stop()
            return
        try:
            current_x, current_y, current_yaw = self._pose()
            goal = self._goal_map()
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f"INTEGRATION_WAITING_FOR_TF {exc}")
            self._publish_stop()
            return
        if goal is None:
            self.publish_status("INTEGRATION_WAITING_FOR_GOAL")
            self._publish_stop()
            return
        # Once the integrated follower has reached the active goal, hold a
        # safe stop until a genuinely new goal arrives. SLAM map->odom updates
        # can move the transformed pose by a few centimeters on later ticks;
        # without this latch the follower could resume motion after reporting
        # completion.
        if self.reached:
            self._publish_stop()
            return
        obstacle = self._front_obstacle()
        if obstacle is not None:
            suggested_turn, distance = obstacle
            if self.avoid_turn is None:
                self.avoid_turn = suggested_turn
                self.avoid_escape_ticks = 0
            self.obstacle_clear_ticks = 0
            cmd = Twist()
            cmd.angular.z = self.avoid_turn * self.max_angular
            # Never push forward into a close LiDAR return. Keep one turn
            # direction until the obstacle is actually cleared; choosing from
            # noisy left/right scans on every tick causes oscillation.
            self.cmd_pub.publish(cmd)
            self.publish_status(
                f"INTEGRATION_OBSTACLE_AVOID distance={distance:.2f} "
                f"turn={self.avoid_turn:+.0f}")
            return
        if self.avoid_turn is not None:
            # Require several consecutive clear scans before handing control
            # back to the map-frame path. This prevents a single dropped scan
            # from sending the rover straight back into the obstacle.
            self.obstacle_clear_ticks += 1
            if self.obstacle_clear_ticks < 8:
                cmd = Twist()
                cmd.angular.z = self.avoid_turn * self.max_angular
                self.cmd_pub.publish(cmd)
                self.publish_status(
                    f"INTEGRATION_OBSTACLE_CLEARING turn={self.avoid_turn:+.0f} "
                    f"clear_ticks={self.obstacle_clear_ticks}")
                return
            # Do not hand control back to pure path pursuit while the rover is
            # still beside the obstacle. Drive a bounded arc toward the side
            # selected by the clearance test so the rover makes real lateral
            # progress instead of rotating back to the same blocking cell.
            if self.avoid_escape_ticks < 18:
                self.avoid_escape_ticks += 1
                cmd = Twist()
                cmd.linear.x = min(self.max_linear * 0.5, 0.10)
                cmd.angular.z = self.avoid_turn * self.max_angular * 0.65
                self.cmd_pub.publish(cmd)
                self.publish_status(
                    f"INTEGRATION_OBSTACLE_ESCAPE turn={self.avoid_turn:+.0f} "
                    f"arc_ticks={self.avoid_escape_ticks}")
                return
            self.avoid_turn = None
            self.obstacle_clear_ticks = 0
            self.avoid_escape_ticks = 0
        # The terrain planner resolves a requested goal that falls on an
        # occupied/inflated cell to its nearest traversable cell. Use the
        # endpoint of the accepted terrain path for completion as well as for
        # following; otherwise the follower can correctly reach the planner's
        # safe endpoint but continue trying to drive into the blocked raw goal.
        path_goal_x, path_goal_y = self.path[-1]
        path_distance = math.hypot(path_goal_x - current_x,
                                   path_goal_y - current_y)
        requested_distance = math.hypot(goal[0] - current_x,
                                        goal[1] - current_y)
        if path_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                if requested_distance <= self.goal_tolerance:
                    self.publish_status(
                        f"INTEGRATION_GOAL_REACHED distance={requested_distance:.2f}")
                else:
                    # A planner fallback endpoint is not the requested goal.
                    # Stop safely, but never report a disconnected goal as
                    # reached; this keeps launcher aggregation honest.
                    self.publish_status(
                        f"INTEGRATION_FALLBACK_REACHED requested_distance={requested_distance:.2f} "
                        f"fallback_distance={path_distance:.2f}")
            return
        nearest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - current_x,
                                               self.path[i][1] - current_y))
        target_index = min(len(self.path) - 1, nearest + self.lookahead_cells)
        target_x, target_y = self.path[target_index]
        heading = math.atan2(target_y - current_y, target_x - current_x)
        error = angle_error(heading - current_yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.4 * error))
        if abs(error) <= 0.9:
            cmd.linear.x = min(self.max_linear, 0.45 * path_distance)
            cmd.linear.x *= max(0.2, math.cos(error))
        self.cmd_pub.publish(cmd)
        if target_index != self.last_waypoint:
            self.last_waypoint = target_index
            self.publish_status(
                f"INTEGRATION_FOLLOWING waypoint={target_index}/{len(self.path)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
