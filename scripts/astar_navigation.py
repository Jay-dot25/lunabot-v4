#!/usr/bin/env python3
"""LunaBot V4 Phase D: grid A* planner and conservative path follower.

The node intentionally uses the Phase C interfaces instead of Nav2: slam_toolbox
publishes /map and map->odom, DiffDrive publishes /lunabot/odom and odom->chassis,
and this node publishes safe planner input on /cmd_vel_in for the inherited
Phase B controller. Unknown/occupied cells are not traversed by default.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import tf2_ros


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def angle_error(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AStarNavigation(Node):
    """A* planner plus waypoint follower with an auditable ROS contract."""

    def __init__(self) -> None:
        super().__init__('lunabot_astar_navigation')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/lunabot/odom')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_topic', '/cmd_vel_in')
        self.declare_parameter('status_topic', '/lunabot/navigation/status')
        self.declare_parameter('auto_goal', False)
        self.declare_parameter('auto_goal_distance', 1.5)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('inflation_radius', 0.25)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 0.7)
        self.declare_parameter('map_frame', 'map')

        p = self.get_parameter
        self.map_topic = p('map_topic').value
        self.goal_topic = p('goal_topic').value
        self.odom_topic = p('odom_topic').value
        self.path_topic = p('path_topic').value
        self.cmd_topic = p('cmd_topic').value
        self.status_topic = p('status_topic').value
        self.auto_goal = bool(p('auto_goal').value)
        self.auto_goal_distance = float(p('auto_goal_distance').value)
        self.goal_tolerance = float(p('goal_tolerance').value)
        self.occupied_threshold = int(p('occupied_threshold').value)
        self.inflation_radius = float(p('inflation_radius').value)
        self.unknown_is_obstacle = bool(p('unknown_is_obstacle').value)
        self.replan_period = float(p('replan_period').value)
        self.max_linear = float(p('max_linear').value)
        self.max_angular = float(p('max_angular').value)
        self.map_frame = str(p('map_frame').value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.path_topic, latched_qos)
        # Goals are commands, not latched state. Republish the active goal from
        # the timer so diagnostics and RViz can observe it without relying on
        # a transient-local command publisher.
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_msg: Optional[OccupancyGrid] = None
        self.odom_msg: Optional[Odometry] = None
        self.goal_msg: Optional[PoseStamped] = None
        self.goal_sent = False
        self.path_points: list[tuple[float, float]] = []
        self.path_stamp = self.get_clock().now()
        self.last_plan = self.get_clock().now() - Duration(seconds=10.0)
        self.last_status = ''
        self.reached = False
        self.last_waypoint = -1
        self.timer = self.create_timer(1.0 / max(1.0, p('control_rate').value),
                                      self._tick)
        self.publish_status('PLANNER_WAITING_FOR_MAP')

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            self.publish_status('PLANNER_INVALID_MAP')
            return
        self.map_msg = msg
        self.path_stamp = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal_msg = msg
        self.goal_sent = True
        self.reached = False
        self.path_points = []
        self.publish_status(
            f'GOAL_RECEIVED frame={msg.header.frame_id or self.map_frame}')

    def _transform_pose(self, x: float, y: float, yaw: float,
                        source_frame: str, target_frame: str):
        source_frame = source_frame or target_frame
        if source_frame == target_frame:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time())
        t = transform.transform.translation
        tf_yaw = yaw_from_quaternion(transform.transform.rotation)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, angle_error(yaw + tf_yaw)

    def _current_map_pose(self):
        if self.odom_msg is None:
            raise RuntimeError('odom not received')
        pose = self.odom_msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        return self._transform_pose(
            pose.position.x, pose.position.y, yaw,
            self.odom_msg.header.frame_id or 'odom', self.map_frame)

    def _send_auto_goal(self, current) -> None:
        if not self.auto_goal or self.goal_sent:
            return
        x, y, yaw = current
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x + self.auto_goal_distance * math.cos(yaw)
        goal.pose.position.y = y + self.auto_goal_distance * math.sin(yaw)
        q = quaternion_from_yaw(yaw)
        goal.pose.orientation.x, goal.pose.orientation.y = q[0], q[1]
        goal.pose.orientation.z, goal.pose.orientation.w = q[2], q[3]
        self.goal_msg = goal
        self.goal_sent = True
        self.goal_pub.publish(goal)
        self.publish_status(
            f'AUTO_GOAL_SENT frame={self.map_frame} '
            f'x={goal.pose.position.x:.2f} y={goal.pose.position.y:.2f}')

    def _world_to_grid(self, x: float, y: float):
        info = self.map_msg.info
        origin = info.origin
        oyaw = yaw_from_quaternion(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        c, s = math.cos(oyaw), math.sin(oyaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(mx / info.resolution)), int(math.floor(my / info.resolution))

    def _grid_to_world(self, cell):
        info = self.map_msg.info
        origin = info.origin
        oyaw = yaw_from_quaternion(origin.orientation)
        mx = (cell[0] + 0.5) * info.resolution
        my = (cell[1] + 0.5) * info.resolution
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (origin.position.x + c * mx - s * my,
                origin.position.y + s * mx + c * my)

    def _cell_is_free(self, cell) -> bool:
        info = self.map_msg.info
        x, y = cell
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return False
        value = self.map_msg.data[y * info.width + x]
        if value < 0:
            return not self.unknown_is_obstacle
        return value < self.occupied_threshold

    def _safe_cell(self, cell):
        info = self.map_msg.info
        radius = max(0, int(math.ceil(self.inflation_radius / info.resolution)))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius and not self._cell_is_free((cell[0] + dx, cell[1] + dy)):
                    return False
        return True

    def _nearest_free(self, cell):
        if self._safe_cell(cell):
            return cell
        for radius in range(1, 15):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self._safe_cell(candidate):
                        return candidate
        return None

    def _astar(self, start, goal):
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                     (1, 1), (1, -1), (-1, 1), (-1, -1))
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._safe_cell(nxt):
                    continue
                step = math.sqrt(2.0) if dx and dy else 1.0
                tentative = g_score[current] + step
                if tentative >= g_score.get(nxt, float('inf')):
                    continue
                came_from[nxt] = current
                g_score[nxt] = tentative
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_set, (tentative + heuristic, nxt))
        return []

    def _plan(self, current):
        if self.map_msg is None or self.goal_msg is None:
            return False
        try:
            goal_pose = self.goal_msg.pose
            goal_yaw = yaw_from_quaternion(goal_pose.orientation)
            goal = self._transform_pose(
                goal_pose.position.x, goal_pose.position.y, goal_yaw,
                self.goal_msg.header.frame_id or self.map_frame, self.map_frame)
            start = current
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f'PLANNER_WAITING_FOR_TF {exc}')
            return False
        start_cell = self._nearest_free(self._world_to_grid(start[0], start[1]))
        goal_cell = self._nearest_free(self._world_to_grid(goal[0], goal[1]))
        if start_cell is None or goal_cell is None:
            self.publish_status('NO_SAFE_START_OR_GOAL_CELL')
            return False
        cells = self._astar(start_cell, goal_cell)
        if not cells:
            self.path_points = []
            self.publish_status('NO_PATH')
            return False
        self.path_points = [self._grid_to_world(cell) for cell in cells]
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(self.path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point
            if index + 1 < len(self.path_points):
                heading = math.atan2(self.path_points[index + 1][1] - point[1],
                                     self.path_points[index + 1][0] - point[0])
            else:
                heading = goal[2]
            q = quaternion_from_yaw(heading)
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.last_plan = self.get_clock().now()
        self.last_waypoint = -1
        self.publish_status(f'PLANNING_PASS cells={len(cells)}')
        return True

    def _follow(self, current):
        if not self.path_points or self.goal_msg is None:
            self._publish_stop()
            return
        goal_pose = self.goal_msg.pose
        goal_x, goal_y, _ = self._transform_pose(
            goal_pose.position.x, goal_pose.position.y,
            yaw_from_quaternion(goal_pose.orientation),
            self.goal_msg.header.frame_id or self.map_frame, self.map_frame)
        goal_distance = math.hypot(goal_x - current[0], goal_y - current[1])
        if goal_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(f'GOAL_REACHED distance={goal_distance:.2f}')
            return
        nearest = min(range(len(self.path_points)),
                      key=lambda i: math.hypot(self.path_points[i][0] - current[0],
                                               self.path_points[i][1] - current[1]))
        lookahead = min(len(self.path_points) - 1, nearest + 3)
        target = self.path_points[lookahead]
        heading = math.atan2(target[1] - current[1], target[0] - current[0])
        error = angle_error(heading - current[2])
        cmd = Twist()
        cmd.angular.z = clamp(2.2 * error, -self.max_angular, self.max_angular)
        if abs(error) > 0.9:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.max_linear, 0.5 * goal_distance)
            cmd.linear.x *= max(0.15, math.cos(error))
        self.cmd_pub.publish(cmd)
        if lookahead != self.last_waypoint:
            self.last_waypoint = lookahead
            self.publish_status(f'FOLLOWING waypoint={lookahead}/{len(self.path_points)}')

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if self.map_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_MAP')
            self._publish_stop()
            return
        if self.odom_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_ODOM')
            self._publish_stop()
            return
        try:
            current = self._current_map_pose()
            self._send_auto_goal(current)
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f'PLANNER_WAITING_FOR_TF {exc}')
            self._publish_stop()
            return
        if self.goal_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_GOAL')
            self._publish_stop()
            return
        if not self.reached:
            self.goal_pub.publish(self.goal_msg)
        now = self.get_clock().now()
        if (not self.path_points or
                (now - self.last_plan).nanoseconds / 1e9 >= self.replan_period):
            self._plan(current)
        self._follow(current)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
