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
from std_msgs.msg import Float64, String
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
        self.declare_parameter("goal_tolerance", 0.10)
        self.declare_parameter("max_linear", 0.28)
        self.declare_parameter("max_angular", 0.85)
        self.declare_parameter("lookahead_cells", 2)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("stuck_timeout", 4.5)
        self.declare_parameter("stuck_min_progress", 0.06)
        self.declare_parameter("recovery_duration", 1.2)
        self.declare_parameter("enable_stuck_recovery", True)
        self.declare_parameter("active_mast_gaze", True)
        self.declare_parameter("pre_move_360_scan", False)
        self.declare_parameter("pre_move_scan_ticks", 30)
        self.declare_parameter("obstacle_status_topic", "/lunabot/obstacles/status")

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
        self.stuck_timeout = max(1.0, float(get("stuck_timeout").value))
        self.stuck_min_progress = max(0.01, float(get("stuck_min_progress").value))
        self.recovery_duration = max(0.3, float(get("recovery_duration").value))
        self.enable_stuck_recovery = bool(get("enable_stuck_recovery").value)
        self.active_mast_gaze = bool(get("active_mast_gaze").value)
        self.pre_move_360_scan = bool(get("pre_move_360_scan").value)
        self.pre_move_scan_ticks = max(1, int(get("pre_move_scan_ticks").value))
        self.obstacle_status_topic = str(get("obstacle_status_topic").value)

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
        self.obstacle_sub = self.create_subscription(
            String, self.obstacle_status_topic, self._obstacle_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.mast_pan_pub = self.create_publisher(
            Float64, "/lunabot/mast/pan", 10)
        self.mast_tilt_pub = self.create_publisher(
            Float64, "/lunabot/mast/tilt", 10)
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
        self.reached = False
        self.last_waypoint = -1
        self.last_status = ""
        self._progress_anchor_x: Optional[float] = None
        self._progress_anchor_y: Optional[float] = None
        self._progress_anchor_ns: int = 0
        self._recovery_until_ns: int = 0
        self._last_angular_cmd: float = 0.0
        self._current_pan_rad: float = 0.0
        self._current_tilt_rad: float = 0.0
        self._sweep_phase: float = 0.0
        self._pre_move_ticks: int = 0
        self.camera_mode: str = "CORRIDOR_HORIZON_SCAN"
        self.nearest_obstacle_dist: float = float("inf")
        self.nearest_obstacle_bearing: float = 0.0
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
        self.last_waypoint = -1
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

    def _obstacle_callback(self, msg: String) -> None:
        text = msg.data or ""
        if text.startswith("OBSTACLE_DETECTED"):
            dist_val = float("inf")
            bear_val = 0.0
            for token in text.split():
                if token.startswith("distance="):
                    try:
                        dist_val = float(token.split("=", 1)[1].rstrip("m"))
                    except ValueError:
                        pass
                elif token.startswith("bearing="):
                    try:
                        bear_val = float(token.split("=", 1)[1].rstrip("rad"))
                    except ValueError:
                        pass
            self.nearest_obstacle_dist = dist_val
            self.nearest_obstacle_bearing = bear_val
        elif text.startswith("OBSTACLE_CLEAR"):
            self.nearest_obstacle_dist = float("inf")
            self.nearest_obstacle_bearing = 0.0

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
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

    def _publish_mast_gaze(self, pan_rad: float, tilt_rad: float) -> None:
        if not self.active_mast_gaze:
            return
        # Smoothly slew the flexible 360-degree camera (-pi to +pi rad) with level horizon tilt (-0.12 to +0.10 rad)
        target_pan = max(-3.14159, min(3.14159, float(pan_rad)))
        target_tilt = max(-0.12, min(0.10, float(tilt_rad)))
        self._current_pan_rad = 0.65 * target_pan + 0.35 * self._current_pan_rad
        self._current_tilt_rad = 0.65 * target_tilt + 0.35 * self._current_tilt_rad
        pan_msg = Float64()
        pan_msg.data = self._current_pan_rad
        tilt_msg = Float64()
        tilt_msg.data = self._current_tilt_rad
        self.mast_pan_pub.publish(pan_msg)
        self.mast_tilt_pub.publish(tilt_msg)

    def _compute_situational_camera_gaze(
        self,
        error: float,
        far_curve_error: float,
        lookahead_dist: float,
        is_stalled_or_waiting: bool = False,
    ) -> tuple[float, float]:
        """Automatically adjust the flexible 360° rotatable camera according to the situation
        (keeping a level horizon view — never facing the ground) without moving the rover chassis."""
        _ = lookahead_dist
        self._sweep_phase += 0.22
        if is_stalled_or_waiting:
            # Situation 1: Full 360-degree situational sweep (-pi to +pi rad) while chassis holds still (0 energy wasted)
            self.camera_mode = "360_SITUATIONAL_SWEEP"
            pan_360 = 3.14159 * math.sin(self._sweep_phase * 0.65)
            tilt_horizon = -0.04 * math.sin(self._sweep_phase)
            return pan_360, tilt_horizon
        if self.nearest_obstacle_dist <= 1.85:
            # Situation 2: Obstacle detected ahead — flexibly pan between obstacle & safe bypass corridor at horizon level
            self.camera_mode = "LOW_OBSTACLE_INSPECTION"
            flank_scan = 0.12 * math.sin(self._sweep_phase * 1.1)
            pan = max(-0.45, min(0.45, 0.45 * self.nearest_obstacle_bearing + 0.55 * far_curve_error + flank_scan))
            tilt = 0.04 + 0.03 * math.cos(self._sweep_phase)
            return pan, tilt
        if abs(far_curve_error) > 0.16:
            # Situation 3: Curve / corridor turn ahead — pan camera into the turn before the chassis turns
            self.camera_mode = "CURVE_LOOKAHEAD"
            return max(-0.40, min(0.40, far_curve_error)), 0.0
        # Situation 4: Clear corridor cruise — level horizon view with gentle flexible pan
        self.camera_mode = "CORRIDOR_HORIZON_SCAN"
        gentle_pan = max(-0.22, min(0.22, 0.75 * error + 0.08 * math.sin(self._sweep_phase * 0.5)))
        return gentle_pan, 0.0

    def _publish_stop(self) -> None:
        self._last_angular_cmd = 0.0
        self.cmd_pub.publish(Twist())
        self._publish_mast_gaze(0.0, 0.0)

    def _tick(self) -> None:
        if not self.path:
            self.cmd_pub.publish(Twist())
            pan_360, tilt_360 = self._compute_situational_camera_gaze(0.0, 0.0, 1.0, True)
            self._publish_mast_gaze(pan_360, tilt_360)
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
        raw_goal_distance = math.hypot(goal[0] - current_x, goal[1] - current_y)
        goal_distance = raw_goal_distance
        if self.path:
            path_end_dist = math.hypot(self.path[-1][0] - current_x, self.path[-1][1] - current_y)
            if math.hypot(goal[0] - self.path[-1][0], goal[1] - self.path[-1][1]) > 0.30:
                goal_distance = min(goal_distance, path_end_dist)
        if self.reached and goal_distance > self.goal_tolerance * 1.5:
            self.reached = False
        if self.reached:
            self._publish_stop()
            return
        if goal_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(
                    f"INTEGRATION_GOAL_REACHED distance={goal_distance:.2f}")
            return

        # Pre-departure 360-degree camera observation scan:
        # Before moving the rover chassis (saving driving energy), sweep the flexible 360° camera
        # across the full 360° horizon while the planner evaluates the safest route.
        now_ns = self.get_clock().now().nanoseconds
        if self.pre_move_360_scan and self._pre_move_ticks < self.pre_move_scan_ticks:
            self._pre_move_ticks += 1
            phase = 2.0 * math.pi * (self._pre_move_ticks / max(1, self.pre_move_scan_ticks))
            pan_360 = 3.14159 * math.sin(phase)
            tilt_horizon = -0.05 * math.sin(2.0 * phase)
            self.camera_mode = "PRE_MOVE_360_SCAN"
            self.cmd_pub.publish(Twist())
            self._publish_mast_gaze(pan_360, tilt_horizon)
            self._progress_anchor_x = current_x
            self._progress_anchor_y = current_y
            self._progress_anchor_ns = now_ns
            if self._pre_move_ticks == 1 or self._pre_move_ticks == self.pre_move_scan_ticks:
                self.publish_status(
                    f"INTEGRATION_PRE_MOVE_360_SCAN step={self._pre_move_ticks}/{self.pre_move_scan_ticks}")
            return

        nearest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - current_x,
                                               self.path[i][1] - current_y))
        target_index = min(len(self.path) - 1, nearest + max(1, self.lookahead_cells))
        for cand_idx in range(nearest + 1, len(self.path)):
            if math.hypot(self.path[cand_idx][0] - current_x,
                          self.path[cand_idx][1] - current_y) >= 0.36:
                target_index = cand_idx
                break
        target_x, target_y = self.path[target_index]
        if raw_goal_distance <= 0.32 and (
            not self.path or math.hypot(goal[0] - self.path[-1][0], goal[1] - self.path[-1][1]) <= 0.30
        ):
            target_x, target_y = goal[0], goal[1]
        lookahead_dist = math.hypot(target_x - current_x, target_y - current_y)
        heading = math.atan2(target_y - current_y, target_x - current_x)
        error = angle_error(heading - current_yaw)

        far_index = target_index
        for cand_idx in range(target_index, len(self.path)):
            if math.hypot(self.path[cand_idx][0] - current_x,
                          self.path[cand_idx][1] - current_y) >= 0.85:
                far_index = cand_idx
                break
        far_x, far_y = self.path[far_index]
        far_curve_error = angle_error(math.atan2(far_y - current_y, far_x - current_x) - current_yaw)

        stalled_pre_recovery = False
        if self.enable_stuck_recovery:
            if self._progress_anchor_x is None or self._progress_anchor_y is None:
                self._progress_anchor_x = current_x
                self._progress_anchor_y = current_y
                self._progress_anchor_ns = now_ns
            elif math.hypot(current_x - self._progress_anchor_x,
                            current_y - self._progress_anchor_y) >= self.stuck_min_progress:
                self._progress_anchor_x = current_x
                self._progress_anchor_y = current_y
                self._progress_anchor_ns = now_ns
            elif (now_ns > self._recovery_until_ns and
                  now_ns - self._progress_anchor_ns >= int(self.stuck_timeout * 1e9)):
                self._recovery_until_ns = now_ns + int(self.recovery_duration * 1e9)
                self._progress_anchor_x = current_x
                self._progress_anchor_y = current_y
                self._progress_anchor_ns = self._recovery_until_ns

        pan_cmd, tilt_cmd = self._compute_situational_camera_gaze(
            error, far_curve_error, lookahead_dist, stalled_pre_recovery)
        self._publish_mast_gaze(pan_cmd, tilt_cmd)

        cmd = Twist()
        if self.enable_stuck_recovery and now_ns < self._recovery_until_ns:
            cmd.linear.x = -0.10
            cmd.angular.z = 0.45 if error >= 0.0 else -0.45
        else:
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.4 * error))
            if abs(error) <= 0.85:
                cmd.linear.x = max(0.09, min(self.max_linear, 0.75 * goal_distance))
                cmd.linear.x *= max(0.25, math.cos(error))
        self.cmd_pub.publish(cmd)
        if target_index != self.last_waypoint:
            self.last_waypoint = target_index
            self.publish_status(
                f"INTEGRATION_FOLLOWING waypoint={target_index}/{len(self.path)} cam_mode={self.camera_mode}")


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
