#!/usr/bin/env python3
"""Comprehensive unit test suite for LunaBot V4 math, perception, planning, and control.

Runs in any standard Python 3 environment (with or without ROS 2 installed) by
providing lightweight message/node stubs when `rclpy` is unavailable.
"""

from __future__ import annotations

import importlib
import math
import os
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT / "scripts"
TOOLS_DIR = ROOT / "tools"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))


def _install_ros_stubs_if_needed() -> None:
    """Install deterministic in-memory ROS 2 test doubles for unit testing."""

    class FakeTimeMsg:
        def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
            self.sec = sec
            self.nanosec = nanosec

    class FakeClockTime:
        def __init__(self, nanoseconds: int = 1_000_000_000) -> None:
            self.nanoseconds = nanoseconds

        def to_msg(self) -> FakeTimeMsg:
            return FakeTimeMsg(
                int(self.nanoseconds // 1_000_000_000),
                int(self.nanoseconds % 1_000_000_000),
            )

        def __sub__(self, other: Any) -> "FakeClockTime":
            other_ns = other.nanoseconds if hasattr(other, "nanoseconds") else int(float(other) * 1e9)
            return FakeClockTime(self.nanoseconds - int(other_ns))

    class FakeClock:
        def __init__(self) -> None:
            self._ns = 10_000_000_000

        def now(self) -> FakeClockTime:
            return FakeClockTime(self._ns)

        def advance_sec(self, seconds: float) -> None:
            self._ns += int(seconds * 1e9)

    class FakeLogger:
        def info(self, _msg: str) -> None:
            pass

        def warn(self, _msg: str) -> None:
            pass

        def error(self, _msg: str) -> None:
            pass

    class FakeParam:
        def __init__(self, value):
            self.value = value

    class FakePublisher:
        def __init__(self, msg_type, topic: str, qos=None) -> None:
            self.msg_type = msg_type
            self.topic = topic
            self.qos = qos
            self.published: list = []

        def publish(self, msg) -> None:
            self.published.append(msg)

    class FakeSubscription:
        def __init__(self, msg_type, topic: str, callback, qos=None) -> None:
            self.msg_type = msg_type
            self.topic = topic
            self.callback = callback
            self.qos = qos

    class FakeNode:
        def __init__(self, name: str) -> None:
            self._name = name
            self._params: dict[str, FakeParam] = {}
            self._publishers: list[FakePublisher] = []
            self._subscriptions: list[FakeSubscription] = []
            self._timers: list = []
            self._clock = FakeClock()
            self._logger = FakeLogger()

        def declare_parameter(self, name: str, default_value=None) -> FakeParam:
            param = FakeParam(default_value)
            self._params[name] = param
            return param

        def get_parameter(self, name: str) -> FakeParam:
            return self._params[name]

        def create_publisher(self, msg_type, topic: str, qos=None) -> FakePublisher:
            pub = FakePublisher(msg_type, topic, qos)
            self._publishers.append(pub)
            return pub

        def create_subscription(self, msg_type, topic: str, callback, qos=None) -> FakeSubscription:
            sub = FakeSubscription(msg_type, topic, callback, qos)
            self._subscriptions.append(sub)
            return sub

        def create_timer(self, period: float, callback):
            timer = (period, callback)
            self._timers.append(timer)
            return timer

        def get_clock(self) -> FakeClock:
            return self._clock

        def get_logger(self) -> FakeLogger:
            return self._logger

        def destroy_node(self) -> None:
            pass

    class Header:
        def __init__(self) -> None:
            self.stamp = FakeTimeMsg()
            self.frame_id = ""

    class Point:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z

    class Vector3:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self) -> None:
            self.position = Point()
            self.orientation = Quaternion()

    class PoseStamped:
        def __init__(self) -> None:
            self.header = Header()
            self.pose = Pose()

    class Twist:
        def __init__(self) -> None:
            self.linear = Vector3()
            self.angular = Vector3()

    class Transform:
        def __init__(self) -> None:
            self.translation = Vector3()
            self.rotation = Quaternion()

    class TransformStamped:
        def __init__(self) -> None:
            self.header = Header()
            self.child_frame_id = ""
            self.transform = Transform()

    class MapMetaData:
        def __init__(self) -> None:
            self.resolution = 0.10
            self.width = 0
            self.height = 0
            self.origin = Pose()

    class OccupancyGrid:
        def __init__(self) -> None:
            self.header = Header()
            self.info = MapMetaData()
            self.data: list[int] = []

    class PathMsg:
        def __init__(self) -> None:
            self.header = Header()
            self.poses: list[PoseStamped] = []

    class Odometry:
        def __init__(self) -> None:
            self.header = Header()
            self.child_frame_id = ""
            self.pose = types.SimpleNamespace(pose=Pose())
            self.twist = types.SimpleNamespace(twist=Twist())

    class Image:
        def __init__(self) -> None:
            self.header = Header()
            self.height = 0
            self.width = 0
            self.encoding = "rgb8"
            self.is_bigendian = 0
            self.step = 0
            self.data = b""

    class LaserScan:
        def __init__(self) -> None:
            self.header = Header()
            self.angle_min = -1.57
            self.angle_max = 1.57
            self.angle_increment = 0.01
            self.ranges: list[float] = []

    class Imu:
        def __init__(self) -> None:
            self.header = Header()
            self.orientation = Quaternion()
            self.angular_velocity = Vector3()
            self.linear_acceleration = Vector3()

    class String:
        def __init__(self, data: str = "") -> None:
            self.data = data

    class Float64:
        def __init__(self, data: float = 0.0) -> None:
            self.data = data

    class Marker:
        ADD = 0
        POINTS = 8

        def __init__(self) -> None:
            self.header = Header()
            self.ns = ""
            self.id = 0
            self.action = 0
            self.type = 0
            self.pose = Pose()
            self.scale = Vector3()
            self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=1.0)
            self.points: list[Point] = []

    class TransformException(Exception):
        pass

    class Buffer:
        def __init__(self, cache_time=None) -> None:
            self._transforms: dict[tuple[str, str], TransformStamped] = {}

        def set_transform(self, target: str, source: str, tf_msg: TransformStamped) -> None:
            self._transforms[(target, source)] = tf_msg

        def lookup_transform(self, target: str, source: str, time=None, timeout=None) -> TransformStamped:
            if (target, source) in self._transforms:
                return self._transforms[(target, source)]
            if target == source:
                return TransformStamped()
            raise TransformException(f"No transform {target} <- {source}")

    class TransformListener:
        def __init__(self, buffer: Buffer, node: FakeNode) -> None:
            self.buffer = buffer
            self.node = node

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.spin = lambda node: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.ok = lambda: True

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode

    exec_mod = types.ModuleType("rclpy.executors")
    exec_mod.ExternalShutdownException = Exception

    dur_mod = types.ModuleType("rclpy.duration")
    dur_mod.Duration = lambda seconds=0.0: seconds

    time_mod = types.ModuleType("rclpy.time")
    time_mod.Time = lambda: 0

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda depth=1, **kwargs: types.SimpleNamespace(depth=depth, **kwargs)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    qos_mod.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2)
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_mod.qos_profile_sensor_data = types.SimpleNamespace(depth=5)

    geom_mod = types.ModuleType("geometry_msgs")
    geom_msg_mod = types.ModuleType("geometry_msgs.msg")
    geom_msg_mod.Point = Point
    geom_msg_mod.Pose = Pose
    geom_msg_mod.PoseStamped = PoseStamped
    geom_msg_mod.Quaternion = Quaternion
    geom_msg_mod.TransformStamped = TransformStamped
    geom_msg_mod.Twist = Twist
    geom_msg_mod.Vector3 = Vector3

    nav_mod = types.ModuleType("nav_msgs")
    nav_msg_mod = types.ModuleType("nav_msgs.msg")
    nav_msg_mod.OccupancyGrid = OccupancyGrid
    nav_msg_mod.Odometry = Odometry
    nav_msg_mod.Path = PathMsg

    sensor_mod = types.ModuleType("sensor_msgs")
    sensor_msg_mod = types.ModuleType("sensor_msgs.msg")
    sensor_msg_mod.Image = Image
    sensor_msg_mod.LaserScan = LaserScan
    sensor_msg_mod.Imu = Imu

    std_mod = types.ModuleType("std_msgs")
    std_msg_mod = types.ModuleType("std_msgs.msg")
    std_msg_mod.String = String
    std_msg_mod.Float64 = Float64

    vis_mod = types.ModuleType("visualization_msgs")
    vis_msg_mod = types.ModuleType("visualization_msgs.msg")
    vis_msg_mod.Marker = Marker

    tf2_mod = types.ModuleType("tf2_ros")
    tf2_mod.Buffer = Buffer
    tf2_mod.TransformListener = TransformListener
    tf2_mod.TransformException = TransformException

    for mod_name, mod_obj in (
        ("rclpy", rclpy_mod),
        ("rclpy.node", node_mod),
        ("rclpy.executors", exec_mod),
        ("rclpy.duration", dur_mod),
        ("rclpy.time", time_mod),
        ("rclpy.qos", qos_mod),
        ("geometry_msgs", geom_mod),
        ("geometry_msgs.msg", geom_msg_mod),
        ("nav_msgs", nav_mod),
        ("nav_msgs.msg", nav_msg_mod),
        ("sensor_msgs", sensor_mod),
        ("sensor_msgs.msg", sensor_msg_mod),
        ("std_msgs", std_mod),
        ("std_msgs.msg", std_msg_mod),
        ("visualization_msgs", vis_mod),
        ("visualization_msgs.msg", vis_msg_mod),
        ("tf2_ros", tf2_mod),
    ):
        sys.modules[mod_name] = mod_obj


_install_ros_stubs_if_needed()

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath  # noqa: E402
from sensor_msgs.msg import Image, Imu, LaserScan  # noqa: E402

import astar_navigation  # noqa: E402
import control_odometry  # noqa: E402
import dynamic_replan_monitor  # noqa: E402
import mission_control_dashboard  # noqa: E402
import obstacle_detector  # noqa: E402
import odometry_monitor  # noqa: E402
import phase_k_evaluator  # noqa: E402
import phase_l_mission  # noqa: E402
import semantic_terrain_mapper  # noqa: E402
import terrain_aware_planner  # noqa: E402
import terrain_cost_mapper  # noqa: E402
import terrain_path_follower  # noqa: E402
import terrain_segmentation  # noqa: E402


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * cy,
    )


class TestLunaBotMathAndAutonomy(unittest.TestCase):
    """Verify all Stage A and Stage B fixes and core autonomy algorithms."""

    def test_a1_quaternion_to_yaw_with_roll_and_pitch(self) -> None:
        """_yaw(q) must include + q.x * q.y so tilted rovers compute exact yaw."""
        q = _quat_from_rpy(roll=0.22, pitch=-0.18, yaw=0.65)
        expected = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.assertAlmostEqual(
            terrain_aware_planner.TerrainAwarePlanner._yaw(q), expected, places=9
        )
        self.assertAlmostEqual(
            terrain_path_follower.TerrainPathFollower._yaw(q), expected, places=9
        )
        self.assertAlmostEqual(
            semantic_terrain_mapper.SemanticTerrainMapper._yaw_from_quaternion(q),
            expected,
            places=9,
        )

    def test_a2_semantic_mapper_camera_projection_and_left_right_sign(self) -> None:
        """Right-side image pixels (x > cx) must project to local_y < 0 (right of rover)."""
        mapper = semantic_terrain_mapper.SemanticTerrainMapper()
        tf_msg = TransformStamped()
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.rotation = _quat_from_rpy(0.0, 0.0, 0.0)
        mapper.tf_buffer.set_transform("map", "chassis", tf_msg)

        width, height = 64, 48
        mask = Image()
        mask.header.stamp.sec = 1
        mask.width = width
        mask.height = height
        mask.encoding = "mono8"
        mask.step = width
        mask_bytes = bytearray(width * height)
        # Put an OBSTACLE pixel strictly on the right side of the image (x = 56, y = 24)
        mask_bytes[24 * width + 56] = semantic_terrain_mapper.OBSTACLE
        mask.data = bytes(mask_bytes)

        depth = Image()
        depth.header.stamp.sec = 1
        depth.width = width
        depth.height = height
        depth.encoding = "32fc1"
        depth.step = width * 4
        depth.data = struct.pack(f"<{width * height}f", *([2.0] * (width * height)))

        mapper._mask_callback(mask)
        mapper._depth_callback(depth)
        mapper._process_latest()

        obstacle_indices = [
            i for i, v in enumerate(mapper.cells)
            if v == semantic_terrain_mapper.OBSTACLE
        ]
        self.assertEqual(len(obstacle_indices), 1)
        idx = obstacle_indices[0]
        gx, gy = idx % mapper.width, idx // mapper.width
        world_x = mapper.origin_x + (gx + 0.5) * mapper.resolution
        world_y = mapper.origin_y + (gy + 0.5) * mapper.resolution
        self.assertGreater(world_x, 1.5)
        # Pixel x=56 > cx=31.5 is to the RIGHT of the camera optical axis -> world_y < 0
        self.assertLess(world_y, -0.3)

    def test_a3_terrain_segmentation_detects_grey_3d_obstacles_and_colored_obstacles(self) -> None:
        """Grey vertical 3D boulders and saturated obstacles must both segment as OBSTACLE."""
        seg = terrain_segmentation.TerrainSegmentation()
        seg.enable_geometric_obstacles = True
        width, height = 32, 32
        img = Image()
        img.header.stamp.sec = 2
        img.width = width
        img.height = height
        img.encoding = "rgb8"
        img.step = width * 3
        # Uniform neutral grey image (saturation = 0)
        img.data = bytes([120, 120, 120] * (width * height))

        depth = Image()
        depth.header.stamp.sec = 2
        depth.width = width
        depth.height = height
        depth.encoding = "32fc1"
        depth.step = width * 4
        # Create a close vertical wall at x=16, y=12..24 with constant depth 1.2m,
        # while the rest of the ground slopes smoothly from 1.5m to 4.5m.
        depths = []
        for y in range(height):
            for x in range(width):
                if x == 16 and 12 <= y <= 24:
                    depths.append(1.20)
                else:
                    depths.append(4.5 - 2.8 * (y / max(1, height - 1)))
        depth.data = struct.pack(f"<{width * height}f", *depths)

        seg._image_callback(img)
        seg._depth_callback(depth)
        seg._process_latest()
        self.assertGreater(len(seg.mask_pub.published), 0)
        mask_msg = seg.mask_pub.published[-1]
        # Vertical grey wall pixel at (x=16, y=20) in lower ROI is detected as OBSTACLE
        wall_label = mask_msg.data[20 * width + 16]
        # Sloped grey ground pixel at (x=8, y=28) in lower ROI is classified as TERRAIN
        ground_label = mask_msg.data[28 * width + 8]
        self.assertEqual(wall_label, terrain_segmentation.OBSTACLE)
        self.assertEqual(ground_label, terrain_segmentation.TERRAIN)

    def test_a4_obstacle_detector_3d_projection_ground_filter_and_persistence(self) -> None:
        """Tilted LiDAR ground strikes are rejected and real obstacles persist in map."""
        det = obstacle_detector.ObstacleDetector()
        det.persist_obstacles = True
        det.use_3d_projection = True
        tf_msg = TransformStamped()
        tf_msg.transform.translation.x = 0.34
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.rotation = _quat_from_rpy(0.0, 0.50, 0.0)
        det.tf_buffer.set_transform("map", "lunabot_v4/sensor_head/lidar", tf_msg)

        # Ground strike at 2.25m along 0.50 rad pitched beam is rejected (< min_obstacle_height)
        self.assertFalse(det._is_above_ground(distance=2.25, angle=0.0))
        # Box obstacle at 1.51m (height ~0.65m above ground) is accepted
        self.assertTrue(det._is_above_ground(distance=1.51, angle=0.0))

        scan = LaserScan()
        scan.header.frame_id = "lunabot_v4/sensor_head/lidar"
        scan.angle_min = -0.2
        scan.angle_increment = 0.1
        scan.ranges = [float("inf"), float("inf"), 1.40, float("inf"), float("inf")]
        det._scan_callback(scan)
        self.assertTrue(det.last_status.startswith("OBSTACLE_DETECTED"))
        first_map = det.map_pub.published[-1]
        self.assertIn(100, first_map.data)

        # Next scan has no close returns; obstacle must remain in persistent map!
        scan_clear = LaserScan()
        scan_clear.header.frame_id = "lunabot_v4/sensor_head/lidar"
        scan_clear.angle_min = -0.2
        scan_clear.angle_increment = 0.1
        scan_clear.ranges = [float("inf")] * 5
        det._scan_callback(scan_clear)
        second_map = det.map_pub.published[-1]
        self.assertIn(100, second_map.data)

    def test_a5_terrain_cost_mapper_slam_and_cross_geometry_fusion(self) -> None:
        """Cost mapper fuses semantic grid, obstacle overlay, and different-resolution SLAM /map."""
        cm = terrain_cost_mapper.TerrainCostMapper()
        cm.fuse_slam_map = True
        sem = OccupancyGrid()
        sem.header.frame_id = "map"
        sem.info.width = 20
        sem.info.height = 20
        sem.info.resolution = 0.10
        sem.info.origin.position.x = -1.0
        sem.info.origin.position.y = -1.0
        sem.data = [terrain_cost_mapper.TERRAIN_VALUE] * 400

        # SLAM map has 0.05m resolution (different from 0.10m semantic grid)
        slam = OccupancyGrid()
        slam.header.frame_id = "map"
        slam.info.width = 40
        slam.info.height = 40
        slam.info.resolution = 0.05
        slam.info.origin.position.x = -1.0
        slam.info.origin.position.y = -1.0
        slam.data = [0] * 1600
        # Mark SLAM cell at world (0.025, 0.025) -> sx=20, sy=20 as occupied (100)
        slam.data[20 * 40 + 20] = 100

        cm._slam_callback(slam)
        cm._semantic_callback(sem)
        self.assertIsNotNone(cm.last_cost_map)
        # World (0.025, 0.025) corresponds to tx=10, ty=10 in the 20x20 0.10m cost map
        self.assertEqual(cm.last_cost_map.data[10 * 20 + 10], 100)
        # Neighboring cells must have inflated halo cost > terrain_cost (20)
        self.assertGreater(cm.last_cost_map.data[10 * 20 + 11], 20)

    def test_b6_control_node_ackermann_spot_turn_and_watchdog(self) -> None:
        """ControlNode computes valid double-Ackermann and spot-turn wheel angles."""
        # Spot turn: zero linear, positive angular
        fl, fr, rl, rr = control_odometry.ControlNode._compute_steering_angles(0.0, 0.5)
        self.assertLess(fl, 0.0)
        self.assertGreater(fr, 0.0)
        self.assertAlmostEqual(fl, -fr, places=6)
        self.assertAlmostEqual(rl, -fl, places=6)
        self.assertAlmostEqual(rr, -fr, places=6)

        # Left Ackermann turn (v > 0, w > 0): inner front-left wheel steers sharper than outer front-right
        fl, fr, rl, rr = control_odometry.ControlNode._compute_steering_angles(0.30, 0.25)
        self.assertGreater(fl, fr)
        self.assertGreater(fr, 0.0)
        self.assertAlmostEqual(rl, -fl, places=6)
        self.assertAlmostEqual(rr, -fr, places=6)

        # Steep IMU pitch triggers slope safety speed scaling
        node = control_odometry.ControlNode(
            "/cmd_vel_in", "/cmd_vel", 0.45, 1.0, 10.0, 10.0, 0.5, 30.0, True
        )
        imu_msg = Imu()
        imu_msg.orientation = _quat_from_rpy(0.0, 0.42, 0.0)
        imu_msg.angular_velocity.z = 0.20
        node._imu_cb(imu_msg)
        cmd_in = Twist()
        cmd_in.linear.x = 0.40
        cmd_in.angular.z = 0.20
        node._input_cb(cmd_in)
        node.last_tick -= 0.10
        node._tick()
        self.assertAlmostEqual(node.output_v, 0.40 * 0.65, places=4)
        self.assertEqual(len(node.steer_fl_pub.published), 1)

        # Watchdog expiry brings output back to 0.0
        node.last_input -= 1.0
        node.last_tick -= 0.10
        node._tick()
        self.assertAlmostEqual(node.output_v, 0.0, places=4)

    def test_b8_odometry_monitor_imu_fusion_and_report(self) -> None:
        """OdometryMonitor fuses IMU yaw rate and records wheel-vs-IMU slip."""
        with tempfile.TemporaryDirectory() as tmpdir:
            mon = odometry_monitor.OdometryMonitor(tmpdir)
            imu = Imu()
            imu.angular_velocity.z = 0.10
            mon._imu_cb(imu)

            for i in range(12):
                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "chassis"
                odom.header.stamp.sec = i
                odom.pose.pose.position.x = 0.1 * i
                odom.pose.pose.orientation = _quat_from_rpy(0.0, 0.0, 0.08 * i)
                odom.twist.twist.linear.x = 0.10
                odom.twist.twist.angular.z = 0.25
                mon._odom_cb(odom)

            self.assertEqual(mon.imu_samples, 1)
            self.assertAlmostEqual(mon.max_yaw_slip_rate, 0.15, places=5)
            mon.write_report()
            report_text = Path(tmpdir, "odometry_report.txt").read_text()
            self.assertIn("quality_result      : PASS", report_text)
            self.assertIn("imu_samples         : 1", report_text)

    def test_b9_planner_weighted_astar_and_smoothing_avoid_obstacles(self) -> None:
        """Weighted A* and path smoothing route around high-cost obstacles."""
        planner = terrain_aware_planner.TerrainAwarePlanner()
        planner.smooth_path = True
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width = 20
        grid.info.height = 20
        grid.info.resolution = 0.10
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.data = [20] * 400
        # Place a vertical obstacle wall at x=10, y=0..14
        for y in range(15):
            grid.data[y * 20 + 10] = 100
        planner.cost_map = grid

        raw_cells, total_cost = planner._weighted_astar((2, 5), (17, 5))
        self.assertGreater(len(raw_cells), 2)
        self.assertTrue(math.isfinite(total_cost))
        for cell in raw_cells:
            self.assertNotEqual( grid.data[cell[1] * 20 + cell[0]], 100)

        smoothed = planner._prune_path(raw_cells)
        self.assertGreaterEqual(len(smoothed), 2)
        for cell in smoothed:
            self.assertNotEqual(grid.data[cell[1] * 20 + cell[0]], 100)

    def test_b7_and_b9_follower_mast_gaze_and_stuck_recovery(self) -> None:
        """TerrainPathFollower publishes mast gaze and reverses when stuck."""
        follower = terrain_path_follower.TerrainPathFollower()
        follower.active_mast_gaze = True
        follower.enable_stuck_recovery = True
        path_msg = NavPath()
        path_msg.header.frame_id = "map"
        for i in range(6):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = 0.5 * i
            ps.pose.position.y = 0.2 * i
            path_msg.poses.append(ps)
        follower._path_callback(path_msg)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 2.5
        goal_msg.pose.position.y = 1.0
        follower._goal_callback(goal_msg)

        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation = _quat_from_rpy(0.0, 0.0, 0.0)
        follower._odom_callback(odom_msg)

        follower._tick()
        self.assertGreater(len(follower.mast_pan_pub.published), 0)
        self.assertGreater(len(follower.mast_tilt_pub.published), 0)
        first_cmd = follower.cmd_pub.published[-1]
        self.assertGreater(first_cmd.linear.x, 0.0)

        # Advance simulated clock past stuck_timeout without moving rover
        follower.get_clock().advance_sec(follower.stuck_timeout + 0.5)
        follower._tick()
        recovery_cmd = follower.cmd_pub.published[-1]
        self.assertLess(recovery_cmd.linear.x, 0.0)

        # Verify automatic 360-degree situational camera modes without chassis oscillation (level horizon, never facing ground)
        pan_sweep, tilt_sweep = follower._compute_situational_camera_gaze(0.0, 0.0, 1.0, True)
        self.assertEqual(follower.camera_mode, "360_SITUATIONAL_SWEEP")
        self.assertTrue(-3.1416 <= pan_sweep <= 3.1416)
        self.assertLessEqual(abs(tilt_sweep), 0.12)

        obs_msg = terrain_path_follower.String()
        obs_msg.data = "OBSTACLE_DETECTED distance=1.42 bearing=-0.35 returns=8 source=lidar frames=5"
        follower._obstacle_callback(obs_msg)
        pan_obs, tilt_obs = follower._compute_situational_camera_gaze(0.0, -0.25, 1.0, False)
        self.assertEqual(follower.camera_mode, "LOW_OBSTACLE_INSPECTION")
        self.assertLessEqual(abs(tilt_obs), 0.12)

        # Verify pre-move 360-degree observation scan holds rover stationary (0 driving energy) while sweeping camera
        follower.pre_move_360_scan = True
        follower.pre_move_scan_ticks = 4
        follower._pre_move_ticks = 0
        follower._tick()
        self.assertEqual(follower.camera_mode, "PRE_MOVE_360_SCAN")
        self.assertEqual(follower.cmd_pub.published[-1].linear.x, 0.0)
        self.assertEqual(follower.cmd_pub.published[-1].angular.z, 0.0)

    def test_terrain_mesh_and_phase_evaluators(self) -> None:
        """Collision mesh vertices around spawn pad, 3D lunar boulder meshes, and evaluators are valid."""
        obj_path = ROOT / "src" / "lunabot_gazebo" / "worlds" / "meshes" / "lunar_terrain_collision.obj"
        self.assertTrue(obj_path.is_file())
        for boulder_mesh in (
            "lunar_boulder_primary.obj",
            "lunar_boulder_ejecta_left.obj",
            "lunar_boulder_outcrop_right.obj",
            "lunar_boulder_crater_fragment.obj",
            "lunar_boulder_anorthosite.obj",
            "lunar_boulder_monolith.obj",
            "lunar_black_skydome.obj",
            "lunar_starfield.obj",
            "lunar_massif_ridge.obj",
            "lunar_impact_crater.obj",
            "lunar_small_rocks.obj",
        ):
            mesh_file = ROOT / "src" / "lunabot_gazebo" / "worlds" / "meshes" / boulder_mesh
            self.assertTrue(mesh_file.is_file(), f"Missing 3D lunar mesh: {boulder_mesh}")
            self.assertGreater(mesh_file.stat().st_size, 5000)
        spawn_z = []
        with obj_path.open() as fh:
            for line in fh:
                if line.startswith("v "):
                    parts = line.split()
                    vx, vy, vz = float(parts[1]), float(parts[2]), float(parts[3])
                    if math.hypot(vx, vy) <= 5.0:
                        spawn_z.append(vz)
                if len(spawn_z) >= 8:
                    break
        self.assertGreaterEqual(len(spawn_z), 4)
        self.assertLess(max(spawn_z) - min(spawn_z), 0.30)

        mesh_dir = ROOT / "src" / "lunabot_gazebo" / "worlds" / "meshes"
        sdf_text = (ROOT / "src" / "lunabot_gazebo" / "worlds" / "lunar_world.sdf").read_text(encoding="utf-8")
        for sem_mesh in (
            "lunar_bedrock_patches.obj",
            "lunar_route_crater_rim.obj",
            "lunar_route_crater_wall.obj",
            "lunar_deep_shadows.obj",
            "lunar_penumbra_shadows.obj",
        ):
            mpath = mesh_dir / sem_mesh
            self.assertTrue(mpath.is_file(), f"Missing semantic terrain mesh: {sem_mesh}")
            self.assertGreater(mpath.stat().st_size, 1024)
            self.assertIn(sem_mesh, sdf_text)

        replan_mon = dynamic_replan_monitor.DynamicReplanMonitor()
        eval_k = phase_k_evaluator.PhaseKEvaluator()
        mission_l = phase_l_mission.PhaseLMission()
        self.assertIsNotNone(replan_mon)
        self.assertIsNotNone(eval_k)
        self.assertIsNotNone(mission_l)

    def test_closed_loop_phase_l_autonomy_pipeline(self) -> None:
        """Full Phase L closed-loop simulation reaches goal, replans, and passes evaluation & mission."""
        nav = astar_navigation.AStarNavigation()
        nav.auto_goal = True
        nav.require_manual_goal = False
        nav.control_enabled = False

        ctrl = control_odometry.ControlNode(
            "/cmd_vel_in", "/cmd_vel", 0.45, 1.0, 0.6, 1.5, 0.5, 20.0
        )
        seg = terrain_segmentation.TerrainSegmentation()
        sem = semantic_terrain_mapper.SemanticTerrainMapper()
        det = obstacle_detector.ObstacleDetector()
        cm = terrain_cost_mapper.TerrainCostMapper()
        planner = terrain_aware_planner.TerrainAwarePlanner()
        follower = terrain_path_follower.TerrainPathFollower()
        replan_mon = dynamic_replan_monitor.DynamicReplanMonitor()
        eval_k = phase_k_evaluator.PhaseKEvaluator()
        mission_l = phase_l_mission.PhaseLMission()
        mission_l.require_manual_goal = False

        obstacles = [
            (1.55,  0.32, 0.21, 0.21),
            (2.10,  2.55, 0.42, 0.42),
            (2.35, -3.15, 0.42, 0.42),
            (4.10, -2.35, 0.40, 0.40),
            (3.95,  2.15, 0.42, 0.42),
            (5.60, -0.40, 0.47, 0.47),
        ]
        slam_map = OccupancyGrid()
        slam_map.header.frame_id = "map"
        slam_map.info.width = 160
        slam_map.info.height = 160
        slam_map.info.resolution = 0.05
        slam_map.info.origin.position.x = -4.0
        slam_map.info.origin.position.y = -4.0
        slam_data = [0] * (160 * 160)
        for gy in range(160):
            wy = -4.0 + (gy + 0.5) * 0.05
            for gx in range(160):
                wx = -4.0 + (gx + 0.5) * 0.05
                if wx < 0.15:
                    slam_data[gy * 160 + gx] = -1
                else:
                    for cx, cy, hx, hy in obstacles:
                        if abs(wx - cx) <= hx and abs(wy - cy) <= hy:
                            slam_data[gy * 160 + gx] = 100
                            break
        slam_map.data = slam_data
        nav._map_callback(slam_map)
        nav.tf_buffer.set_transform("map", "odom", TransformStamped())
        follower.tf_buffer.set_transform("map", "odom", TransformStamped())

        rx, ry, ryaw = 0.0, 0.0, 0.0
        dt = 0.1
        min_wall_clearance = 999.0
        min_ry = 999.0
        for step in range(160):
            tf_chassis = TransformStamped()
            tf_chassis.transform.translation.x = rx
            tf_chassis.transform.translation.y = ry
            tf_chassis.transform.rotation = _quat_from_rpy(0.0, 0.0, ryaw)
            for node in (nav, sem, det, planner, follower):
                node.tf_buffer.set_transform("map", "chassis", tf_chassis)
            tf_lidar = TransformStamped()
            tf_lidar.transform.translation.x = rx + 0.20 * math.cos(ryaw)
            tf_lidar.transform.translation.y = ry + 0.20 * math.sin(ryaw)
            tf_lidar.transform.rotation = _quat_from_rpy(0.0, 0.50, ryaw)
            det.tf_buffer.set_transform("map", "lunabot_v4/sensor_head/lidar", tf_lidar)

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "chassis"
            odom.pose.pose.position.x = rx
            odom.pose.pose.position.y = ry
            odom.pose.pose.orientation = _quat_from_rpy(0.0, 0.0, ryaw)
            nav._odom_callback(odom)
            follower._odom_callback(odom)
            replan_mon._odom_callback(odom)
            eval_k._odom_callback(odom)
            mission_l._map_callback(slam_map)

            scan = LaserScan()
            scan.header.frame_id = "lunabot_v4/sensor_head/lidar"
            scan.angle_min = -0.85
            scan.angle_increment = 0.05
            ranges = []
            for i in range(35):
                ang = scan.angle_min + i * scan.angle_increment
                world_ang = ryaw + ang
                hit_r = 2.25
                for d in [0.30 + 0.05 * k for k in range(38)]:
                    wx = (rx + 0.20 * math.cos(ryaw)) + d * math.cos(0.50) * math.cos(world_ang)
                    wy = (ry + 0.20 * math.sin(ryaw)) + d * math.cos(0.50) * math.sin(world_ang)
                    if any(abs(wx - cx) <= hx and abs(wy - cy) <= hy for cx, cy, hx, hy in obstacles):
                        hit_r = d
                        break
                ranges.append(hit_r)
            scan.ranges = ranges
            det._scan_callback(scan)
            if det.map_pub.published:
                cm._obstacle_callback(det.map_pub.published[-1])
            if det.status_pub.published:
                replan_mon._obstacle_callback(det.status_pub.published[-1])
                mission_l._obstacle_callback(det.status_pub.published[-1])

            w, h = 40, 30
            rgb = Image()
            rgb.header.stamp.sec = step + 1
            rgb.width, rgb.height, rgb.encoding, rgb.step = w, h, "rgb8", w * 3
            rgb_bytes = bytearray([120, 120, 120] * (w * h))
            depth = Image()
            depth.header.stamp.sec = step + 1
            depth.width, depth.height, depth.encoding, depth.step = w, h, "32fc1", w * 4
            d_vals = [2.4] * (w * h)
            if abs(ry) < 0.20 and abs(ryaw) < 0.15 and rx < 1.2:
                for py in range(12, 22):
                    for px in range(16, 24):
                        idx = (py * w + px) * 3
                        rgb_bytes[idx:idx + 3] = bytes([210, 110, 40])
                        d_vals[py * w + px] = max(0.6, 1.525 - (rx + 0.32))
            rgb.data = bytes(rgb_bytes)
            depth.data = struct.pack(f"<{w * h}f", *d_vals)
            seg._image_callback(rgb)
            seg._depth_callback(depth)
            seg._process_latest()
            sem._mask_callback(seg.mask_pub.published[-1])
            sem._depth_callback(depth)
            sem._process_latest()
            cm._semantic_callback(sem.map_pub.published[-1])
            planner._cost_callback(cm.last_cost_map)

            nav._tick()
            if nav.goal_pub.published:
                g = nav.goal_pub.published[-1]
                planner._goal_callback(g)
                follower._goal_callback(g)
                replan_mon._goal_callback(g)
                mission_l._goal_callback(g)
            if nav.status_pub.published:
                mission_l._navigation_callback(nav.status_pub.published[-1])

            planner._plan()
            if planner.status_pub.published:
                replan_mon._planner_status_callback(planner.status_pub.published[-1])
            if planner.plan_pub.published:
                p = planner.plan_pub.published[-1]
                follower._path_callback(p)
                replan_mon._plan_callback(p)
                eval_k._plan_callback(p)
                mission_l._plan_callback(p)

            follower._tick()
            if follower.cmd_pub.published:
                cmd = follower.cmd_pub.published[-1]
                eval_k._input_callback(cmd)
                ctrl._input_cb(cmd)
                ctrl.last_tick -= dt
                ctrl.last_status -= 1.0
                ctrl._tick()
                if ctrl.cmd_pub.published:
                    eval_k._output_callback(ctrl.cmd_pub.published[-1])
                if ctrl.status_pub.published:
                    eval_k._control_callback(ctrl.status_pub.published[-1])
                rx += cmd.linear.x * math.cos(ryaw) * dt
                ry += cmd.linear.x * math.sin(ryaw) * dt
                ryaw += cmd.angular.z * dt
                min_ry = min(min_ry, ry)
                for cx, cy, hx, hy in obstacles:
                    dx = max(0.0, abs(rx - cx) - hx)
                    dy = max(0.0, abs(ry - cy) - hy)
                    min_wall_clearance = min(min_wall_clearance, math.hypot(dx, dy))

            if follower.status_pub.published:
                s = follower.status_pub.published[-1]
                eval_k._autonomy_callback(s)
                mission_l._autonomy_callback(s)
            if replan_mon.status_pub.published:
                rs = replan_mon.status_pub.published[-1]
                eval_k._replan_callback(rs)
                mission_l._replan_callback(rs)
            eval_k._evaluate()
            if eval_k.status_pub.published:
                mission_l._evaluation_callback(eval_k.status_pub.published[-1])
            mission_l._evaluate()

            if follower.reached and eval_k.last_status.startswith("EVALUATION_PASS") and mission_l.last_status.startswith("MISSION_DEMO_PASS"):
                break

        self.assertGreaterEqual(len(nav.path_pub.published[-1].poses), 2)
        self.assertTrue(follower.reached, follower.last_status)
        self.assertLess(min_ry, -0.35)
        self.assertGreater(min_wall_clearance, 0.38)
        self.assertTrue(eval_k.last_status.startswith("EVALUATION_PASS"), eval_k.last_status)
        self.assertTrue(mission_l.last_status.startswith("MISSION_DEMO_PASS"), mission_l.last_status)

    def test_mission_control_dashboard_and_report(self) -> None:
        """Mission Control Dashboard encodes camera/semantic feeds, compares Route A vs B, and exports HTML/report."""
        import tempfile
        from std_msgs.msg import String

        with tempfile.TemporaryDirectory() as tmpdir:
            dash = mission_control_dashboard.MissionControlDashboard()
            dash.evidence_dir = Path(tmpdir)

            img = Image()
            img.width = 16
            img.height = 12
            img.encoding = "rgb8"
            img.data = bytes([120, 125, 130] * (16 * 12))
            dash._on_camera(img)
            dash._on_overlay(img)
            self.assertTrue(dash.camera_data_uri.startswith("data:image/bmp;base64,"))
            self.assertTrue(dash.semantic_data_uri.startswith("data:image/bmp;base64,"))

            s_obs = String()
            s_obs.data = "OBSTACLE_DETECTED|min_range=1.42m|points=14"
            dash._on_obstacle_status(s_obs)

            s_rep = String()
            s_rep.data = "DYNAMIC_REPLAN_PASS|replans=2"
            dash._on_replan_status(s_rep)

            s_mis = String()
            s_mis.data = "MISSION_DEMO_PASS|goal=1.65,-0.62"
            dash._on_mission_status(s_mis)

            dash._periodic_export()
            report_txt = (Path(tmpdir) / "mission_control_report.txt").read_text(encoding="utf-8")
            html_txt = (Path(tmpdir) / "mission_control_dashboard.html").read_text(encoding="utf-8")
            self.assertIn("LUNABOT — MISSION CONTROL DASHBOARD", report_txt)
            self.assertIn("LUNABOT SELECTS ROUTE B", report_txt)
            self.assertIn("AUTOMATED DEMONSTRATION SCENARIO", report_txt)
            self.assertIn("WHY DID LUNABOT CHOOSE THIS PATH?", html_txt)
            self.assertIn("SCENE 1: MISSION START", html_txt)
            self.assertIn("DECISION REPLAY", html_txt)


if __name__ == "__main__":
    unittest.main()
