#!/usr/bin/env python3
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'src/lunabot_bringup/launch/phase17_bridges.launch.py'


class Phase17BridgeStageTests(unittest.TestCase):
    def test_launch_is_parseable_and_includes_gazebo_stage(self):
        text = LAUNCH.read_text()
        ast.parse(text)
        self.assertIn('phase17_gazebo.launch.py', text)
        self.assertNotIn('rviz2', text)
        self.assertNotIn('terrain_inference', text)

    def test_required_receive_bridges_are_one_way(self):
        text = LAUNCH.read_text()
        topics = ('/clock', '/lunabot/camera/image_raw', '/lunabot/camera/camera_info',
                  '/lunabot/depth/image_raw', '/lunabot/depth/camera_info',
                  '/lunabot/lidar/scan', '/lunabot/imu', '/lunabot/odom',
                  '/lunabot/joint_states')
        for topic in topics:
            self.assertIn("'" + topic + '@', text)
        self.assertGreaterEqual(text.count('[gz.msgs.'), len(topics))

    def test_velocity_command_bridge_is_ros_to_gazebo_only(self):
        text = LAUNCH.read_text()
        self.assertIn("'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'", text)

    def test_launch_and_dependency_are_installed(self):
        self.assertIn('phase17_bridges.launch.py', (ROOT/'src/lunabot_bringup/setup.py').read_text())
        self.assertIn('<exec_depend>ros_gz_bridge</exec_depend>', (ROOT/'src/lunabot_bringup/package.xml').read_text())


if __name__ == '__main__':
    unittest.main()
