#!/usr/bin/env python3
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'src/lunabot_bringup/launch/phase17_tf_localization.launch.py'


class Phase17TfStageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = LAUNCH.read_text()
        ast.parse(cls.text)

    def test_builds_on_bridge_stage_without_rviz(self):
        self.assertIn('phase17_bridges.launch.py', self.text)
        self.assertNotIn('rviz2', self.text)

    def test_tree_has_single_explicit_chain_for_observed_frames(self):
        for parent, child in (
            ("'map', 'odom'", 'map'),
            ("'chassis',\n                  'lunabot_v4/sensor_head/rgb_camera'", 'rgb'),
            ("'chassis',\n                  'lunabot_v4/sensor_head/depth_camera'", 'depth'),
            ("'chassis',\n                  'lunabot_v4/sensor_head/lidar'", 'lidar'),
            ("'chassis',\n                  'lunabot_v4/imu_link/imu'", 'imu')):
            with self.subTest(frame=child):
                self.assertEqual(self.text.count(parent), 1)

    def test_ekf_consumes_actual_bridge_topic_and_publishes_tf(self):
        self.assertIn("package='robot_localization'", self.text)
        self.assertIn("{'odom0': '/lunabot/odom', 'publish_tf': True", self.text)
        self.assertNotIn("'/tf@", self.text)

    def test_installed_with_runtime_dependencies(self):
        setup = (ROOT/'src/lunabot_bringup/setup.py').read_text()
        package = (ROOT/'src/lunabot_bringup/package.xml').read_text()
        self.assertIn('phase17_tf_localization.launch.py', setup)
        self.assertIn('<exec_depend>tf2_ros</exec_depend>', package)
        self.assertIn('<exec_depend>robot_localization</exec_depend>', package)


if __name__ == '__main__':
    unittest.main()
