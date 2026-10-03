#!/usr/bin/env python3
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'src/lunabot_bringup/launch/phase17_rviz.launch.py'
RVIZ = ROOT/'src/lunabot_bringup/rviz/phase17_minimal.rviz'


class Phase17RvizStageTests(unittest.TestCase):
    def test_launch_builds_only_on_validated_tf_stage(self):
        text = LAUNCH.read_text()
        ast.parse(text)
        self.assertIn('phase17_tf_localization.launch.py', text)
        self.assertIn("package='rviz2'", text)
        for excluded in ('terrain_inference', 'semantic_fusion', 'traversability',
                         'dstar', 'path_follower', 'mission_metrics'):
            self.assertNotIn(excluded, text)

    def test_config_contains_only_validated_display_classes(self):
        text = RVIZ.read_text()
        expected = {'Grid': 1, 'TF': 2, 'Camera': 1, 'LaserScan': 1,
                    'Odometry': 1}
        for display, count in expected.items():
            self.assertEqual(text.count('Class: rviz_default_plugins/' + display), count)
        for forbidden in ('Map', 'Path', 'PointCloud2', 'Marker', 'RobotModel'):
            self.assertNotIn('Class: rviz_default_plugins/' + forbidden, text)
        self.assertIn('Fixed Frame: map', text)

    def test_topics_match_live_stage3_evidence(self):
        text = RVIZ.read_text()
        self.assertIn('Value: /lunabot/camera/image_raw', text)
        self.assertIn('Value: /lunabot/lidar/scan', text)
        self.assertIn('Value: /odometry/filtered', text)
        self.assertNotIn('/semantic', text)
        self.assertNotIn('/traversability', text)
        self.assertNotIn('/planned_path', text)

    def test_launch_and_config_are_installed(self):
        setup = (ROOT/'src/lunabot_bringup/setup.py').read_text()
        package = (ROOT/'src/lunabot_bringup/package.xml').read_text()
        self.assertIn('phase17_rviz.launch.py', setup)
        self.assertIn('rviz/phase17_minimal.rviz', setup)
        self.assertIn('<exec_depend>rviz2</exec_depend>', package)


if __name__ == '__main__':
    unittest.main()
