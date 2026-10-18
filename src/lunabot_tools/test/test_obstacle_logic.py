import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lunabot_tools.obstacle_logic import clamp_velocity, evaluate_forward_scan


class ObstacleLogicTest(unittest.TestCase):
    def setUp(self):
        self.args = (-math.pi, math.pi / 180, 0.12, 20.0, 1.5, math.radians(15))

    def test_detects_nearest_forward_obstacle(self):
        ranges = [float('inf')] * 361
        ranges[180] = 0.8
        ranges[185] = 1.1
        self.assertEqual(evaluate_forward_scan(ranges, *self.args), ('OBSTACLE', 0.8))

    def test_positive_infinity_is_a_valid_clear_return(self):
        ranges = [float('inf')] * 361
        self.assertEqual(evaluate_forward_scan(ranges, *self.args), ('CLEAR', None))

    def test_outside_sector_does_not_trigger(self):
        ranges = [float('inf')] * 361
        ranges[0] = 0.2
        self.assertEqual(evaluate_forward_scan(ranges, *self.args), ('CLEAR', None))

    def test_all_invalid_data_is_not_clear(self):
        ranges = [float('nan')] * 361
        self.assertEqual(evaluate_forward_scan(ranges, *self.args), ('INVALID', None))

    def test_empty_data_is_invalid(self):
        self.assertEqual(evaluate_forward_scan([], *self.args), ('INVALID', None))

    def test_clamps_commands_and_replaces_nan(self):
        self.assertEqual(clamp_velocity(2.0, -3.0, 0.35, 0.8), (0.35, -0.8))
        self.assertEqual(clamp_velocity(float('nan'), float('inf'), 0.35, 0.8), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
