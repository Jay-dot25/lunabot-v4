"""Pure Python policy tests; these run even when ROS 2 is not installed."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOLS_PACKAGE = ROOT / "src" / "lunabot_phase1_tools"
sys.path.insert(0, str(TOOLS_PACKAGE))

from lunabot_phase1_tools.command_guard_core import (  # noqa: E402
    clamp_command,
    resolve_command,
)
from lunabot_phase1_tools.obstacle_monitor_core import (  # noqa: E402
    CLEAR,
    NO_VALID_MEASUREMENTS,
    OBSTACLE_DETECTED,
    classify_obstacle,
    closest_forward_range,
    scan_is_stale,
)


class CommandGuardPolicyTests(unittest.TestCase):
    def test_velocity_is_clamped_to_configured_limits(self):
        self.assertEqual(clamp_command(0.9, -2.0, 0.35, 0.8), (0.35, -0.8))

    def test_non_finite_velocity_components_become_zero(self):
        self.assertEqual(clamp_command(float("nan"), float("inf"), 0.35, 0.8), (0.0, 0.0))

    def test_missing_or_expired_input_resolves_to_zero(self):
        missing = resolve_command(0.2, 0.4, None, 10.0, 0.5, 0.35, 0.8)
        expired = resolve_command(0.2, 0.4, 10.0, 10.51, 0.5, 0.35, 0.8)
        self.assertEqual((missing.linear_x, missing.angular_z), (0.0, 0.0))
        self.assertEqual((expired.linear_x, expired.angular_z), (0.0, 0.0))
        self.assertTrue(missing.stale)
        self.assertTrue(expired.stale)

    def test_fresh_input_is_forwarded_after_limiting(self):
        decision = resolve_command(0.6, -0.4, 5.0, 5.2, 0.5, 0.35, 0.8)
        self.assertEqual((decision.linear_x, decision.angular_z), (0.35, -0.4))
        self.assertFalse(decision.stale)

    def test_invalid_limits_are_rejected(self):
        with self.assertRaises(ValueError):
            clamp_command(0.0, 0.0, 0.0, 0.8)
        with self.assertRaises(ValueError):
            resolve_command(0.0, 0.0, 1.0, 1.0, 0.0, 0.35, 0.8)


class ObstacleMonitorPolicyTests(unittest.TestCase):
    def test_only_forward_sector_contributes_to_minimum(self):
        # Angles are -1.0, -0.5, 0.0, +0.5, +1.0 radians. The short
        # readings at the sides are outside the configured +/-0.4 rad cone.
        ranges = [0.30, 0.40, 2.20, 0.50, 0.20]
        measured = closest_forward_range(
            ranges, -1.0, 0.5, 0.12, 15.0, 0.4
        )
        self.assertEqual(measured, 2.20)

    def test_invalid_and_out_of_range_returns_are_ignored(self):
        measured = closest_forward_range(
            [float("nan"), float("inf"), 0.05, 16.0, 3.1],
            -0.4,
            0.2,
            0.12,
            15.0,
            0.4,
        )
        self.assertEqual(measured, 3.1)

    def test_no_valid_forward_readings_are_unknown_not_clear(self):
        measured = closest_forward_range(
            [float("nan"), float("inf"), 0.1], -0.2, 0.2, 0.12, 15.0, 0.3
        )
        self.assertIsNone(measured)
        self.assertEqual(
            classify_obstacle(measured, 1.5).status,
            NO_VALID_MEASUREMENTS,
        )

    def test_distance_threshold_separates_clear_and_obstacle_states(self):
        self.assertEqual(classify_obstacle(1.51, 1.5).status, CLEAR)
        self.assertEqual(classify_obstacle(1.50, 1.5).status, OBSTACLE_DETECTED)

    def test_malformed_scan_limits_do_not_crash(self):
        self.assertIsNone(
            closest_forward_range([1.0], 0.0, 0.1, 2.0, 1.0, 0.5)
        )
        self.assertIsNone(
            closest_forward_range([1.0], 0.0, 0.1, 0.0, 2.0, float("nan"))
        )

    def test_absent_or_stale_scan_is_reported_as_unknown(self):
        self.assertTrue(scan_is_stale(None, 10.0, 1.0))
        self.assertTrue(scan_is_stale(8.0, 9.01, 1.0))
        self.assertFalse(scan_is_stale(8.0, 8.99, 1.0))
        self.assertTrue(scan_is_stale(8.0, 7.0, 1.0))
        with self.assertRaises(ValueError):
            scan_is_stale(1.0, 2.0, 0.0)


if __name__ == "__main__":
    unittest.main()
