#!/usr/bin/env python3
"""Phase 3 structured-interface and legacy compatibility tests."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

from lunabot_common.status_compat import (
    PLANNER_GOAL_REACHED,
    PLANNER_NO_PATH,
    PLANNER_PATH_READY,
    REPLAN_OBSTACLE,
    mission_fields,
    parse_legacy_status,
    planner_fields,
    replan_fields,
)

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "validate_interfaces", ROOT / "tools/validate_interfaces.py")
INTERFACES = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(INTERFACES)


class StatusCompatibilityTests(unittest.TestCase):
    def test_interface_contracts_validate(self):
        self.assertEqual(len(INTERFACES.validate()), 5)

    def test_parser_extracts_first_key_value(self):
        status = parse_legacy_status('PLAN_PASS cells=42 planner="weighted astar" cells=99')
        self.assertEqual(status.code, "PLAN_PASS")
        self.assertEqual(status.values["cells"], "42")
        self.assertEqual(status.values["planner"], "weighted astar")

    def test_empty_and_malformed_status_are_safe(self):
        self.assertEqual(parse_legacy_status("   ").code, "")
        malformed = parse_legacy_status('WAITING detail="unterminated')
        self.assertEqual(malformed.code, "WAITING")

    def test_typed_value_accessors_reject_invalid_numbers(self):
        status = parse_legacy_status("X count=-2 value=nan enabled=maybe")
        self.assertEqual(status.integer("count", 7), 0)
        self.assertEqual(status.number("value", 3.0), 3.0)
        self.assertFalse(status.boolean("enabled"))

    def test_planner_pass_conversion(self):
        fields = planner_fields(
            "TERRAIN_PLAN_PASS cells=51 planning_ms=12.5 expanded=83 "
            "length=4.2 cost=73.5 planner=weighted_astar")
        self.assertEqual(fields["state"], PLANNER_PATH_READY)
        self.assertTrue(fields["success"])
        self.assertEqual(fields["path_cells"], 51)
        self.assertEqual(fields["expanded_nodes"], 83)
        self.assertAlmostEqual(fields["planning_time_ms"], 12.5)

    def test_planner_goal_and_failure_conversion(self):
        self.assertEqual(planner_fields("INTEGRATION_GOAL_REACHED")["state"],
                         PLANNER_GOAL_REACHED)
        failed = planner_fields("TERRAIN_NO_PATH reason=blocked")
        self.assertEqual(failed["state"], PLANNER_NO_PATH)
        self.assertFalse(failed["success"])

    def test_replan_requires_exact_pass_code(self):
        waiting = replan_fields("DYNAMIC_REPLAN_WAITING obstacle=1")
        self.assertFalse(waiting["success"])
        passed = replan_fields(
            "DYNAMIC_REPLAN_PASS revisions=2 obstacle=1 changed_cells=4 "
            "path_invalidated=true replan_ms=8.5 latency_ms=15.0")
        self.assertTrue(passed["success"])
        self.assertEqual(passed["reason"], REPLAN_OBSTACLE)
        self.assertTrue(passed["active_path_invalidated"])
        self.assertEqual(passed["revision"], 2)

    def test_mission_conversion(self):
        passed = mission_fields(
            "MISSION_DEMO_PASS motion=6.4 revisions=2 collisions=0 duration_s=28")
        self.assertTrue(passed["complete"])
        self.assertTrue(passed["success"])
        self.assertAlmostEqual(passed["executed_path_length_m"], 6.4)
        self.assertEqual(passed["replan_count"], 2)
        failed = mission_fields("MISSION_DEMO_FAIL reason=timeout")
        self.assertTrue(failed["complete"])
        self.assertFalse(failed["success"])
        self.assertEqual(failed["failure_reason"], "timeout")

    def test_unknown_text_never_becomes_success(self):
        self.assertFalse(planner_fields("arbitrary text")["success"])
        self.assertFalse(replan_fields("DYNAMIC_REPLAN_PASSING")["success"])
        self.assertFalse(mission_fields("MISSION_DEMO_PASSING")["success"])


if __name__ == "__main__":
    unittest.main()
