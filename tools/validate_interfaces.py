#!/usr/bin/env python3
"""Statically validate Phase 3 ROS message contracts and compatibility bridge."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MSG_DIR = ROOT / "src/lunabot_msgs/msg"
EXPECTED = {
    "TerrainPrediction.msg": {"header", "state", "model_name", "class_image", "confidence_image", "inference_time_ms"},
    "PlannerStatus.msg": {"header", "state", "planner_id", "success", "planning_time_ms", "expanded_nodes", "path_cells"},
    "ReplanEvent.msg": {"header", "reason", "detection_stamp", "map_update_stamp", "replan_start_stamp", "replan_finish_stamp", "success"},
    "SafetyStatus.msg": {"header", "state", "reason", "safe_to_move", "emergency_stop_latched"},
    "MissionMetrics.msg": {"header", "complete", "success", "failure_reason", "planned_path_length_m", "executed_path_length_m", "replan_count", "collision_count"},
}
FIELD = re.compile(r"^[A-Za-z][A-Za-z0-9_/]*(?:\[\])?\s+([a-z][a-z0-9_]*)$")
CONSTANT = re.compile(r"^[A-Za-z][A-Za-z0-9_/]*\s+([A-Z][A-Z0-9_]*)=(.+)$")


def parse_message(path: Path) -> tuple[set[str], set[str]]:
    fields: set[str] = set()
    constants: set[str] = set()
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        constant = CONSTANT.fullmatch(line)
        if constant:
            name = constant.group(1)
            if name in constants:
                raise ValueError(f"{path.name}:{line_number}: duplicate constant {name}")
            constants.add(name)
            continue
        field = FIELD.fullmatch(line)
        if not field:
            raise ValueError(f"{path.name}:{line_number}: invalid message declaration {line!r}")
        name = field.group(1)
        if name in fields:
            raise ValueError(f"{path.name}:{line_number}: duplicate field {name}")
        fields.add(name)
    return fields, constants


def validate() -> list[str]:
    actual = {path.name for path in MSG_DIR.glob("*.msg")}
    if actual != set(EXPECTED):
        raise ValueError(f"message set mismatch: expected {sorted(EXPECTED)}, found {sorted(actual)}")
    details = []
    for name, required in sorted(EXPECTED.items()):
        fields, constants = parse_message(MSG_DIR / name)
        missing = sorted(required - fields)
        if missing:
            raise ValueError(f"{name}: missing required fields: {', '.join(missing)}")
        if not fields:
            raise ValueError(f"{name}: no fields")
        details.append(f"{name}:fields={len(fields)},constants={len(constants)}")

    cmake = (ROOT / "src/lunabot_msgs/CMakeLists.txt").read_text(encoding="utf-8")
    for name in EXPECTED:
        if f'"msg/{name}"' not in cmake:
            raise ValueError(f"CMakeLists.txt does not generate {name}")
    for dependency in ("builtin_interfaces", "sensor_msgs", "std_msgs"):
        if f"find_package({dependency} REQUIRED)" not in cmake:
            raise ValueError(f"CMakeLists.txt missing dependency {dependency}")

    package_root = ET.parse(ROOT / "src/lunabot_msgs/package.xml").getroot()
    if package_root.find("member_of_group") is None:
        raise ValueError("lunabot_msgs must be a rosidl_interface_packages member")

    bridge = (ROOT / "src/lunabot_evaluation/lunabot_evaluation/status_compat_bridge.py").read_text(encoding="utf-8")
    for message in ("PlannerStatus", "ReplanEvent", "MissionMetrics"):
        if message not in bridge:
            raise ValueError(f"compatibility bridge does not publish {message}")
    for forbidden in ("Twist", "PoseStamped", "OccupancyGrid"):
        if forbidden in bridge:
            raise ValueError(f"compatibility bridge must not control navigation: {forbidden}")
    return details


def main() -> int:
    try:
        details = validate()
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f"INTERFACE_CONTRACT_FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"INTERFACE_CONTRACT_PASS messages={len(details)}")
    for detail in details:
        print(f"  {detail}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
