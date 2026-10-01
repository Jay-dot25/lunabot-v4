"""Compatibility parsing for legacy string status topics.

The A–L baseline remains unchanged. This module gives the Phase 3 bridge a
strict, tested translation boundary from legacy strings to structured message
fields. Unknown tokens are retained in `detail` and never treated as success.
"""

from __future__ import annotations

import math
import shlex
from dataclasses import dataclass, field
from typing import Mapping


@dataclass(frozen=True)
class LegacyStatus:
    code: str
    values: Mapping[str, str] = field(default_factory=dict)
    detail: str = ""

    def integer(self, key: str, default: int = 0) -> int:
        raw = self.values.get(key)
        if raw is None:
            return default
        try:
            value = int(raw)
        except (TypeError, ValueError):
            return default
        return max(0, value)

    def number(self, key: str, default: float = 0.0) -> float:
        raw = self.values.get(key)
        if raw is None:
            return default
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return default
        return value if math.isfinite(value) else default

    def boolean(self, key: str, default: bool = False) -> bool:
        raw = self.values.get(key)
        if raw is None:
            return default
        normalized = raw.strip().lower()
        if normalized in {"1", "true", "yes", "on", "pass"}:
            return True
        if normalized in {"0", "false", "no", "off", "fail"}:
            return False
        return default


def parse_legacy_status(text: str) -> LegacyStatus:
    if not isinstance(text, str):
        raise TypeError("legacy status must be a string")
    stripped = text.strip()
    if not stripped:
        return LegacyStatus(code="", detail="")
    try:
        tokens = shlex.split(stripped)
    except ValueError:
        tokens = stripped.split()
    code = tokens[0]
    values: dict[str, str] = {}
    for token in tokens[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key and key not in values:
            values[key] = value
    return LegacyStatus(code=code, values=values, detail=stripped)


# Numeric values intentionally mirror msg/PlannerStatus.msg.
PLANNER_IDLE = 0
PLANNER_PLANNING = 1
PLANNER_PATH_READY = 2
PLANNER_GOAL_REACHED = 3
PLANNER_NO_PATH = 4
PLANNER_ERROR = 5


def planner_fields(text: str) -> dict:
    status = parse_legacy_status(text)
    code = status.code.upper()
    state = PLANNER_IDLE
    success = False
    if any(marker in code for marker in ("PLAN_PASS", "PATH_READY", "PLANNED")):
        state, success = PLANNER_PATH_READY, True
    elif "GOAL_REACHED" in code:
        state, success = PLANNER_GOAL_REACHED, True
    elif any(marker in code for marker in ("NO_PATH", "FAILED", "ERROR")):
        state = PLANNER_NO_PATH if "NO_PATH" in code else PLANNER_ERROR
    elif any(marker in code for marker in ("PLANNING", "WAITING")):
        state = PLANNER_PLANNING
    return {
        "state": state,
        "success": success,
        "planner_id": status.values.get("planner", "legacy_weighted_astar"),
        "incremental": status.boolean("incremental", False),
        "planning_time_ms": status.number("planning_ms", status.number("time_ms")),
        "expanded_nodes": status.integer("expanded"),
        "path_cells": status.integer("cells"),
        "path_length_m": status.number("length", status.number("path_length")),
        "accumulated_cost": status.number("cost"),
        "detail": status.detail,
    }


# Numeric values mirror msg/ReplanEvent.msg.
REPLAN_NONE = 0
REPLAN_OBSTACLE = 1
REPLAN_TERRAIN_COST = 2
REPLAN_GOAL_CHANGED = 3
REPLAN_START_MOVED = 4
REPLAN_PATH_INVALID = 5


def replan_fields(text: str) -> dict:
    status = parse_legacy_status(text)
    upper = status.detail.upper()
    reason = REPLAN_NONE
    if "OBSTACLE" in upper:
        reason = REPLAN_OBSTACLE
    elif "TERRAIN" in upper or "COST" in upper:
        reason = REPLAN_TERRAIN_COST
    elif "GOAL" in upper:
        reason = REPLAN_GOAL_CHANGED
    elif "START" in upper or "MOTION" in upper:
        reason = REPLAN_START_MOVED
    elif "PATH_INVALID" in upper:
        reason = REPLAN_PATH_INVALID
    success = status.code.upper() == "DYNAMIC_REPLAN_PASS"
    return {
        "reason": reason,
        "active_path_invalidated": status.boolean("path_invalidated", False),
        "success": success,
        "changed_cells": status.integer("changed_cells"),
        "revision": status.integer("revision", status.integer("revisions")),
        "detection_to_map_ms": status.number("detection_to_map_ms"),
        "replan_time_ms": status.number("replan_ms"),
        "total_latency_ms": status.number("latency_ms"),
        "detail": status.detail,
    }


def mission_fields(text: str) -> dict:
    status = parse_legacy_status(text)
    code = status.code.upper()
    complete = code in {"MISSION_DEMO_PASS", "MISSION_DEMO_FAIL"}
    success = code == "MISSION_DEMO_PASS"
    return {
        "complete": complete,
        "success": success,
        "failure_reason": "" if success else status.values.get("reason", ""),
        "planned_path_length_m": status.number("planned_length"),
        "executed_path_length_m": status.number("executed_length", status.number("motion")),
        "mission_duration_s": status.number("duration_s"),
        "path_efficiency": status.number("efficiency"),
        "replan_count": status.integer("replans", status.integer("revisions")),
        "collision_count": status.integer("collisions"),
        "detail": status.detail,
    }
