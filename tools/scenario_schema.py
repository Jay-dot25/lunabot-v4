"""Validated LunaBot simulator scenario manifests."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

NAME = re.compile(r"^[a-z][a-z0-9_]*$")


class ScenarioError(ValueError):
    pass


def load_scenario(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioError(f"cannot load scenario {path}: {exc}") from exc
    validate_scenario(data)
    return data


def _finite_vector(value, length: int, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length or not all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            and math.isfinite(float(item)) for item in value):
        raise ScenarioError(f"{field} must contain {length} finite numbers")
    return [float(item) for item in value]


def validate_scenario(data: object) -> None:
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ScenarioError("scenario schema_version must be 1")
    scenario_id = data.get("scenario_id")
    if not isinstance(scenario_id, str) or not NAME.fullmatch(scenario_id):
        raise ScenarioError("scenario_id must use lower_snake_case")
    seed = data.get("terrain_seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ScenarioError("terrain_seed must be a non-negative integer")
    density = data.get("crater_density")
    if not isinstance(density, (int, float)) or isinstance(density, bool) or not 0.1 <= density <= 2.0:
        raise ScenarioError("crater_density must be within [0.1, 2.0]")
    lighting = data.get("lighting")
    if not isinstance(lighting, dict):
        raise ScenarioError("lighting must be an object")
    direction = _finite_vector(lighting.get("direction"), 3, "lighting.direction")
    if math.isclose(sum(component * component for component in direction), 0.0):
        raise ScenarioError("lighting.direction cannot be zero")
    intensity = lighting.get("intensity")
    if not isinstance(intensity, (int, float)) or isinstance(intensity, bool) or not 0 <= intensity <= 10:
        raise ScenarioError("lighting.intensity must be within [0, 10]")
    objects = data.get("objects")
    if not isinstance(objects, list):
        raise ScenarioError("objects must be a list")
    names: set[str] = set()
    allowed_semantics = {4, 5, 8}
    for index, obj in enumerate(objects):
        field = f"objects[{index}]"
        if not isinstance(obj, dict):
            raise ScenarioError(f"{field} must be an object")
        name = obj.get("name")
        if not isinstance(name, str) or not NAME.fullmatch(name) or name in names:
            raise ScenarioError(f"{field}.name must be unique lower_snake_case")
        names.add(name)
        semantic_id = obj.get("semantic_id")
        if semantic_id not in allowed_semantics:
            raise ScenarioError(f"{field}.semantic_id must be one of {sorted(allowed_semantics)}")
        pose = _finite_vector(obj.get("pose"), 6, f"{field}.pose")
        if abs(pose[0]) > 195 or abs(pose[1]) > 195:
            raise ScenarioError(f"{field}.pose is outside terrain bounds")
        size = _finite_vector(obj.get("size"), 3, f"{field}.size")
        if any(value <= 0 for value in size):
            raise ScenarioError(f"{field}.size values must be positive")
        dynamic = obj.get("dynamic", False)
        if not isinstance(dynamic, bool):
            raise ScenarioError(f"{field}.dynamic must be boolean")
        if dynamic:
            trigger = obj.get("insertion_trigger")
            if not isinstance(trigger, dict) or trigger.get("type") not in {
                    "time_seconds", "rover_distance", "service"}:
                raise ScenarioError(f"{field}: dynamic object requires insertion_trigger")
    goal = _finite_vector(data.get("goal_xy"), 2, "goal_xy")
    if abs(goal[0]) > 190 or abs(goal[1]) > 190:
        raise ScenarioError("goal_xy is outside safe terrain bounds")
    if data.get("ground_truth_navigation_allowed") is not False:
        raise ScenarioError("ground_truth_navigation_allowed must be false")


def normalized_runtime_manifest(data: dict) -> dict:
    validate_scenario(data)
    return {
        "schema_version": 1,
        "scenario_id": data["scenario_id"],
        "terrain": {
            "seed": data["terrain_seed"],
            "crater_density": float(data["crater_density"]),
        },
        "lighting": data["lighting"],
        "goal_xy": data["goal_xy"],
        "objects": data["objects"],
        "ground_truth": {
            "semantic_topic": "/lunabot/ground_truth/semantic",
            "pose_topic": "/lunabot/ground_truth/pose",
            "evaluation_only": True,
            "navigation_allowed": False,
        },
    }
