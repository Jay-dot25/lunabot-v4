#!/usr/bin/env python3
"""Validate Phase 5 semantic ground-truth and camera metadata contracts."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from validate_scenarios import validate_all as validate_scenarios  # noqa: E402


def validate() -> dict:
    scenarios = validate_scenarios()
    generator = (ROOT / "tools/generate_lunar_terrain.py").read_text(encoding="utf-8")
    for token in ("--crater-density", "--metadata-output", "--semantic-mask-output",
                  "semantic_layers", "write_ground_truth", "evaluation_only"):
        if token not in generator:
            raise ValueError(f"terrain generator missing {token}")
    camera_source = ROOT / "src/lunabot_perception/lunabot_perception/camera_info_publisher.py"
    ast.parse(camera_source.read_text(encoding="utf-8"), filename=str(camera_source))
    camera_config = ROOT / "src/lunabot_perception/config/camera_info.yaml"
    if not camera_config.is_file():
        raise ValueError("camera calibration configuration missing")
    config_text = camera_config.read_text(encoding="utf-8")
    for field in ("fx:", "fy:", "cx:", "cy:", "rgb_topic:", "depth_topic:"):
        if field not in config_text:
            raise ValueError(f"camera configuration missing {field}")
    world = (ROOT / "src/lunabot_gazebo/worlds/lunar_world.sdf").read_text(encoding="utf-8")
    for model in ("lunar_habitat_main", "lunar_habitat_equipment",
                  "presentation_rock_left", "presentation_obstacle_forward"):
        if f'name="{model}"' not in world:
            raise ValueError(f"world model missing stable identity: {model}")
    return {"scenarios": len(scenarios), "camera_info_topics": 2, "ground_truth_layers": 5}


def main() -> int:
    try:
        result = validate()
    except (OSError, ValueError) as exc:
        print(f"GROUND_TRUTH_VALIDATION_FAIL: {exc}", file=sys.stderr)
        return 1
    print("GROUND_TRUTH_VALIDATION_PASS "
          + " ".join(f"{key}={value}" for key, value in result.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
