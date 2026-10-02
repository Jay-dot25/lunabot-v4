#!/usr/bin/env python3
"""Validate all Phase 5 semantic simulation scenarios."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from scenario_schema import ScenarioError, load_scenario  # noqa: E402


def validate_all() -> list[str]:
    paths = sorted((ROOT / "config/scenarios").glob("*.json"))
    if len(paths) < 3:
        raise ScenarioError("at least three scenario manifests are required")
    identifiers = []
    dynamic_count = 0
    seeds = set()
    for path in paths:
        scenario = load_scenario(path)
        identifiers.append(scenario["scenario_id"])
        seeds.add(scenario["terrain_seed"])
        dynamic_count += sum(bool(item.get("dynamic")) for item in scenario["objects"])
    if len(set(identifiers)) != len(identifiers):
        raise ScenarioError("scenario IDs must be unique")
    if dynamic_count < 1:
        raise ScenarioError("at least one dynamic obstacle scenario is required")
    if len(seeds) < 2:
        raise ScenarioError("scenario suite must use multiple terrain seeds")
    return identifiers


def main() -> int:
    try:
        identifiers = validate_all()
    except ScenarioError as exc:
        print(f"SCENARIO_VALIDATION_FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"SCENARIO_VALIDATION_PASS scenarios={len(identifiers)}")
    for identifier in identifiers:
        print(f"  {identifier}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
