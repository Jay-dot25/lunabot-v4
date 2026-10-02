#!/usr/bin/env python3
"""Validate and normalize a LunaBot simulator scenario manifest."""

import argparse
import json
from pathlib import Path

from scenario_schema import ScenarioError, load_scenario, normalized_runtime_manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        runtime = normalized_runtime_manifest(load_scenario(args.manifest))
    except ScenarioError as exc:
        print(f"SCENARIO_GENERATION_FAIL: {exc}")
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(runtime, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"SCENARIO_GENERATION_PASS id={runtime['scenario_id']} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
