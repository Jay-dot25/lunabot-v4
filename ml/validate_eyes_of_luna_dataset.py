#!/usr/bin/env python3
"""Validate an Eyes of Luna RGB/terrain/shadow dataset without ML dependencies."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    DatasetError,
    load_task_config,
    validate_dataset,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        load_task_config()
        report = validate_dataset(args.dataset)
    except DatasetError as exc:
        print(f"EYES_OF_LUNA_VALIDATION_FAIL: {exc}")
        return 1
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    print("EYES_OF_LUNA_VALIDATION_PASS" if report["valid"]
          else "EYES_OF_LUNA_VALIDATION_FAIL")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
