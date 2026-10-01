#!/usr/bin/env python3
"""Validate all LunaBot dataset files, labels, metadata, dimensions, and hashes."""

import argparse
import json
from pathlib import Path

from lunabot_ml.dataset_manifest import DatasetError, validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--class-ids", default="0,1,2,3,4,5,6,7,8")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        allowed = {int(value) for value in args.class_ids.split(",") if value.strip()}
        report = validate_dataset(args.dataset, allowed)
    except (ValueError, DatasetError) as exc:
        print(f"DATASET_VALIDATION_FAIL: {exc}")
        return 1
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    print("DATASET_VALIDATION_PASS" if report["valid"] else "DATASET_VALIDATION_FAIL")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
