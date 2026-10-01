#!/usr/bin/env python3
"""Create deterministic, leakage-free train/validation/test splits by world."""

import argparse
from pathlib import Path

from lunabot_ml.dataset_manifest import DatasetError, split_by_world


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--train", type=float, default=0.70)
    parser.add_argument("--validation", type=float, default=0.15)
    parser.add_argument("--test", type=float, default=0.15)
    args = parser.parse_args()
    try:
        result = split_by_world(
            args.dataset, args.seed, (args.train, args.validation, args.test))
    except DatasetError as exc:
        print(f"DATASET_SPLIT_FAIL: {exc}")
        return 1
    print(f"DATASET_SPLIT_PASS seed={args.seed}")
    for name, split in result["splits"].items():
        print(f"  {name}: worlds={len(split['worlds'])} samples={len(split['samples'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
