#!/usr/bin/env python3
"""Make deterministic train/validation/test splits by capture group, not frame."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    DatasetError,
    load_task_config,
    save_capture_group_split,
    split_by_capture_group,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--train", type=float, default=0.70)
    parser.add_argument("--validation", type=float, default=0.15)
    parser.add_argument("--test", type=float, default=0.15)
    args = parser.parse_args()
    try:
        load_task_config()
        split = split_by_capture_group(
            args.dataset, args.seed, (args.train, args.validation, args.test))
        output = save_capture_group_split(args.dataset, split)
    except DatasetError as exc:
        print(f"EYES_OF_LUNA_SPLIT_FAIL: {exc}")
        return 1
    print(json.dumps(split, indent=2, sort_keys=True))
    print(f"EYES_OF_LUNA_SPLIT_PASS file={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
