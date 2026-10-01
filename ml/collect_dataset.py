#!/usr/bin/env python3
"""Atomically add one synchronized RGB/depth/mask sample to a LunaBot dataset."""

import argparse
import json
from pathlib import Path

from lunabot_ml.dataset_manifest import DatasetError, add_sample


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--sample-id", required=True)
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--rgb", type=Path, required=True)
    parser.add_argument("--depth", type=Path, required=True)
    parser.add_argument("--mask", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--class-ids", default="0,1,2,3,4,5,6,7,8")
    args = parser.parse_args()
    try:
        metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
        allowed = {int(value) for value in args.class_ids.split(",") if value.strip()}
        record = add_sample(args.dataset, args.sample_id, args.world_id,
                            args.rgb, args.depth, args.mask, metadata, allowed)
    except (OSError, ValueError, json.JSONDecodeError, DatasetError) as exc:
        print(f"DATASET_COLLECT_FAIL: {exc}")
        return 1
    print(f"DATASET_COLLECT_PASS sample={record['sample_id']} world={record['world_id']} "
          f"size={record['width']}x{record['height']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
