#!/usr/bin/env python3
"""Register one RGB frame and two independently human-reviewed masks."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunabot_ml.eyes_of_luna_dataset import (  # noqa: E402
    DatasetError,
    collect_sample,
    load_task_config,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--sample-id", required=True)
    parser.add_argument("--group-id", required=True,
                        help="shared world/capture group; all its frames stay in one split")
    parser.add_argument("--rgb", type=Path, required=True)
    parser.add_argument("--terrain-mask", type=Path, required=True)
    parser.add_argument("--shadow-mask", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True,
                        help="JSON capture details and reviewed annotation provenance")
    args = parser.parse_args()
    try:
        load_task_config()
        metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
        record = collect_sample(args.dataset, args.sample_id, args.group_id,
                                args.rgb, args.terrain_mask, args.shadow_mask, metadata)
    except (DatasetError, OSError, json.JSONDecodeError) as exc:
        print(f"EYES_OF_LUNA_COLLECTION_FAIL: {exc}")
        return 1
    print(json.dumps(record, indent=2, sort_keys=True))
    print("EYES_OF_LUNA_COLLECTION_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
