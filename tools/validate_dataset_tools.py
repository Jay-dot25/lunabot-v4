#!/usr/bin/env python3
"""Validate Phase 4 dataset tooling contracts without requiring a real dataset."""

from __future__ import annotations

import ast
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ML = ROOT / "ml"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ML))

from lunabot_ml.dataset_manifest import (  # noqa: E402
    add_sample,
    split_by_world,
    validate_dataset,
    write_mask_png,
)
from lunabot_common import load_terrain_config  # noqa: E402


def metadata(timestamp: int) -> dict:
    return {
        "rgb_timestamp_ns": timestamp,
        "depth_timestamp_ns": timestamp + 1,
        "mask_timestamp_ns": timestamp + 2,
        "sync_tolerance_ns": 10,
        "camera_intrinsics": {"fx": 10.0, "fy": 10.0, "cx": 1.0, "cy": 1.0},
        "camera_pose": [0, 0, 1, 0, 0, 0, 1],
        "rover_pose": [0, 0, 0, 0, 0, 0, 1],
        "lighting": {"fixture": True},
    }


def validate() -> dict:
    scripts = ("collect_dataset.py", "validate_dataset.py", "split_dataset.py",
               "report_dataset.py")
    for name in scripts:
        path = ML / name
        if not path.is_file():
            raise ValueError(f"missing dataset tool: {path}")
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    allowed = set(load_terrain_config().by_id)
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        dataset = root / "dataset"
        for index in range(3):
            files = []
            for channel, value in (("rgb", 20 + index), ("depth", 40 + index),
                                   ("mask", index)):
                path = root / f"{channel}-{index}.png"
                write_mask_png(path, 2, 2, bytes([value]) * 4)
                files.append(path)
            add_sample(dataset, f"sample-{index}", f"world-{index}", *files,
                       metadata(1_000 + index * 10), allowed)
        report = validate_dataset(dataset, allowed)
        if not report["valid"] or report["sample_count"] != 3:
            raise ValueError("dataset smoke validation failed: " + json.dumps(report))
        split = split_by_world(dataset, seed=42)
        worlds = [set(item["worlds"]) for item in split["splits"].values()]
        if any(worlds[i] & worlds[j] for i in range(3) for j in range(i + 1, 3)):
            raise ValueError("world leakage detected in smoke split")
    return {"tools": len(scripts), "classes": len(allowed), "samples": 3, "worlds": 3}


def main() -> int:
    try:
        result = validate()
    except (OSError, ValueError) as exc:
        print(f"DATASET_TOOLS_FAIL: {exc}", file=sys.stderr)
        return 1
    print("DATASET_TOOLS_PASS " + " ".join(f"{key}={value}" for key, value in result.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
