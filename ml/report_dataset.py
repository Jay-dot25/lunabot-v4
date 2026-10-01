#!/usr/bin/env python3
"""Generate a readable LunaBot dataset inventory from a validated manifest."""

import argparse
import json
from pathlib import Path

from lunabot_ml.dataset_manifest import DatasetError, validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = validate_dataset(args.dataset, set(range(9)))
    except DatasetError as exc:
        print(f"DATASET_REPORT_FAIL: {exc}")
        return 1
    lines = [
        "# LunaBot dataset report", "",
        f"- Valid: **{report['valid']}**",
        f"- Samples: **{report['sample_count']}**",
        f"- Worlds: **{report['world_count']}**", "",
        "## Samples per world", "",
        "| World | Samples |", "|---|---:|",
    ]
    lines.extend(f"| `{world}` | {count} |"
                 for world, count in report["samples_per_world"].items())
    lines.extend(["", "## Class pixels", "", "| Class ID | Pixels |", "|---:|---:|"])
    lines.extend(f"| {class_id} | {count} |"
                 for class_id, count in report["class_pixels"].items())
    if report["errors"]:
        lines.extend(["", "## Errors", ""] + [f"- {error}" for error in report["errors"]])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"DATASET_REPORT_{'PASS' if report['valid'] else 'FAIL'} output={args.output}")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
