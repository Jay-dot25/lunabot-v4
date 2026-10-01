#!/usr/bin/env python3
"""Validate and summarize the shared LunaBot terrain class schema."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lunabot_common.terrain_config import (  # noqa: E402
    DEFAULT_CONFIG_PATH,
    TerrainConfigError,
    load_terrain_config,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--json", action="store_true", help="print machine-readable summary")
    args = parser.parse_args()
    try:
        config = load_terrain_config(args.config)
    except TerrainConfigError as exc:
        print(f"TERRAIN_CONFIG_FAIL: {exc}", file=sys.stderr)
        return 1
    summary = {
        "schema_version": config.schema_version,
        "unknown_class": config.unknown_class,
        "class_count": len(config.classes),
        "classes": [
            {
                "id": item.id,
                "name": item.name,
                "cost": item.traversal_cost,
                "lethal": item.lethal,
                "confidence_threshold": item.confidence_threshold,
                "color_rgb": list(item.color_rgb),
            }
            for item in config.classes
        ],
    }
    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print(f"TERRAIN_CONFIG_PASS schema={config.schema_version} classes={len(config.classes)}")
        for item in config.classes:
            print(f"  {item.id}: {item.name:<16} cost={item.traversal_cost:3d} "
                  f"lethal={str(item.lethal).lower():5s} "
                  f"threshold={item.confidence_threshold:.2f} rgb={item.color_rgb}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
