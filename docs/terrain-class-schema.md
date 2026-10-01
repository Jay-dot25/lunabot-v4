# Phase 1 — Shared terrain class schema

## Purpose

`config/terrain_classes.yaml` is the single source of truth for semantic class
IDs, display colors, initial traversal costs, lethal policy, and inference
confidence thresholds. It is JSON-compatible YAML and therefore loads without
a third-party YAML dependency.

These values are initial engineering policy, not experimentally proven lunar
mobility values. Later experiment phases must tune costs while preserving IDs.

## Classes

| ID | Name | Initial cost | Lethal | Meaning |
|---:|---|---:|:---:|---|
| 0 | `unknown` | 85 | no | Invalid, stale, unobserved, or low confidence |
| 1 | `flat_regolith` | 10 | no | Preferred smooth regolith |
| 2 | `rough_regolith` | 45 | no | Uneven/loose regolith; reduce speed |
| 3 | `bedrock` | 25 | no | Traversable if geometric limits permit |
| 4 | `small_rock` | 85 | no | High-risk potentially avoidable rock |
| 5 | `large_rock` | 100 | yes | Non-traversable obstacle |
| 6 | `crater` | 100 | yes | Hazardous crater interior/rim |
| 7 | `shadow` | 75 | no | Conservative uncertainty class |
| 8 | `habitat` | 100 | yes | Protected structure/equipment |

A cost of 100 is reserved for lethal cells. Geometry, slope, roughness,
clearance, and confidence penalties will be layered on these base costs in a
later phase.

## Compatibility with the current baseline

The old `scripts/terrain_segmentation.py` intentionally remains unchanged. Its
private mask encoding is:

```text
0 unknown, 1 generic terrain, 2 generic obstacle
```

That baseline ID `2` does **not** mean `rough_regolith`. Until the ML inference
and compatibility adapter phase, old baseline masks must not be interpreted as
the new multiclass schema. Preserving this boundary prevents silently changing
the already validated A–L behavior.

Likewise, the old semantic mapper and cost mapper remain baseline components.
They will be migrated only after message/package foundations and adapter tests
exist.

## Loader behavior

`lunabot_common.terrain_config` provides immutable `TerrainClass` and
`TerrainConfig` objects. Validation rejects:

- duplicate IDs, names, or colors;
- IDs or RGB channels outside 0–255;
- costs outside 0–100;
- confidence thresholds outside 0–1;
- lethal classes whose cost is not 100;
- absent or nonzero unknown class;
- malformed names and missing descriptions.

Unknown numeric predictions safely map to `unknown`. Predictions below their
class confidence threshold also map to `unknown`.

## Verification

```bash
python3 tools/validate_terrain_config.py
python3 tools/validate_terrain_config.py --json
python3 -m unittest tests.test_terrain_config
python3 tools/validate_all.py --no-phase-validators --no-write
```

The visualization palette is stable and ID-indexed. Do not reorder or reuse IDs
after a dataset or model has been produced; make schema changes through an
explicit version migration.
