# Phase 4 — Dataset format, collection, validation, and splitting

## Scope

Phase 4 supplies dependency-free tools for constructing a reproducible semantic
terrain dataset. It does not claim that the final dataset has been collected.
A real dataset requires Phase 5 simulator ground-truth outputs and execution on
a ROS/Gazebo workstation.

## Layout

```text
datasets/lunabot/
  manifest.json
  rgb/<sample-id>.png
  depth/<sample-id>.png
  masks/<sample-id>.png
  metadata/<sample-id>.json
  splits/world_split.json
```

Generated `datasets/` content is ignored by Git. Store large shared data in DVC,
Git LFS, or external object storage.

## Semantic mask contract

Masks must be non-interlaced 8-bit grayscale PNG files. Every pixel is a class
ID from `config/terrain_classes.yaml` (currently 0–8). Palette/color images are
not accepted as labels, preventing accidental training on visualization colors.

## Metadata contract

Every sample metadata JSON requires:

- `rgb_timestamp_ns`, `depth_timestamp_ns`, `mask_timestamp_ns`;
- optional `sync_tolerance_ns` (default 50 ms);
- `camera_intrinsics`: positive finite `fx`, `fy`, `cx`, `cy`;
- `camera_pose`: `[x,y,z,qx,qy,qz,qw]`;
- `rover_pose`: `[x,y,z,qx,qy,qz,qw]`;
- `lighting`: an object containing scenario lighting parameters.

Collection rejects unsynchronized timestamps, mismatched dimensions, unknown
class IDs, duplicate sample IDs, and duplicate RGB/depth/mask triplets. Stored
files and metadata receive SHA-256 checksums.

## Add one sample

```bash
python3 ml/collect_dataset.py \
  --dataset datasets/lunabot \
  --sample-id world01-frame000001 \
  --world-id world01 \
  --rgb /path/rgb.png \
  --depth /path/depth.png \
  --mask /path/mask.png \
  --metadata /path/metadata.json
```

Files are copied into canonical paths and the manifest is replaced atomically.
If collection fails, partially copied files are removed.

## Validate

```bash
python3 ml/validate_dataset.py \
  --dataset datasets/lunabot \
  --output results/dataset-validation.json
```

Validation checks existence, PNG structure/CRC, dimensions, mask labels,
metadata, checksums, IDs, and duplicate triplets.

## Split by world

```bash
python3 ml/split_dataset.py --dataset datasets/lunabot --seed 42
```

Splitting is deterministic and assigns complete worlds—not random frames—to
train, validation, and test. At least three worlds are required. No world may
occur in more than one split.

## Report

```bash
python3 ml/report_dataset.py \
  --dataset datasets/lunabot \
  --output results/dataset-report.md
```

## Verification

```bash
python3 tools/validate_dataset_tools.py
python3 -m unittest -v tests.test_dataset_tools
python3 tools/validate_all.py --no-phase-validators --no-write
```
