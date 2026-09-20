# Phase E — Corrected Five-Class Gazebo RGB-D Perception

## Status

Phase E is **blocked pending a newly captured and annotated Gazebo dataset**. The earlier `terrain_mlp_v1.json` synthetic-feature model is rejected: GUI evidence showed horizontal image bands, no `BEDROCK`, and invalid depth presented as `SHADOW`. Static/topic PASS text is not semantic acceptance. Phase F must not begin.

## Exact classes

The valid-terrain `mono8` contract remains `0 BEDROCK`, `1 REGOLITH`, `2 ROCK`, `3 CRATER`, `4 SHADOW`. `/lunabot/terrain/validity` is a `mono8` validity image (`255` valid, `0` ignored). Ignored sky/missing depth is magenta in the overlay and excluded from class statistics. It must never be presented as observed shadow evidence.

## 1. Capture real Gazebo sensor frames

Start the independently runnable Phase D GUI (it provides unchanged RGB/depth sensors):

```bash
source /opt/ros/humble/setup.bash
~/launch-d
```

In another terminal, capture while driving to varied terrain, rocks, crater edges, lit bedrock and real cast shadows:

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
rm -rf datasets/phase_e_gazebo
python3 tools/capture_phase_e_dataset.py --ros-args \
  -p output_dir:=$PWD/datasets/phase_e_gazebo \
  -p count:=40 -p interval:=2.0
```

The capture tool writes synchronized `.npz` RGB-D data, `.ppm` previews, and JSON timestamp/encoding metadata. Do not commit the dataset; retain it as runtime evidence.

## 2. Annotate

```bash
python3 tools/annotate_phase_e_dataset.py datasets/phase_e_gazebo
```

Click polygon vertices, select a class, and press **Apply**. Save/Next writes `frame_NNNN.mask.pgm`. Unpainted pixels are `255 IGNORE`. Do not label sky, missing depth, rover parts, or uncertain boundaries. Use `n/p` for next/previous, `u` to undo, and `s` to save. Annotate at least ten frames and at least 500 valid pixels of every class, distributed across viewpoints.

## 3. Train from captured frames

```bash
python3 tools/train_terrain_mlp_gazebo.py datasets/phase_e_gazebo \
  --output models/terrain_mlp_v2.json
python3 tools/validate_phase_e.py
```

Training uses an 80/20 **whole-frame** split, not random pixels from the same image. The artifact records dataset name, frame count, class support, per-class precision/recall, confusion matrix, and held-out accuracy. Training refuses missing classes or held-out recall below 0.55. The launcher rejects the old v1 artifact.

## 4. Runtime gates

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
EVIDENCE=1 ~/launch-e
```

Headless acceptance requires the inherited Phase D goal/map/control checks and `model=lunabot_mlp_v2`. GUI acceptance requires RGB/depth/overlay comparison from several viewpoints, non-banded spatial boundaries, plausible correspondence to world objects, and evidence for all five classes. Counts alone do not prove quality.

After Ctrl+C, require clean shutdown with no stale processes and no output from:

```bash
pgrep -af 'lunar_world.sdf|parameter_bridge|terrain_segmentation.py|astar_navigation.py|async_slam_toolbox_node|control_odometry.py|odometry_monitor.py'
```

Repeat Phase E headless, then independently regress `~/launch-d`. Phase E remains incomplete until capture provenance, model metrics, visual evidence, relaunch, regression, and cleanup all pass.
