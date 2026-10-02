# Phase 5 — Simulation ground truth and scenario metadata

## Scope and safety boundary

Phase 5 adds deterministic semantic terrain metadata, scenario manifests,
stable object semantic IDs, and calibrated `CameraInfo`. Ground truth is for
dataset generation and evaluation only. Every scenario sets:

```json
"ground_truth_navigation_allowed": false
```

The navigation, SLAM, mapping, and planning nodes must never subscribe to
`/lunabot/ground_truth/*` topics. A rendered camera-view semantic source is
simulator-version-dependent and remains a Phase 5 workstation integration
step; this phase supplies the validated top-down world truth and object
contracts without risking the working Fortress world.

## Terrain generator additions

`tools/generate_lunar_terrain.py` now supports:

```text
--seed
--crater-density (0.1 to 2.0)
--mesh-output-dir
--evidence-dir
--metadata-output
--semantic-mask-output
```

The metadata/mask options must be supplied together. Outputs include:

- 8-bit semantic terrain mask PNG;
- elevation layer (`float32 .npy`);
- slope layer (`float32 .npy`);
- roughness layer (`float32 .npy`);
- illumination layer (`float32 .npy`);
- crater geometry and semantic IDs;
- class histogram and SHA-256 checksums.

Terrain labels use the shared schema. Craters override appearance classes, and
the spawn pad remains flat regolith. Rock and habitat classes come from
scenario objects rather than the terrain raster.

Example isolated generation (does not overwrite baseline assets):

```bash
rm -rf /tmp/lunabot-world-42
mkdir -p /tmp/lunabot-world-42/{meshes,evidence,truth}
python3 tools/generate_lunar_terrain.py \
  --seed 42 --crater-density 1.0 --no-previews \
  --mesh-output-dir /tmp/lunabot-world-42/meshes \
  --evidence-dir /tmp/lunabot-world-42/evidence \
  --metadata-output /tmp/lunabot-world-42/truth/terrain.json \
  --semantic-mask-output /tmp/lunabot-world-42/truth/semantic.png
```

## Scenarios

Validated manifests are under `config/scenarios/`:

- `baseline_habitat.json`;
- `dynamic_obstacle.json`;
- `low_sun.json`.

They define terrain seed/density, illumination, goal, objects, stable semantic
IDs, and optional insertion triggers. Normalize one for a runner with:

```bash
python3 tools/generate_scenario.py \
  --manifest config/scenarios/dynamic_obstacle.json \
  --output /tmp/dynamic-obstacle-runtime.json
```

Semantic object IDs are restricted to small rock (4), large rock (5), and
habitat/structure (8). Terrain craters use ID 6.

## Camera calibration metadata

The Gazebo camera model is 640×480 with horizontal FOV 1.047 rad. The initial
pinhole calibration is in:

```text
src/lunabot_perception/config/camera_info.yaml
```

After building:

```bash
ros2 run lunabot_perception camera_info_publisher \
  --ros-args --params-file \
  install/lunabot_perception/share/lunabot_perception/config/camera_info.yaml
```

It publishes retained RGB and depth `CameraInfo`. These simulator intrinsics
must not be represented as physical-camera calibration.

## Verification

```bash
python3 tools/validate_scenarios.py
python3 tools/validate_ground_truth.py
python3 -m unittest -v tests.test_simulation_ground_truth
python3 tools/validate_all.py --no-phase-validators --no-write
```

## Honest limitation

Phase 5 does not claim photorealistic ground-truth camera masks are already
streaming in Gazebo Fortress. It creates deterministic world-space labels and
all contracts required to add or bridge a supported segmentation-camera plugin
without modifying the validated baseline world. Dataset collection must not
begin at scale until camera-view masks are aligned and verified on the target
simulator.
