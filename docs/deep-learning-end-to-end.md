# Deep learning terrain perception — end-to-end implementation guide

This guide turns the high-level "Deep Learning = perception module" plan into
concrete steps for **this repository**. It supersedes the generic advice in
earlier discussions wherever the two conflict. The authoritative contracts are:

- `config/terrain_classes.yaml` — class IDs, costs, lethal flags, thresholds
- `docs/terrain-class-schema.md` — schema policy and compatibility boundary
- `docs/dataset-tooling.md` — dataset layout, mask and metadata contracts
- `docs/ml-training.md` — training, evaluation, export, acceptance gates
- `docs/ros-ml-inference.md` — ROS inference node contract
- `docs/goal-level-implementation-plan.md` — overall goal-level plan (§5–§7 cover this guide)

## 0. Current status (verified in the working tree)

| Component | State | Evidence |
|---|---|---|
| 9-class terrain schema (0 unknown … 8 habitat) | **Done** | `config/terrain_classes.yaml`, `tools/validate_terrain_config.py` |
| Dataset format, validator, world-split tool, report | **Done** | `ml/collect_dataset.py`, `ml/validate_dataset.py`, `ml/split_dataset.py`, `ml/report_dataset.py` |
| PyTorch U-Net / DeepLabV3+ / SegFormer-B0 code | **Done (code only)** | `ml/lunabot_ml/models.py` |
| Training, evaluation, ONNX export, smoke inference scripts | **Done (code only)** | `ml/train.py`, `ml/evaluate.py`, `ml/export_onnx.py`, `ml/infer_image.py` |
| ROS ONNX inference node + contract | **Done (code only)** | `src/lunabot_perception/lunabot_perception/terrain_inference_node.py`, `inference_core.py` |
| Static validation baseline | **Green** | `python3 tools/validate_all.py --no-phase-validators --no-write` → 187 passed, 0 failed, 12 skipped |
| Camera-view semantic ground truth from Gazebo | **Missing (blocker)** | Rover model defines `rgb_camera`, `depth_camera`, `lidar` only (`src/lunabot_gazebo/models/lunabot_v4/model.sdf`). No segmentation sensor. |
| Capture script that writes RGB/depth/mask/metadata from the simulator | **Missing** | `ml/collect_dataset.py` only copies files you already have |
| Dataset with ≥5,000 valid samples over ≥30 worlds | **Missing** | `datasets/` is not present |
| Trained weights, `models/terrain_segmentation.onnx` | **Missing** | `models/README.md` only; torch is not installed in this sandbox, so no training has been run here |

**Bottom line:** the software skeleton exists. The critical path is the
simulator-side labelled data, then training, then acceptance evaluation.

## 1. Corrections to the earlier plan

| Earlier suggestion | What this repository does / should do | Why |
|---|---|---|
| Train 5 classes (bedrock, regolith, rock, crater, shadow) | Use the 9-class schema: `unknown, flat_regolith, rough_regolith, bedrock, small_rock, large_rock, crater, shadow, habitat` | The schema is already validated and consumed by the ROS node. Changing IDs later requires a version migration. |
| Use a 3-class Kaggle dataset as the main training set | Use it at most for optional pre-training. Primary data = Gazebo-rendered, labelled samples. | Kaggle's dataset has only sky / small rock / large rock labels, and its renders are not from your rover camera. |
| U-Net + ResNet34 encoder | Start with the repo's `unet` (base_channels 32, no pretrained encoder). Add a ResNet encoder only as a later, measured experiment. | The repo has no ResNet encoder, and `ml/requirements.txt` has no torchvision. The baseline must come first. |
| Deploy `best_model.pth` to ROS | Export to **ONNX** and ship `terrain_segmentation.onnx` + `.config.json` + `.sha256` in `models/`. | ROS uses ONNX Runtime with checksum and config checks. |
| ROS topics `/terrain/segmentation`, `/planner` | Use the repo topics: `/lunabot/camera/image_raw`, `/lunabot/camera/camera_info`, `/lunabot/terrain/class_image`, `/lunabot/terrain/confidence`, `/lunabot/terrain/overlay`, `/lunabot/terrain/inference_status` | Already implemented and documented. |
| Cost values (1, 4, 8, 20, 30 …) | Use `config/terrain_classes.yaml` (10, 45, 25, 85, 100 …) and tune them by experiment | Single source of truth. Costs are policy, not measured facts. |
| Accuracy / Dice / IoU as "good enough" | Use the acceptance gates in §8 | Held-out-world evaluation, recall for hazard classes. |
| "Kaggle is the training environment" | Optional. Everything runs locally with `.venv-ml`, and Kaggle or a cloud GPU is fine if the same scripts and config hashes are used. | Reproducibility is enforced by config hashing and the dataset manifest, not by the platform. |

Facts I checked: the Kaggle "Artificial Lunar Landscape" dataset contains
9,766 rendered lunar landscapes with segmented equivalents, and the 3 segmentation
classes are sky, smaller rocks and larger rocks, per the
[LunarX-Alpha lunar-segmentation README](https://github.com/LunarX-Alpha/lunar-segmentation).
Check the dataset's own licence on Kaggle before using it.

## 2. Blocker: camera-view semantic labels from Gazebo

The model must learn from **what the rover camera sees**, with a label per
pixel. Right now the simulator only produces:

- `tools/generate_lunar_terrain.py --semantic-mask-output`: a **top-down** world
  mask, not a camera image;
- rover RGB and depth topics, with no label stream.

You need a per-frame label image aligned with `/lunabot/camera/image_raw`. Options, in order of preference:

1. **Gazebo segmentation camera sensor.** Add a second camera on the same
   `sensor_head` pose with the same 640×480 and 1.047 rad FOV as `rgb_camera`.
   Check first that your installed gz-sensors version supports a segmentation
   camera (`type="segmentation"`) and that you can map its output to class IDs.
   Requirements: nearest-neighbour output with no colour blending, and a stable
   entity→class table generated when the world is built. Bridge it to
   `/lunabot/ground_truth/semantic` (evaluation/dataset-only topic).
2. **Projection from depth + top-down truth.** Use the depth image,
   `CameraInfo` intrinsics and the camera pose to back-project each pixel to
   world XY, then look up the top-down semantic grid. Objects (rocks, habitat)
   need their own label layer because they are not in the terrain raster.
   This is slower but works on any Gazebo version.
3. **Colour-coded duplicate materials.** Temporarily swap materials to unique
   flat colours, render, and decode. Most fragile; avoid unless 1 and 2 fail.

Whichever you choose, validate alignment before collecting data at scale, as
`docs/simulation-ground-truth.md` requires. Overlay the mask on the RGB frame
and check that rock and crater edges line up.

## 3. Environments

Simulator workstation (ROS 2 Humble, Gazebo Fortress, per `README.md`):

```bash
cd ~/lunabot-v4
colcon build --symlink-install   # from the ROS workspace layout in docs/ros-package-foundation.md
```

ML workstation (can be the same machine, but use a separate venv):

```bash
cd ~/lunabot-v4
python3 -m venv .venv-ml
.venv-ml/bin/pip install -r ml/requirements.txt
# On a GPU machine, install a CUDA-enabled torch wheel first if the default is wrong for your driver.
```

Keep `numpy<2` and `ml-dtypes<0.6`. ROS 2 Humble's `cv_bridge` needs the NumPy 1.x ABI.

## 4. Capture a labelled dataset

### 4.1 Generate varied worlds

```bash
# one world per seed; vary density and roughness across worlds
for seed in $(seq 1 30); do
  W=/tmp/lunabot-worlds/world$(printf %02d $seed)
  mkdir -p $W/{meshes,evidence,truth}
  python3 tools/generate_lunar_terrain.py --seed $seed --crater-density 1.0 --no-previews \
    --mesh-output-dir $W/meshes --evidence-dir $W/evidence \
    --metadata-output $W/truth/terrain.json --semantic-mask-output $W/truth/semantic.png
done
```

Vary the seeds, crater densities (0.1–2.0), rock density and illumination
(`config/scenarios/*.json` already has `low_sun` and `dynamic_obstacle`). Aim for
at least 30 worlds (roughly 70/15/15 train/validation/test by world).

### 4.2 Capture synchronised samples (to be written)

No capture tool exists yet. Write `tools/capture_dataset_samples.py` (or a ROS node) that, for each
rover pose along a random or waypoint trajectory:

1. waits for RGB, depth and label images with the same simulation time;
2. writes `rgb.png` (8-bit RGB), `depth.png`, `mask.png` (**8-bit grayscale, values 0–8 only**);
3. writes metadata JSON with every field `docs/dataset-tooling.md` requires:
   `rgb_timestamp_ns`, `depth_timestamp_ns`, `mask_timestamp_ns`, `camera_intrinsics`
   (`fx, fy, cx, cy`), `camera_pose` and `rover_pose` as `[x,y,z,qx,qy,qz,qw]`,
   and `lighting`;
4. calls `ml/collect_dataset.py` for each sample:

```bash
python3 ml/collect_dataset.py \
  --dataset datasets/lunabot \
  --sample-id world01-frame000001 \
  --world-id world01 \
  --rgb  /tmp/capture/rgb/000001.png \
  --depth /tmp/capture/depth/000001.png \
  --mask /tmp/capture/mask/000001.png \
  --metadata /tmp/capture/meta/000001.json
```

Use sync tolerance ≤ 50 ms. Record a rosbag of the same run for later replay
tests (`docs/goal-level-implementation-plan.md` §2 lists the topics).

Pitfalls to avoid:

- Do not save palette/colour PNGs as masks. The validator rejects them on purpose.
- Do not use simulator pose or label topics in navigation. They are for dataset generation and evaluation only.
- Keep the sample count honest: 5,000 valid samples is the **gate**, not a
  claim you can make from a few hundred frames.

### 4.3 Validate, split, report

```bash
python3 ml/validate_dataset.py --dataset datasets/lunabot --output results/dataset-validation.json
python3 ml/split_dataset.py --dataset datasets/lunabot --seed 42
python3 ml/report_dataset.py --dataset datasets/lunabot --output results/dataset-report.md
```

Gate: zero corrupt or mismatched files, every class 0–8 present (the report
shows the histogram), and **no world appears in more than one split**.

## 5. Optional: Kaggle pre-training

Only do this after the baseline in §6 exists, and only if the baseline is data-starved.

- The Kaggle masks must be converted to 8-bit grayscale IDs first: sky → 0
  (`unknown`, which the loss ignores), small rocks → 4, large rocks → 5. Other
  terrain classes are absent from this dataset.
- `ml/train.py` has no weight-initialisation option today. Add one, for
  example `--init-from`, that loads all tensors whose shapes match and skips the
  9-way head. Document it in the config hash so that pre-trained runs are
  distinguishable.
- Fine-tune on the Gazebo dataset afterwards. Compare against the scratch baseline
  on the same held-out worlds; keep pre-training only if it helps.

## 6. Train the baseline

Note: `ml/configs/unet.yaml` sets `data.dataset` to `datasets/lunabot_semantic`,
but the docs use `datasets/lunabot`. Pass `--dataset` explicitly (see §9).

```bash
.venv-ml/bin/python ml/train.py --config ml/configs/unet.yaml \
  --dataset datasets/lunabot --output results/unet --device cuda
```

What it does: seeded deterministic training; weighted cross-entropy + 0.5 × Dice,
with `ignore_index=0`; AdamW; early stopping on validation loss; `best.pt` saved
with the config hash. The three architectures
(`unet`, `deeplabv3plus`, `segformer_b0`) use the same loop, so they can be compared
directly. Input is 256×256 RGB normalised with the config mean/std.

Recommended order (from `docs/goal-level-implementation-plan.md` §6): U-Net first as
the correctness baseline, then DeepLabV3+, then SegFormer-B0. Select by safety-critical
recall and mIoU, not pixel accuracy.

## 7. Evaluate on held-out worlds only

```bash
.venv-ml/bin/python ml/evaluate.py --config results/unet/config.json \
  --checkpoint results/unet/best.pt --dataset datasets/lunabot \
  --output results/unet/test-report.json --device cuda
```

Read: per-class IoU / precision / recall / F1, confusion matrix, `hazard_false_negative_rate`
(classes 5 and 6), latency and FPS. Save success and failure images next to the
report. See §9 for a metric issue to fix before trusting `mean_iou`.

## 8. Export and check

```bash
.venv-ml/bin/python ml/export_onnx.py --config results/unet/config.json \
  --checkpoint results/unet/best.pt --output models/terrain_segmentation.onnx
.venv-ml/bin/python ml/infer_image.py --model models/terrain_segmentation.onnx \
  --config models/terrain_segmentation.config.json --image sample.png --output prediction.png
sha256sum -c models/terrain_segmentation.sha256
```

Add an **ONNX parity check** before release: run the same tensor through PyTorch
and ONNX Runtime and confirm the argmax agrees (the exporter does not do this today).

### Acceptance gates (from `docs/ml-training.md`)

| Gate | Threshold |
|---|---|
| mIoU, held-out worlds | ≥ 0.65 |
| Recall, large rock (5) and crater (6) | ≥ 0.90 each |
| Speed on deployment hardware | ≥ 10 FPS |
| World leakage | none |
| Live ROS vs offline predictions on the same bag | match within numeric tolerance |

Do not state any of these as achieved until the report and the target-hardware benchmark exist.

## 9. Integrate with ROS

Place the release artifacts (`.onnx`, `.config.json`, `.sha256`) somewhere the
launch file can read, not in Git, and start the existing node:

```bash
ros2 launch lunabot_bringup perception.launch.py perception_mode:=ml \
  model_path:=/models/terrain_segmentation.onnx \
  model_config:=/models/terrain_segmentation.config.json \
  model_checksum:=/models/terrain_segmentation.sha256 \
  inference_device:=auto
```

Check outputs:

```bash
ros2 topic hz /lunabot/terrain/class_image
ros2 topic echo /lunabot/terrain/inference_status --once
```

The node rejects a missing, tampered or incompatible model: it stays alive, emits an
ERROR status, and publishes no fabricated class map. That is intentional.

Next layers, already scaffolded but to be validated with the trained model: semantic
fusion (`src/lunabot_mapping`), layered traversability (`config/terrain_classes.yaml`
costs, `src/lunabot_mapping/config/traversability.yaml`), and D*-Lite
(`src/lunabot_planning`). Do not tune the planner on ground-truth masks if the final
claim uses predicted masks (`docs/goal-level-implementation-plan.md` §17).

## 10. Known issues found during this review

These are not fixed in this change. Each needs a decision or a small follow-up.

1. **`mean_iou` includes the ignored `unknown` class.** Training and the loss ignore
   class 0 (`ignore_index=0`), but `metrics.report` still computes its IoU. Any false
   positive on class 0 yields IoU 0 and pulls the mean down. Example from a local
   check: truth `[1,1,1,5,5,6,6]`, prediction `[1,1,0,5,5,6,6]` gives `mean_iou = 0.667`,
   which includes class 0 at 0.0. Excluding class 0 gives 0.889. Decide whether the
   gate uses 8 classes (1–8) or 9, then fix `metrics.py` and add a test.
2. **`class_weights[0]` is unused.** The config gives class 0 weight 0.25, but class 0
   is ignored in cross-entropy. Either drop it or stop ignoring class 0.
3. **Dataset path mismatch.** `ml/configs/*.yaml` use `datasets/lunabot_semantic`, but
   the docs use `datasets/lunabot`. Standardise one.
4. **Confidence thresholds are duplicated.** `inference_core.py` hard-codes the
   per-class thresholds. They should be read from `config/terrain_classes.yaml` so a
   change in one place cannot drift from the other.
5. **Aspect ratio.** The camera is 640×480 but training and inference resize to
   256×256, which distorts the image. Acceptable for a first baseline; test a
   16:9-compatible size (for example 256×192 or 384×288) before finalising.
6. **No ONNX parity check** (see §8).
7. **Training and evaluation have never run here** (no torch in this environment),
   so every number in this guide is a process description, not a result.

## 11. Definition of done for the deep-learning module

- `datasets/` contains ≥5,000 valid, world-split samples across 30+ worlds, with a
  clean validation report and the labels visible in simulator-camera overlays.
- `results/<model>/test-report.json` exists, with held-out metrics and the
  config hash that produced them.
- `models/terrain_segmentation.onnx`, its config and checksum pass
  `sha256sum -c` and the parity check.
- The ROS `perception_mode:=ml` path publishes class, confidence and overlay topics
  at the target FPS, and offline and live predictions agree on a recorded bag.
- The final write-up quotes only numbers from saved reports and benchmarks.
