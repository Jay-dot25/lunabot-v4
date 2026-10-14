# Phase 2 — Eyes of Luna

Phase 2 is authorized. It uses the existing Phase 1 rover/camera; this directory
contains the phase-specific contract and acceptance record, not a second ROS
workspace or simulator. The Phase 1 camera interface is `/camera/image_raw`,
640×480 RGB (`R8G8B8`) at 20 Hz.

## Current status

The repository does **not** contain a usable annotated camera dataset or model
checkpoint. Its two PNGs are terrain-preview images, not camera frames or
annotations. The existing `ml/` training and `src/lunabot_perception/` inference
scaffolds use a different nine-class, single-output schema; they are not a
Phase 2 model and cannot be accepted as one. The agent host also lacks PyTorch,
ROS 2, Gazebo, NumPy, Pillow, and ONNX Runtime.

This increment implements and tests only a dependency-free Phase 2 data
contract, collection/validation utilities, and capture-group splits. Unit tests
use throwaway synthetic PNGs to test file handling only; those fixtures are not
training data and supply no quality or runtime evidence. No model was trained,
no metrics or predictions were generated, and no ROS/Gazebo integration was
claimed. See [PHASE2_ACCEPTANCE.md](PHASE2_ACCEPTANCE.md).

## Output contract

Each frame has two separately stored masks. Shadow is **not** a terrain class
and may overlap any terrain pixel.

| Output | Encoding | Meaning |
|---|---|---|
| Terrain mask | 8-bit grayscale PNG; IDs `0–3`; `255` = ignore/uncertain | `0 regolith`, `1 bedrock`, `2 rock/boulder`, `3 crater` |
| Shadow mask | 8-bit grayscale PNG; IDs `0–1`; `255` = ignore/uncertain | `0 not-shadow`, `1 shadow` |

`ml/configs/eyes_of_luna.json` records the source camera and independent output
heads: four-class terrain softmax and one-logit binary shadow sigmoid. It is a
data/output contract, **not** a trained model configuration or evidence of a
checkpoint.

## Dataset layout and tools

Use `datasets/eyes_of_luna/` (ignored by Git):

```text
datasets/eyes_of_luna/
  manifest.json
  rgb/<sample-id>.png
  terrain_masks/<sample-id>.png
  shadow_masks/<sample-id>.png
  metadata/<sample-id>.json
  splits/capture_group_split.json
```

Register a real frame only after both masks are reviewed:

```bash
python3 ml/collect_eyes_of_luna_sample.py \
  --dataset datasets/eyes_of_luna \
  --sample-id world01-frame000001 \
  --group-id world01-session01 \
  --rgb /path/to/frame.png \
  --terrain-mask /path/to/terrain.png \
  --shadow-mask /path/to/shadow.png \
  --metadata /path/to/reviewed-frame.json
```

The metadata must record the exact camera topic, image timestamp, world/session,
Gazebo/simulator and renderer versions, lighting, camera pose, and simulator
ground-truth rover pose. If odometry is available, record it separately as
`rover_pose_odom`; never substitute odometry for simulator ground truth or vice
versa. Each head must record its annotation method and a `human_reviewed`
status, reviewer ID, and review timestamp. Semantic-camera annotations must also
record implementation, version, and class mapping. The validator verifies the
metadata declaration and file integrity; it cannot prove that a review occurred
or that pixels are semantically correct.

Validate files, dimensions, checksums, class IDs, and metadata:

```bash
python3 ml/validate_eyes_of_luna_dataset.py \
  --dataset datasets/eyes_of_luna \
  --output results/eyes-of-luna-dataset-validation.json
```

Split by whole capture group, not by frame, so correlated frames cannot cross
train/validation/test boundaries. At least three independent groups are needed
for non-empty three-way splits:

```bash
python3 ml/split_eyes_of_luna_dataset.py \
  --dataset datasets/eyes_of_luna --seed 42
```

The generated split is deterministic. Inspect the returned group assignment and
per-class coverage before model work; a valid split alone does not show that the
held-out set has adequate examples of every class.

## Capture and annotation workflow

1. Use the existing Phase 1 launch and capture frames from `/camera/image_raw`;
   preserve timestamps and group frames by a whole world/scene or capture
   session. Do not start a duplicate simulator just for Phase 2.
2. Record lighting, camera pose, simulator ground-truth rover pose, and—if
   available—wheel odometry as a distinct value. Record the simulator/rendering
   version and source camera topic with each capture.
3. Until an aligned semantic-camera implementation is verified on the target
   Gazebo version, annotate the RGB frames manually. The Phase 1 SDF currently
   configures an ordinary RGB camera, not a semantic camera.
4. Create two aligned, non-interlaced, 8-bit grayscale PNGs per frame. Terrain
   IDs are `0–3`; shadow IDs are `0–1`; use `255` for ignored/uncertain pixels.
   Do not derive or guess shadow labels from terrain labels.
5. A human must review **both** masks against the RGB frame before registration.
   Record the tool/method and review provenance. If a semantic camera is later
   used, preserve its implementation and mapping and still review its masks.
6. Run validation, then create a capture-group split. Keep original data and
   split manifests immutable for reproducibility; do not tune a model on the
   test groups.

## Work deliberately not claimed yet

Training, fine-tuning, held-out metrics, prediction artifacts, standalone
inference, ROS 2 publication, live Gazebo camera execution, and performance
measurements remain blocked. The present nine-class/single-head model and
inference node need a deliberate multi-head adaptation; the root inference node
also has a different camera-topic assumption. No Phase 2 acceptance gate is
passed by the presence of code or by these unit tests.
