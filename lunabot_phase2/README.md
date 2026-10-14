# Phase 2 — Eyes of Luna

Phase 2 is authorized. It uses the existing Phase 1 rover/camera; this directory
contains the phase-specific contract and acceptance record, not a second ROS
workspace or simulator. The Phase 1 camera interface is `/camera/image_raw`,
640×480 RGB (`R8G8B8`) at 20 Hz.

## Current status

The repository has **no reviewed camera dataset, trained checkpoint, held-out
evaluation, or ROS runtime result**. The two existing terrain-preview PNGs are
not camera frames or annotations. The prior nine-class, single-output pipeline
was not reused as a Phase 2 model.

Implemented incrementally: a compact four-class terrain head plus an independent
binary shadow head; reviewed-mask dataset validation and capture-group split
loading; ignore-aware losses; CPU/GPU-capable train/evaluate/inference tools;
checksum-bound `torch.export` deployment bundles; and a separate ROS 2 image
node/launch file for `/camera/image_raw`. Local CPU tests check shapes, loss,
serialization, mask IDs, dataset contracts, and static ROS package contracts.
They use temporary synthetic fixtures and are not model-quality evidence. The
original perception package build passed on the target (user-reported), but the
new optional capture-world launch/assets must be rebuilt and exercised there.
No reviewed dataset, training outputs, prediction visuals, metrics, or live
inference result exist. **Phase 2 is NOT PASSED.** See
[PHASE2_ACCEPTANCE.md](PHASE2_ACCEPTANCE.md).

## Output contract

Each frame has two separately stored masks. Shadow is **not** a terrain class
and may overlap any terrain pixel.

| Output | Encoding | Meaning |
|---|---|---|
| Terrain mask | 8-bit grayscale PNG; IDs `0–3`; `255` = ignore/uncertain | `0 regolith`, `1 bedrock`, `2 rock/boulder`, `3 crater` |
| Shadow mask | 8-bit grayscale PNG; IDs `0–1`; `255` = ignore/uncertain | `0 not-shadow`, `1 shadow` |

`ml/configs/eyes_of_luna.json` records the source camera, output heads, U-Net
architecture defaults, input size, normalization, and provisional shadow
threshold. These settings are unvalidated against real data; they are not a
trained model or evidence of a checkpoint.

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
set -euo pipefail
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

## Extract frames from a ROS 2 bag

After a Phase 1 bag has been recorded, source the Humble environment and extract
an initial inspection subset. Run from the repository root after the Phase 2
code is synced; the exporter uses the target's `rosbag2_py` and preserves the
source header timestamp, frame ID, width, height, encoding, step, and bag receive
timestamp in capture sidecars.

```bash
set -euo pipefail
source /opt/ros/humble/setup.bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
CAPTURE_ROOT="$HOME/eyes_of_luna_capture"
SESSION="$(cat "$CAPTURE_ROOT/active_session.txt")"
python3 "$REPO_ROOT/ml/extract_eyes_of_luna_frames.py" \\
  --bag "$CAPTURE_ROOT/$SESSION" \\
  --output "$CAPTURE_ROOT/${SESSION}_extracted" \\
  --group-id "$SESSION" \\
  --every-n 30
```

This creates RGB PNGs in `rgb/`, provenance sidecars in
`capture_metadata/`, and `capture_manifest.json`. The sidecars are deliberately
marked `pending_human_review` and are **not ready for dataset registration**:
fill verified world/session, simulator/renderer, lighting, camera and rover
poses from capture evidence, annotate and review both masks, then create the
collector metadata. Check the extracted frames for visible class diversity
before deciding whether to capture more sessions. `--every-n 30` is an initial
sampling choice, not an accepted dataset sampling rate.

## Train, evaluate, and run standalone inference

Use only a reviewed dataset with a valid capture-group split. Run the following
from the repository root. These commands use the configured CPU path; `run01` is
the chosen output directory, not an existing artifact. The current checkout has
no dataset, so these commands have not produced a checkpoint here.

```bash
python3 ml/validate_eyes_of_luna_dataset.py \
  --dataset datasets/eyes_of_luna \
  --output results/eyes_of_luna/run01/dataset_validation.json
python3 ml/split_eyes_of_luna_dataset.py \
  --dataset datasets/eyes_of_luna --seed 42
python3 ml/train_eyes_of_luna.py \
  --dataset datasets/eyes_of_luna \
  --output results/eyes_of_luna/run01 \
  --device cpu
python3 ml/evaluate_eyes_of_luna.py \
  --dataset datasets/eyes_of_luna \
  --checkpoint results/eyes_of_luna/run01/best.pt \
  --output results/eyes_of_luna/run01/test_metrics.json \
  --device cpu
FRAME="$(find datasets/eyes_of_luna/rgb -maxdepth 1 -type f -name '*.png' -print -quit)"
test -n "$FRAME" || { echo "No RGB frames found in the dataset" >&2; exit 2; }
python3 ml/infer_eyes_of_luna_image.py \
  --checkpoint results/eyes_of_luna/run01/best.pt \
  --image "$FRAME" \
  --image-id "$(basename "$FRAME" .png)" \
  --output-prefix results/eyes_of_luna/run01/visuals/frame01 \
  --device cpu
```

Training fails closed on missing/invalid data, incomplete class coverage, or missing
capture groups; it does not generate synthetic annotations. Evaluation enforces
the saved manifest/split checksums and uses only the test capture groups. The
report's metrics are meaningful only if those labels and splits are valid and
representative. Evaluation prediction records and filenames retain each dataset
`sample_id`; standalone inference writes `image_id` (supplied with `--image-id`,
or defaulted to the filename stem) and original dimensions to its JSON report.
The `.terrain.png` and `.shadow.png` outputs preserve class IDs (`0–3` and
`0/1` respectively); `.shadow_visualization.png` is a separate `0/255` preview,
not a machine mask.

## Optional Phase 2 terrain-capture worlds

The Phase 1 base-camp world contains a flat regolith lane, rocks and obstacles;
it has no explicit bedrock or crater geometry. To collect more varied RGB data
without changing the Phase 1 world or `launch-1`, the optional Phase 2 package
now installs three separate Gazebo Fortress capture worlds: `scene01`,
`scene02`, and `scene03`. Each has a cratered regolith height mesh, a broad
faceted bedrock outcrop, irregular boulders, and directional lighting that
casts shadows. Each world includes the same rover/camera model and publishes
`/camera/image_raw` and `/camera/camera_info`.

These worlds are **synthetic simulator scenes**, not verified semantic cameras
and not ground-truth masks. They provide candidate visual examples only; make
both terrain and shadow masks manually and have a person review them. The SDF
terrain geometry and model names must not be used to claim pixel-label accuracy.
Target rendering, mesh collision behavior, camera framing, and rover traversal
have not yet been tested on Fortress; inspect each scene before recording.

Build the optional package and its workspace dependencies from the Phase 1
workspace root:

```bash
REPO_ROOT="$HOME/lunabot-v4-humble.tDEKWr/repo"
source /opt/ros/humble/setup.bash
cd "$REPO_ROOT/lunabot_phase1"
colcon build --symlink-install --packages-up-to lunabot_phase2_perception \\
  --event-handlers console_direct+
source install/setup.bash
```

Stop any existing `launch-1`/Phase 1 simulator before starting one of the
separate capture worlds. Pick one scene per capture group:

```bash
ros2 launch lunabot_phase2_perception eyes_of_luna_capture.launch.py scene:=scene01
# Also available: scene:=scene02 or scene:=scene03
```

The launch starts Fortress, the camera/odometry/TF bridges, static camera TF,
and the existing Phase 1 command guard. It does **not** start Phase 2 inference
or an autonomous controller. In another sourced Humble terminal, start manual
teleop (use short movements and stop between maneuvers):

```bash
REPO_ROOT="$HOME/lunabot-v4-humble.tDEKWr/repo"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/lunabot_phase1/install/setup.bash"
ros2 run lunabot_phase1_tools keyboard_teleop
```

Record the camera, calibration, ordinary wheel odometry, TF, and command topics
in one terminal, and the Gazebo simulator pose stream in a separate terminal.
Do not substitute `/odom` for simulator ground truth. Start both logs before
moving the rover; stop both with Ctrl+C. For `scene01`, for example:

```bash
REPO_ROOT="$HOME/lunabot-v4-humble.tDEKWr/repo"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/lunabot_phase1/install/setup.bash"
CAPTURE_ROOT="$HOME/eyes_of_luna_capture"
GROUP_ID="scene01"
mkdir -p "$CAPTURE_ROOT"
test ! -e "$CAPTURE_ROOT/$GROUP_ID" || { echo "Bag path already exists" >&2; exit 2; }
ros2 bag record -o "$CAPTURE_ROOT/$GROUP_ID" \\
  /camera/image_raw /camera/camera_info /odom /tf /tf_static /cmd_vel /cmd_vel_sim
```

In a second terminal, capture Fortress dynamic poses to a distinct file:

```bash
source /opt/ros/humble/setup.bash
CAPTURE_ROOT="$HOME/eyes_of_luna_capture"
GROUP_ID="scene01"
WORLD="eyes_of_luna_capture_scene01"
POSE_LOG="$CAPTURE_ROOT/${GROUP_ID}_gazebo_ground_truth.txt"
test ! -e "$POSE_LOG" || { echo "Pose log already exists" >&2; exit 2; }
ign topic -e -t "/world/$WORLD/dynamic_pose/info" > "$POSE_LOG"
```

Use `GROUP_ID=scene02` / `scene03` and their corresponding world names for the
other layouts. One bag per scene group is recommended; keep all frames from a
scene together in one split. The pose log and rosbag are not yet automatically
time-aligned, so preserve both and verify alignment before metadata registration.
Then extract frames as usual:

```bash
REPO_ROOT="$HOME/lunabot-v4-humble.tDEKWr/repo"
CAPTURE_ROOT="$HOME/eyes_of_luna_capture"
GROUP_ID="scene01"
python3 "$REPO_ROOT/ml/extract_eyes_of_luna_frames.py" \\
  --bag "$CAPTURE_ROOT/$GROUP_ID" \\
  --output "$CAPTURE_ROOT/${GROUP_ID}_extracted" \\
  --group-id "$GROUP_ID" --every-n 30
```

Collect one reviewed contact sheet per scene before creating masks. Keep the
existing `session01` and 2026-10-14 captures intact; they remain useful examples
of the original base-camp appearance but do not visibly cover all four terrain
classes.

## Optional ROS 2 integration

`lunabot_phase1/src/lunabot_phase2_perception` is an optional `ament_python`
package included in the Phase 1 Humble workspace. Building it does not start or
change the Phase 1 simulator. Runtime also requires compatible NumPy, Pillow,
and PyTorch to be importable by the same Python interpreter used by ROS 2; this
has not been checked on the target, and no system packages were installed here.
After building the workspace and creating a real training bundle, run this from
the repository root in a Humble terminal with the Phase 1 workspace sourced:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
export EYES_OF_LUNA_BUNDLE="$REPO_ROOT/results/eyes_of_luna/run01/deployment.json"
ros2 launch lunabot_phase2_perception eyes_of_luna.launch.py \\
  model_bundle_path:="$EYES_OF_LUNA_BUNDLE" \\
  image_topic:=/camera/image_raw device:=cpu
```

The bundle path here is produced by the `run01` training command above. The node
preserves the source header on both masks and publishes:

- `/eyes_of_luna/terrain_mask`: `sensor_msgs/Image`, `mono8`, terrain IDs `0–3`.
- `/eyes_of_luna/shadow_mask`: `sensor_msgs/Image`, `mono8`, shadow IDs `0/1`.
- `/eyes_of_luna/status`: JSON `std_msgs/String`, including state, device,
  latency, average FPS, dropped frames, source dimensions/encoding, and header
  timestamp/frame ID.

The decoder accepts 8-bit `rgb8`/`R8G8B8`, BGR, RGBA/BGRA, and mono inputs;
unknown encodings fail the frame safely. `sensor_msgs/Image` has no separate
sample-ID field, so the original timestamp and `frame_id` are copied into both
output headers and echoed in status; dataset `sample_id` is retained by the
standalone/evaluation tools. The node publishes perception outputs only and
does not command or control the rover.

Without a loadable deployment bundle, the node reports `ERROR` and does not
publish masks. Neither a successful colcon build nor synthetic unit tests are a
ROS runtime test; target build, live camera subscription, message metadata,
resource use, and latency still need to be exercised.

## Work deliberately not claimed yet

No real model has been trained, no held-out metrics or prediction results exist,
and the ROS package has not been built or run on the target. Model quality,
visual accuracy, FPS/resource suitability, and Phase 2 acceptance remain
unverified. No navigation, localization, semantic mapping, traversal-cost,
planning, autonomous driving, or dashboard work is in Phase 2 scope.
