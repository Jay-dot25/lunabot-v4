# Phase 2 — Eyes of Luna acceptance record

**Overall status: NOT PASSED.** Phase 2 is authorized, but there is no usable
annotated camera dataset, trained checkpoint, held-out prediction/metric report,
or tested live ROS 2/Gazebo inference pipeline. No model quality or deployment
performance is claimed.

| Section | Criterion / evidence | Status |
|---|---|---|
| **A. Phase 1 prerequisite** | The Phase 1 acceptance record says `PASSED (user-reported)`. This is not an independent agent-host reproduction. The target test for the newer `launch-1` wrapper remains separate and pending. | **PASS (user-reported)** for the reported direct Phase 1 launch; wrapper **NOT VERIFIED** |
| **B. Available environment** | Agent host is Debian 12 / Python 3.11.2 with 2 CPUs and about 4 GB RAM. `ros2`, `ign`, `gz`, `colcon`, `nvidia-smi`, PyTorch, NumPy, Pillow, OpenCV, ONNX Runtime, `rclpy`, and `cv_bridge` are absent. The Phase 1 target was reported as Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 6.18.0; Phase 2 ML/runtime dependency availability on that target is not established. | **BLOCKED / NOT VERIFIED** |
| **C. Camera and data inventory** | Phase 1 SDF declares ordinary RGB at `/camera/image_raw`, 640×480, `R8G8B8`, 20 Hz. On 2026-10-14, user terminal output showed the live-interface smoke test receiving `/camera/image_raw` and `/camera/camera_info`; `ros2 topic hz` observed about 11.4–11.9 Hz under that run. This confirms a live message stream, not a verified 20 Hz rate or a viewed image. Repository inventory found only two terrain-preview PNGs; no annotated camera images, dual masks, Phase 2 manifest, or capture groups exist. | Camera stream **PASS (user-reported live smoke)**; observed rate **below configured 20 Hz**; dataset **BLOCKED** |
| **D. Annotation and provenance** | The Phase 1 camera is not a semantic camera. No valid terrain or shadow annotations were found. The new contract requires separate, human-reviewed terrain and shadow masks and records capture/annotation provenance. A metadata declaration cannot independently prove that a person reviewed or that labels are correct. | **BLOCKED** — real human-reviewed masks unavailable |
| **E. Dataset contract and collection/validation tooling** | Added `ml/configs/eyes_of_luna.json`, a camera/multi-output contract, and dependency-free collector/validator. Checks cover 8-bit RGB PNG decoding, two independent grayscale masks, dimensions, label IDs, checksums, capture metadata, and declared human review. `python3 -m unittest -v tests.test_eyes_of_luna_dataset`: **17 tests passed**. Fixtures are synthetic test-only PNGs, not training data. | **PASS (tooling only)**; no real dataset validation |
| **F. Train/validation/test split** | Added deterministic whole-capture-group splitting and leakage checks. Unit tests confirm reproducibility and no group/sample overlap on synthetic fixtures. No real split file exists; class coverage in held-out groups is not established. | **PASS (tooling only)**; real split **NOT VERIFIED** |
| **G. Model architecture and outputs** | Required contract is a four-class terrain softmax (`0 regolith`, `1 bedrock`, `2 rock/boulder`, `3 crater`) and a separate one-logit shadow sigmoid (`0 not-shadow`, `1 shadow`), with `255` ignored. Existing repository model is a mutually-exclusive nine-class single head and is incompatible. No Phase 2 model implementation/checkpoint was produced. | **NOT IMPLEMENTED / NOT VERIFIED** |
| **H. Training or fine-tuning** | No real labels, PyTorch environment, or Phase 2 training configuration/checkpoint are available. No synthetic or guessed data was used for training. | **BLOCKED** |
| **I. Held-out evaluation** | No held-out annotated images or trained checkpoint exist; there are no real predictions, confusion matrices, IoU/precision/recall, failure-case outputs, or test reports. | **BLOCKED** |
| **J. Standalone inference** | The existing ONNX core expects one nine-class logits output. It does not implement the Phase 2 dual-mask contract. No Phase 2 checkpoint/ONNX artifact exists to reload or test. | **BLOCKED / NOT IMPLEMENTED** |
| **K. ROS 2 and live Gazebo integration** | Phase 1 camera topic is statically known. This host has no ROS 2/Gazebo runtime, and the existing root perception node is not the requested Phase 2 dual-head implementation. No Phase 2 node, live topic outputs, or Gazebo camera smoke test is claimed. | **NOT VERIFIED** |
| **L. Performance and reliability measurements** | No model or deployment runtime was available; FPS, latency, memory, confidence/calibration, and failure behavior were not measured. | **NOT MEASURED** |
| **M. Acceptance decision** | Required evidence is absent: valid reviewed annotations, reloadable Phase 2 checkpoint, real held-out metrics/predictions, standalone inference, and tested live Gazebo camera pipeline. | **NOT PASSED** |

## Implementation and test evidence

- Dataset/output contract: `ml/configs/eyes_of_luna.json`.
- Data utilities: `ml/lunabot_ml/eyes_of_luna_dataset.py` and the three
  `ml/*eyes_of_luna*` command-line tools.
- Tests: `tests/test_eyes_of_luna_dataset.py` (17 passing); these verify schema
  and file-processing behavior only, not semantic accuracy.
- `python3 -m unittest discover -s tests`: **161 passed, 4 skipped**.
- `python3 tools/validate_all.py --no-phase-validators --no-write`:
  **195 passed, 0 failed, 12 skipped**. These static/unit checks do not exercise
  ROS 2, Gazebo, model quality, or target performance.
- Full training/inference acceptance remains gated on real annotated data and
  access to the ROS 2/Gazebo target. Do not treat source presence or unit tests
  as a passing model/runtime gate.

## Evidence required to resume

1. Supply or capture aligned Phase 1 RGB frames and separate terrain/shadow
   masks; every mask must receive human review and its provenance must be
   recorded. Preserve capture-group/world IDs and keep simulator ground-truth
   pose distinct from wheel odometry.
2. Run the dataset validator and group splitter on that real dataset; retain
   the manifest/checksums and verify class coverage in each split.
3. Provide a usable PyTorch training environment and target ROS 2/Gazebo access
   (or logs/artifacts from that target). Then adapt and test a separate-head
   model, train it, reload an independently saved checkpoint, and evaluate only
   on the held-out groups.
4. Integrate the resulting model with the existing `/camera/image_raw` stream,
   publish the two machine-readable masks, and record live smoke/performance
   evidence on the target.
5. Update this record only from observed evidence. Until all required gates pass,
   Phase 2 remains **NOT PASSED** and no later phase should begin.
