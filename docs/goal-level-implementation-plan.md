# LunaBot Goal-Level Implementation Plan

This document is the implementation plan for taking the current LunaBot V4
proof of concept to the complete project goal: trained multiclass lunar-terrain
perception, GPS-denied localization, semantic mapping, terrain-aware planning,
true incremental replanning, autonomous control, and quantitative evaluation.

> Do not replace the working A–L baseline all at once. Preserve it as a
> regression baseline and introduce each production subsystem behind a launch
> parameter. Every stage below has a measurable exit gate.

## 1. Target architecture

```text
Gazebo RGB ───────────────┐
Gazebo depth + CameraInfo ├─> ML segmentation ─> label + confidence images ─┐
Gazebo LiDAR ─> SLAM ─────┤                                                   │
Wheel odom + IMU ─> EKF ──┴─> robot pose / TF                                │
                                                                             v
LiDAR occupancy ─────────────────────────────────────────────> semantic fusion
Depth/elevation ─────────────────────────────────────────────> grid layers
                                                        occupancy | class |
                                                        confidence | slope |
                                                        roughness | age
                                                                    │
                                                                    v
                                                         traversability map
                                                                    │
Goal ────────────────────────────────────────────────────────────────┤
                                                                    v
                                             A* initial plan / D*-Lite repair
                                                                    │
                                                                    v
                                                  regulated path follower
                                                                    │
                                                                    v
                                            safety supervisor -> rover command
                                                                    │
                                                                    v
                                            metrics recorder and experiment DB
```

Only one node may publish the final rover command topic. The safety supervisor
must own `/cmd_vel`; navigation and teleoperation publish requests to separate
input topics.

## 2. Preserve the current baseline

Before feature work, run and archive the current system.

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_l.py
HEADLESS=1 DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 ./launch-l
```

Record the commit, console log, topic list, TF tree, rosbag, and final map. Do
not call the goal-level system complete if this baseline does not run.

Create a rosbag during a successful run:

```bash
ros2 bag record -o evidence/baseline \
  /clock /tf /tf_static /lunabot/odom /lunabot/imu \
  /lunabot/lidar/scan /lunabot/camera/image_raw /lunabot/depth/image_raw \
  /map /goal_pose /lunabot/terrain/cost_map /lunabot/terrain/plan \
  /cmd_vel_in /cmd_vel
```

## 3. Convert the repository into ROS 2 packages

The existing standalone scripts may continue to run during migration. Add these
packages under `src/`:

```text
src/
  lunabot_description/     rover description and TF
  lunabot_gazebo/          world, models, ground-truth plugins
  lunabot_bringup/         launch files and shared parameters
  lunabot_msgs/            structured messages
  lunabot_localization/    EKF and SLAM configuration
  lunabot_perception/      ML inference
  lunabot_mapping/         semantic/elevation fusion
  lunabot_planning/        A*, D*-Lite, traversability
  lunabot_control/         path follower and safety supervisor
  lunabot_evaluation/      metrics and experiment runner
```

Each package needs `package.xml`, `CMakeLists.txt` or `setup.py`, installed
executables, tests, and a README. Replace Bash process orchestration gradually
with Python ROS 2 launch files. Keep `launch-l` as a compatibility wrapper that
calls the final launch file.

Add structured messages instead of parsing status strings:

```text
lunabot_msgs/msg/TerrainPrediction.msg
lunabot_msgs/msg/PlannerStatus.msg
lunabot_msgs/msg/ReplanEvent.msg
lunabot_msgs/msg/MissionMetrics.msg
```

Minimum fields should include a `Header`, enum/state values, timestamps,
measured durations, counts, and explicit booleans. Keep old string status
topics temporarily for the existing validators.

**Exit gate:** `colcon build --symlink-install` succeeds and the current Phase L
behavior still runs from a ROS launch file.

## 4. Define the semantic classes and navigation policy

Create `config/terrain_classes.yaml` as the single source of truth. Start with:

| ID | Class | Initial cost | Lethal? |
|---:|---|---:|---|
| 0 | unknown | 85 | no, configurable |
| 1 | flat_regolith | 10 | no |
| 2 | rough_regolith | 45 | no |
| 3 | bedrock | 25 | no |
| 4 | small_rock | 85 | no |
| 5 | large_rock | 100 | yes |
| 6 | crater | 100 | yes |
| 7 | shadow | 75 | no, confidence-dependent |
| 8 | habitat | 100 | yes |

These are hypotheses, not final scientific values. Tune them through
experiments. Document the reason for every change.

Update these old files after the schema exists:

- `scripts/terrain_segmentation.py`: retain as `heuristic` fallback, but emit
  compatible class IDs or explicitly remain binary.
- `scripts/semantic_terrain_mapper.py`: remove hard-coded three-class logic.
- `scripts/terrain_cost_mapper.py`: load the YAML class-to-cost table.
- RViz configs: use a fixed color palette for all classes.

**Exit gate:** a synthetic unit test sends every class and verifies the exact
output cost and lethal/non-lethal behavior.

## 5. Create labeled lunar data

### 5.1 Add ground-truth semantics to Gazebo

Update:

- `src/lunabot_gazebo/worlds/lunar_world.sdf`
- `src/lunabot_gazebo/models/lunabot_v4/model.sdf`
- `tools/generate_lunar_terrain.py`

Required changes:

1. Generate terrain-region metadata alongside each mesh: terrain class,
   elevation, slope, roughness, and crater membership.
2. Give rocks, habitat, and obstacles stable semantic IDs.
3. Add/bridge `CameraInfo` for RGB and depth cameras.
4. Add a semantic ground-truth image source. Prefer a Gazebo segmentation
   camera/plugin; otherwise render unique class colors from duplicate hidden
   materials or project generated labels using camera pose and depth.
5. Publish ground truth on `/lunabot/ground_truth/semantic`.
6. Publish simulator pose on `/lunabot/ground_truth/pose` for evaluation only.
   Neither navigation nor SLAM may consume this topic.

### 5.2 Generate varied worlds

Extend `tools/generate_lunar_terrain.py` to accept and record:

```text
--seed --crater-density --rock-density --roughness --shadow-angle
--output-world --metadata-output
```

Generate at least 30 distinct worlds. Split by world, never by neighboring
frames:

- 70% training worlds
- 15% validation worlds
- 15% held-out test worlds

### 5.3 Record the dataset

Add:

```text
ml/collect_dataset.py
ml/validate_dataset.py
ml/split_dataset.py
ml/datasets/README.md
```

For every sample save:

```text
rgb/<sample>.png
depth/<sample>.png
mask/<sample>.png
metadata/<sample>.json
```

Metadata must include world seed, camera intrinsics, camera pose, rover pose,
timestamp, lighting parameters, and class histogram. Reject RGB/mask timestamp
mismatches and invalid dimensions.

Do not commit the dataset to ordinary Git. Use external object storage, DVC,
or Git LFS and commit only manifests/checksums.

**Exit gate:** at least 5,000 valid samples, every required class represented,
no train/test world leakage, and the dataset validator reports zero corrupt or
mismatched files.

## 6. Train the terrain model

Add this ML workspace:

```text
ml/
  requirements.txt
  configs/{unet,deeplabv3plus,segformer_b0}.yaml
  lunabot_ml/{dataset,augmentations,models,losses,metrics}.py
  train.py
  evaluate.py
  export_onnx.py
  infer_image.py
  tests/
models/README.md
```

Use PyTorch consistently unless a hard constraint requires TensorFlow.
Implement deterministic seeds and save the complete configuration with every
run.

Training sequence:

1. Train U-Net as the correctness baseline.
2. Train DeepLabV3+ with a lightweight and a ResNet backbone.
3. Train SegFormer-B0.
4. Use weighted cross-entropy plus Dice or focal loss for class imbalance.
5. Augment brightness, contrast, gamma, blur, noise, exposure clipping,
   shadows, and mild perspective changes.
6. Evaluate only on held-out worlds.
7. Select the model using safety-critical recall, mIoU, and inference latency;
   do not select by pixel accuracy alone.
8. Export the final model to ONNX or TorchScript and checksum it.

Required report:

- per-class IoU, precision, recall, F1;
- mean IoU and confusion matrix;
- large-rock/crater false-negative rate;
- model size, CPU/GPU latency, and FPS;
- representative success and failure images.

Suggested initial acceptance targets (revise with evidence):

- mIoU >= 0.65 on held-out simulated worlds;
- crater and large-rock recall >= 0.90;
- >= 10 FPS on deployment hardware;
- no test-world leakage.

**Exit gate:** `models/terrain_segmentation.onnx`, its config/checksum, and a
reproducible evaluation report exist.

## 7. Integrate model inference into ROS

Add `src/lunabot_perception/lunabot_perception/terrain_inference_node.py`.

Inputs:

- `/lunabot/camera/image_raw`
- `/lunabot/depth/image_raw` if the selected model uses depth
- `/lunabot/camera/camera_info`

Outputs:

- `/lunabot/terrain/class_image` (`mono8` class IDs)
- `/lunabot/terrain/confidence` (`32FC1`)
- `/lunabot/terrain/overlay` (`rgb8`)
- `/lunabot/terrain/inference_status`

Implement:

- `message_filters` synchronization;
- `cv_bridge` conversion;
- exact training normalization;
- CPU/CUDA/ONNX Runtime backend selection;
- confidence threshold to unknown;
- dropped-frame and latency counters;
- model/config compatibility checks;
- safe failure when the model is absent.

Update final launch files to select:

```text
perception_mode:=heuristic|ml|ground_truth
```

`ground_truth` is for debugging/evaluation only. Final results must use `ml`.

**Exit gate:** live predictions match offline inference, output is stable at the
required FPS, and a rosbag replay produces deterministic labels within numeric
tolerance.

## 8. Add realistic GPS-denied localization

Add `src/lunabot_localization/config/ekf.yaml` for `robot_localization`.
Fuse wheel odometry and IMU into `/odometry/filtered`. Configure realistic
covariances and disable ground-truth inputs.

Update SLAM to consume filtered odometry and preserve the TF tree:

```text
map -> odom -> base_link/chassis -> sensors
```

The current `config/slam_toolbox_phase_c.yaml` may remain the 2D baseline.
Create a second configuration for RTAB-Map if RGB-D SLAM is required by the
final claim. Do not run two systems publishing `map -> odom` simultaneously.

Update the Gazebo model with configurable noise/bias for IMU, LiDAR, and
odometry. Create tests with wheel slip and drift.

Add `lunabot_localization/localization_evaluator.py` that compares estimated
pose with `/lunabot/ground_truth/pose` and reports:

- Absolute Trajectory Error;
- Relative Pose Error;
- drift per metre;
- final pose error;
- loop-closure corrections.

**Exit gate:** localization remains stable without GPS and meets documented ATE
and drift thresholds over multiple held-out missions.

## 9. Replace simplified semantic projection with calibrated fusion

Create a new mapping package rather than expanding the current script without
bounds. The new semantic mapper must use:

- synchronized class, confidence, and depth images;
- `CameraInfo` intrinsics;
- TF at the image timestamp;
- full pinhole projection;
- occupancy map geometry;
- temporal confidence fusion.

Recommended layers:

```text
occupancy, semantic_class, semantic_confidence, elevation,
slope, roughness, obstacle_distance, last_observed
```

Use `grid_map` or aligned typed grids. Unknown and stale observations must be
explicit. Obstacle evidence must dominate traversable evidence. Resolve class
conflicts through confidence-weighted Bayesian/log-odds fusion rather than
last-write-wins.

Handle loop closure: either retain observations with poses and rebuild affected
cells, or document a bounded local rolling semantic map that is not invalidated
by global corrections.

Keep `scripts/semantic_terrain_mapper.py` as the baseline and run both on the
same bag for comparison before replacing it.

**Exit gate:** projection tests with known camera geometry pass, semantic-map
IoU is measured against simulator ground truth, and repeated observations
improve rather than destabilize confidence.

## 10. Build the complete traversability map

Replace the current three-value conversion with a configurable layered cost:

```text
cost = semantic_cost
     + slope_weight * slope_cost
     + roughness_weight * roughness_cost
     + clearance_weight * clearance_cost
     + uncertainty_weight * (1 - confidence)
```

Clamp to `[0, 100]`; reserve `100` for lethal cells. Add:

- rover-footprint-aware inflation;
- maximum longitudinal and cross-slope limits;
- step-height limit;
- unknown-space policy;
- stale-observation policy;
- NaN/invalid-depth policy;
- explanation/debug layer showing why each cell is expensive.

Update `scripts/terrain_cost_mapper.py` or replace it with a packaged
`traversability_node.py`. The final planner must consume only the new map; do
not accidentally combine baseline and production publishers.

**Exit gate:** unit tests cover every layer and a scenario demonstrates a
longer safe route receiving lower total cost than a shorter crater route.

## 11. Implement true D*-Lite

Keep `scripts/terrain_aware_planner.py` as the weighted-A* baseline. Add:

```text
src/lunabot_planning/lunabot_planning/dstar_lite.py
src/lunabot_planning/lunabot_planning/planner_node.py
src/lunabot_planning/test/test_dstar_lite.py
```

The implementation must retain:

- `g` values;
- `rhs` values;
- `km`;
- priority queue keys;
- changed-cell state between map updates.

It must call `update_vertex` only for affected cells and repair the path with
`compute_shortest_path`, not run A* from scratch and rename it D*-Lite.

Planner inputs/outputs:

```text
IN:  traversability map, current pose, goal, changed cells
OUT: nav_msgs/Path, PlannerStatus, ReplanEvent
```

Record:

- planning/replanning trigger;
- changed-cell count;
- queue operations/expanded nodes;
- planning time;
- path length;
- accumulated semantic cost;
- failure reason.

Add footprint collision checks, diagonal corner-cut prevention, path smoothing,
and line-of-sight validation.

Tests must include standard published D*-Lite grid cases, unreachable goals,
start motion, cost increases, cost decreases, and random comparison with
fresh A* optimal costs.

**Exit gate:** D*-Lite returns correct paths and demonstrates lower repair time
than repeated A* for local map changes on representative maps.

## 12. Prove obstacle-triggered replanning

The current `scripts/dynamic_replan_monitor.py` only observes path changes.
Replace it for production with explicit event correlation while retaining the
old topic for compatibility.

Create a deterministic scenario:

1. Place the goal so the initial route crosses a known insertion zone.
2. Wait until the rover begins following the initial path.
3. Spawn or reveal a physical obstacle directly on that path.
4. Timestamp first sensor detection.
5. Timestamp cost-map lethal-cell update.
6. Timestamp replan request and new-path publication.
7. Verify the new path excludes the blocked footprint.
8. Verify the rover reaches the goal without contact.

Set obstacle evidence as mandatory. Merely changing the path because the start
position moved must not count as a replan.

**Exit gate:** at least 20 runs complete with recorded detection-to-replan
latency and no false `REPLAN_PASS` caused only by rover motion.

## 13. Upgrade control and safety

Rename topic ownership as follows:

```text
/cmd_vel_nav     navigation request
/cmd_vel_teleop  operator request
/cmd_vel_safe    safety-approved command (optional internal)
/cmd_vel         only safety supervisor publishes this
```

Implement or integrate Regulated Pure Pursuit. Add:

- cross-track and heading error;
- speed reduction for curvature, low clearance, rough terrain, and uncertainty;
- acceleration/deceleration limits;
- progress checker and stuck detection;
- localization-loss stop;
- sensor-timeout stop;
- cost-map/path timeout stop;
- rollover/slope stop;
- emergency stop service;
- no-path and recovery behavior.

Update `scripts/control_odometry.py` and `scripts/terrain_path_follower.py` only
as compatibility baselines; production behavior belongs in
`lunabot_control`.

**Exit gate:** command ownership test confirms one final publisher, injected
sensor/localization failures stop the rover, and path tracking error is
reported.

## 14. Add quantitative evaluation

Replace the pass-only role of `scripts/phase_k_evaluator.py` with a metrics
package while preserving its compatibility output.

Collect per trial:

- mission success and failure reason;
- planned and executed path length;
- mission duration;
- path efficiency;
- accumulated traversability cost;
- distance/time in every terrain class;
- hazardous/lethal cells entered;
- minimum obstacle clearance;
- replan count and latency distribution;
- collision count, duration, object, relative speed, and impulse if available;
- ATE/RPE localization error;
- inference FPS/latency and dropped frames;
- CPU/GPU/memory utilization;
- controller cross-track error.

Bridge Gazebo contact data and add collision sensors where required. A status
string is not collision evidence.

Store one machine-readable `trial.json` or CSV row per run plus artifacts. Add
`tools/summarize_experiments.py` to produce tables and plots.

Run a factorial experiment across held-out world seeds, goals, illumination,
sensor noise, and obstacle scenarios. Compare:

1. geometric A*;
2. semantic weighted A*;
3. semantic D*-Lite.

Use at least 30 trials per major comparison if time permits. Fix seeds and
publish the complete scenario manifest.

**Exit gate:** the final report contains confidence intervals or distribution
plots, not only one successful demonstration.

## 15. Final launch and demonstration

Add:

```text
src/lunabot_bringup/launch/lunabot_goal.launch.py
src/lunabot_bringup/config/goal_system.yaml
scripts/run_experiment.sh
scripts/run_final_demo.sh
```

The final GUI demonstration must show:

1. Gazebo world and physical rover;
2. RGB image and multiclass overlay;
3. SLAM occupancy map;
4. semantic class/confidence layers;
5. traversability map;
6. initial path;
7. autonomous motion;
8. newly introduced physical obstacle;
9. cost-map update and D*-Lite repair;
10. safe goal arrival;
11. final quantitative metrics.

Provide modes for `simulation`, `bag_replay`, and `evaluation`. Never use
simulator ground-truth labels or pose in the final navigation pipeline.

## 16. Files to retain, update, replace, and add

### Retain as baselines

- `scripts/astar_navigation.py`
- `scripts/terrain_segmentation.py`
- `scripts/semantic_terrain_mapper.py`
- `scripts/terrain_cost_mapper.py`
- `scripts/terrain_aware_planner.py`
- `scripts/terrain_path_follower.py`
- `scripts/dynamic_replan_monitor.py`
- A–L validators and evidence

### Update

- `src/lunabot_gazebo/models/lunabot_v4/model.sdf`: camera info, noise,
  contacts, ground-truth-only evaluation interfaces.
- `src/lunabot_gazebo/worlds/lunar_world.sdf`: semantic IDs, scenario objects,
  dynamic obstacle insertion zones, lighting variants.
- `tools/generate_lunar_terrain.py`: labels, metadata, controllable generation.
- `config/slam_toolbox_phase_c.yaml`: filtered odometry and tuned covariances
  after baseline measurements.
- `scripts/launch-l.sh`: eventually become a compatibility wrapper around the
  ROS final launch.
- `rviz/phase_l.rviz`: multiclass, confidence, cost, D*-Lite event, and metrics
  displays.
- `README.md`: only claim technologies backed by code and reports.

### Add

- ROS packages listed in Section 3.
- ML dataset/training/export workspace.
- Trained-model manifest and checksum.
- Terrain class configuration.
- EKF and optional RTAB-Map configuration.
- Calibrated semantic fusion.
- Layered traversability map.
- Tested D*-Lite implementation.
- Safety supervisor.
- Metrics recorder and experiment runner.
- Unit, integration, bag-replay, and scenario tests.
- CI workflow and reproducible environment/container.

## 17. Recommended execution order

Do work in this exact order:

1. Validate and record current Phase L baseline.
2. Package current nodes without behavior changes.
3. Define semantic classes and messages.
4. Add simulator ground truth and dataset capture.
5. Generate and validate the dataset.
6. Train/evaluate/export models.
7. Integrate ROS ML inference.
8. Add EKF and localization evaluation.
9. Build calibrated multiclass semantic mapping.
10. Build layered traversability mapping.
11. Implement/test D*-Lite.
12. Build deterministic obstacle-insertion replanning scenario.
13. Upgrade path following and safety supervision.
14. Implement quantitative metrics.
15. Run multi-seed comparative experiments.
16. Update final documentation and claims.
17. Record the final live demonstration.

Do not start D*-Lite before cost-map semantics are stable, and do not tune the
planner using ground-truth segmentation if the final system uses predicted
segmentation.

## 18. Definition of done

The project reaches goal level only when all items below are supported by saved
evidence:

- A trained multiclass model—not the heuristic—runs in ROS.
- Held-out test-world semantic metrics are reported.
- Localization operates without GPS/ground truth and its error is measured.
- Semantic predictions are calibrated and fused into a persistent map.
- Traversability combines semantic, geometry, slope/roughness, clearance, and
  uncertainty.
- A* supplies a baseline and genuine D*-Lite performs incremental repairs.
- A newly detected physical obstacle demonstrably triggers map update and path
  repair.
- The rover safely reaches goals across multiple unseen scenarios.
- Success, path length, safety, replanning time, collision, traversability,
  localization, and perception metrics are reported.
- The final pipeline is launched reproducibly from one command.
- Simulator ground truth is used only for labels and evaluation, never for
  navigation decisions.
