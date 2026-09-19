# Phase E — Trained Five-Class RGB-D Terrain Perception (`launch-e`)

## Scope and current gate

Phase E adds sensor-derived semantic perception while preserving the independently runnable Phase D geometric A* baseline. It does **not** feed semantic costs into A*, publish velocity commands, or modify SLAM. Phase B remains the sole `/cmd_vel_in` to `/cmd_vel` command boundary.

Phase E is not approved by this document or by static validation. Approval requires the headless, GUI/evidence, relaunch, regression, and stale-process gates below on ROS 2 Humble/Gazebo. No later phase may begin before those gates pass.

## Model and labels

`tools/train_terrain_mlp.py` deterministically generates a balanced simulation-domain RGB-D feature dataset and performs supervised SGD. It writes `models/terrain_mlp_v1.json`, a compact CPU-oriented MLP with architecture `7-12-8-5` and two ReLU hidden layers. This is a genuinely trained model; it is not threshold logic and is not SegFormer, DeepLab, or U-Net.

Features are red, green, blue, normalized depth, normalized image row, local depth gradient, and local RGB texture. The exact `mono8` contract is:

| Value | Class | Overlay color |
|---:|---|---|
| 0 | `BEDROCK` | blue-grey |
| 1 | `REGOLITH` | ochre |
| 2 | `ROCK` | red |
| 3 | `CRATER` | purple |
| 4 | `SHADOW` | near-black |

The artifact reports `92.800%` accuracy on its generated held-out synthetic split. That number is reproducibility metadata—not real-world accuracy, Gazebo runtime acceptance, or proof that every class occurs in a particular camera frame.

## Data flow and ROS interfaces

```text
/lunabot/camera/image_raw + /lunabot/depth/image_raw
  -> lunabot_terrain_segmentation
     -> /lunabot/terrain/segmentation        sensor_msgs/Image (mono8, labels 0..4)
     -> /lunabot/terrain/overlay             sensor_msgs/Image (rgb8)
     -> /lunabot/terrain/segmentation/status std_msgs/String
```

Image input/output uses sensor-compatible Best Effort QoS. Status is Reliable and Transient Local. A processed frame reports `SEGMENTATION_PASS`, `model=lunabot_mlp_v1`, `trained=true`, dimensions, invalid-depth count, and separate counts for all five classes. The status proves inference ran, but visual and behavioral inspection remains mandatory.

## Installation and static checks

```bash
sudo apt install python3-numpy
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
chmod +x launch-e scripts/launch-e.sh scripts/terrain_segmentation.py tools/train_terrain_mlp.py
ln -sfn "$PWD/launch-e" ~/launch-e
python3 tools/train_terrain_mlp.py --output /tmp/terrain_mlp_v1.json
cmp /tmp/terrain_mlp_v1.json models/terrain_mlp_v1.json
python3 tools/validate_phase_e.py
```

The launcher fails early when NumPy or the model artifact is absent. The standard-library trainer can regenerate the artifact reproducibly; inference itself uses NumPy for vectorized pixel processing.

## Normal GUI run (manual goal default)

```bash
EVIDENCE=1 ~/launch-e
```

Without `DEMO=1`, A* does not inject an automatic goal. Use RViz's goal tool or the documented Phase D goal interface. RViz retains map, path, camera, LiDAR, odometry, and TF and adds the semantic overlay and optional mask. Inspect the overlay against camera/depth imagery: classes must be spatially meaningful, stable enough to audit, and not merely a single renamed class. Record screenshots externally if required by the workstation evidence procedure.

## Automated headless regression gate

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

`DEMO=1` enables the inherited automatic geometric A* goal. Acceptance requires real messages and all launcher checks, including:

- exact trained-model status and exact class order;
- non-empty mask and overlay messages derived from live RGB-D;
- Phase D A* goal reached through `/cmd_vel_in`;
- map change during navigation and fresh map save when map saver is installed;
- `PHASE E RUN COMPLETE - overall result: PASS`.

Evidence is written under `evidence/phase-e-launch-e/`, including logs, topic/type samples, semantic status/mask/overlay samples, TF, odometry, navigation, and map artifacts. Do not hand-edit generated evidence or treat old evidence as proof for this replacement model.

## Runtime semantic inspection

Confirm the status names all five classes exactly:

```bash
ros2 topic echo /lunabot/terrain/segmentation/status \
  --qos-reliability reliable --qos-durability transient_local --once
```

Inspect `/lunabot/terrain/overlay` in RViz and compare it with `/lunabot/camera/image_raw`. Class counts may legitimately be zero in one frame, but across suitable viewpoints evidence must demonstrate meaningful differentiation of `BEDROCK`, `REGOLITH`, `ROCK`, `CRATER`, and `SHADOW`. Synthetic held-out accuracy alone is insufficient.

## Clean relaunch and regression

After Ctrl+C, require `Launch E environment cleanly closed.` and verify no stale Gazebo, bridge, perception, SLAM, planner, controller, or monitor process remains. Repeat the headless command, then independently rerun the closed Phase D launcher to ensure its semantic-cost-free behavior is unchanged.

## Acceptance checklist

- [ ] Static validator passes and model regeneration check is deterministic.
- [ ] Real RGB and depth topics feed inference; mask values remain in 0..4.
- [ ] Status reports the exact five labels, trained model identity, and per-class counts.
- [ ] GUI inspection shows meaningful five-class behavior with captured evidence.
- [ ] Existing map, sensor, SLAM, TF, planner, controller, and odometry behavior remains healthy.
- [ ] Automated A* reaches its goal through the Phase B boundary.
- [ ] Fresh map evidence is saved and generated evidence is non-empty.
- [ ] Two independent Phase E runs shut down cleanly with no stale processes.
- [ ] Independent Phase D regression remains passing and semantic-cost-free.

Until every item is verified on the workstation, Phase E remains incomplete and the next phase must not start.
