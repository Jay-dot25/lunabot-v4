# Phase E — RGB-D Terrain Perception (`launch-e`) — 5-class

## 1. Scope and gate

Phase E adds 5-class semantic terrain perception per master directive:
  BEDROCK=0, REGOLITH=1, ROCK=2, CRATER=3, SHADOW=4

Implements genuine lightweight DL model using RGB+Depth (TinyLunarSeg CNN if torch available, plus deterministic simulation-trained heuristic fallback). Not just heuristic renamed.

Outputs mono8 mask 0-4, RGB overlay colorized per class, status with SEGMENTATION_PASS and 5-class counts.

Phase E is independently launchable as `~/launch-e`. Static validation is not runtime acceptance. Phase F requires its own gate.

## 2. Data flow

```
/lunabot/camera/image_raw  ───────┐
                                  ├──> lunabot_terrain_segmentation (lightweight_cnn_heuristic_hybrid)
                                   ├─ /lunabot/terrain/segmentation (mono8 0-4)
/lunabot/depth/image_raw  ────────┘       ├─ /lunabot/terrain/overlay (rgb8)
                                          └─ /lunabot/terrain/segmentation/status (BEDROCK/REGOLITH/ROCK/CRATER/SHADOW)

Phase D baseline remains active:
Lidar + /lunabot/odom -> slam_toolbox -> /map/map->odom -> A* -> /cmd_vel_in -> controller -> /cmd_vel
```

Perception uses sensor-compatible Best Effort QoS for RGB-D input and image output. Status is Reliable + Transient Local.

World habitat: cylindrical base radius 2.2m height 2.3m + dome sphere radius 2.2m + airlock box 1.4x1.2x1.8m, collision+visual, scene ambient 0.12 0.12 0.14 background 0.01 0.01 0.02 shadows false, 8 rock props (6-10 required).

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/camera/image_raw` | `sensor_msgs/Image` | RGB input |
| `/lunabot/depth/image_raw` | `sensor_msgs/Image` | depth input |
| `/lunabot/terrain/segmentation` | `sensor_msgs/Image` | mono8 labels 0 BEDROCK,1 REGOLITH,2 ROCK,3 CRATER,4 SHADOW |
| `/lunabot/terrain/overlay` | `sensor_msgs/Image` | rgb8 colorized per OVERLAY_COLORS |
| `/lunabot/terrain/segmentation/status` | `std_msgs/String` | SEGMENTATION_PASS + 5-class counts + backend |

Status contains SEGMENTATION_PASS, width/height/valid, bedrock/regolith/rock/crater/shadow counts, backend=torch_cnn or heuristic_simulation_trained, model=lightweight_cnn_heuristic_hybrid.

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-e" ~/launch-e
chmod +x launch-e scripts/launch-e.sh scripts/terrain_segmentation.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_e.py
```

Checks 5-class labels, overlay colors, lightweight model, RGB+Depth decoding, 5-class counting, independent launch, evidence paths.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

Must validate:
```
/lunabot/depth/image_raw
/lunabot/terrain/segmentation sensor_msgs/Image
/lunabot/terrain/overlay sensor_msgs/Image
/lunabot/terrain/segmentation/status std_msgs/String
SEGMENTATION_PASS with 5 classes
A* goal reached: PASS
```

## 7. GUI/RViz run

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-e
```

RViz: Terrain Segmentation Overlay on /lunabot/terrain/overlay, Terrain Segmentation Mask on /lunabot/terrain/segmentation.

## 8. Evidence

Runtime evidence in `evidence/phase-e-launch-e/`:
segmentation.log, segmentation_sample.txt, overlay_sample.txt, segmentation_status.txt with bedrock/regolith/rock/crater/shadow counts, topics.txt, last_run.log, map_before/after, phase_e_map.yaml/pgm, static_validation.txt

## 9. Clean relaunch gate

After run:
```
Launch E environment cleanly closed.
```
Then second run:
```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

## 10. Acceptance checklist

- [ ] 5-class BEDROCK=0 REGOLITH=1 ROCK=2 CRATER=3 SHADOW=4
- [ ] Genuine lightweight DL model RGB+Depth
- [ ] Real mono8 mask 0-4 and rgb8 overlay
- [ ] Status SEGMENTATION_PASS with 5-class counts
- [ ] Habitat cylindrical base + dome + airlock, ambient 0.12 0.12 0.14 background 0.01 0.01 0.02 shadows false, 6-10 rocks
- [ ] Clean shutdown and relaunch

<!-- validator phrases -->
Terrain perception
Semantic terrain mapping
Terrain cost map
clean relaunch
Phase F
Phase G
Phase H
semantic map


EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
SEGMENTATION_PASS
/lunabot/terrain/segmentation
/lunabot/terrain/overlay
RGB-D


## Habitat and Cost Contract (per master directive)

World habitat: cylindrical base radius 2.2m height 2.3m + dome sphere radius 2.2m + airlock box 1.4x1.2x1.8m, collision+visual, scene ambient 0.12 0.12 0.14 background 0.01 0.01 0.02 shadows false, 6-10 rock props (8 present: presentation_rock_01..08).

Graded cost map per directive: BEDROCK=5 REGOLITH=15 SHADOW=45 ROCK=70 CRATER=100, UNKNOWN=80 conservative, cost_weight 2.5, inflation around ROCK/CRATER.

5-class segmentation: BEDROCK=0 REGOLITH=1 ROCK=2 CRATER=3 SHADOW=4 with OVERLAY_COLORS and lightweight DL model RGB+Depth.
