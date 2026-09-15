# Phase E — RGB-D Terrain Perception (`launch-e`)

## 1. Scope and gate

Phase E adds the first terrain-perception layer to the runtime-approved Phase
D stack. It consumes the existing RGB and depth camera streams and publishes a
pixel-wise terrain/obstacle/unknown segmentation mask plus a visualization
overlay. It does not modify the lunar terrain, rover model, SLAM system, A*
planner, controller, odometry source, or Phase D topics.

Terrain perception is implemented as an auditable, deterministic RGB-D baseline
rather than a trained semantic model. A valid depth return in the lower camera field is a
terrain candidate when its RGB return is low-saturation; valid returns outside
the ground region or with saturated RGB are conservative obstacle candidates.
Invalid depth is unknown. Semantic terrain mapping and navigation cost fusion
remain future phases.

Phase E is independently launchable as `~/launch-e`. Static validation is not
runtime acceptance. The workstation runtime must produce real mask, overlay,
and status messages, then cleanly relaunch before Phase E can be approved.

## 2. Data flow

```text
/lunabot/camera/image_raw  ───────┐
                                  ├──> lunabot_terrain_segmentation
/lunabot/depth/image_raw  ────────┘       ├─ /lunabot/terrain/segmentation
                                          ├─ /lunabot/terrain/overlay
                                          └─ /lunabot/terrain/segmentation/status

Phase D baseline remains active:
Lidar + /lunabot/odom -> slam_toolbox -> /map/map->odom -> A* -> /cmd_vel_in
                                                      -> Phase B controller -> /cmd_vel
```

The perception node uses sensor-compatible Best Effort QoS for RGB-D input and
image output. The status topic is Reliable + Transient Local so late-joining
runtime checks can observe the latest segmentation result.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/camera/image_raw` | `sensor_msgs/Image` | existing RGB input |
| `/lunabot/depth/image_raw` | `sensor_msgs/Image` | existing depth input |
| `/lunabot/terrain/segmentation` | `sensor_msgs/Image` | `mono8` labels: 0 unknown, 1 terrain, 2 obstacle |
| `/lunabot/terrain/overlay` | `sensor_msgs/Image` | `rgb8` green terrain/red obstacle/black unknown visualization |
| `/lunabot/terrain/segmentation/status` | `std_msgs/String` | frame counts and class statistics |

A successful status contains `SEGMENTATION_PASS` and reports the image size,
valid depth count, terrain count, obstacle count, and unknown count.

The perception node intentionally publishes no velocity, map, or cost-map
messages. Phase D remains the only navigation and motion layer.

## 4. Installation

Phase E uses the approved Phase D workstation dependencies. No ML runtime or
`cv_bridge` package is required; image and depth bytes are decoded directly
from `sensor_msgs/Image`.

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

The validator checks the Phase A-D baseline, independent launch behavior,
RGB-D decoding, output labels, image/status interfaces, RViz displays, real
runtime checks in the launcher, evidence paths, and honest runtime gating.
The validator never claims a camera frame was actually segmented.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, `slam_toolbox`, A*, and terrain perception directly. It must validate
the inherited Phase D interfaces and these Phase E interfaces:

```text
/lunabot/depth/image_raw
/lunabot/terrain/segmentation       sensor_msgs/Image
/lunabot/terrain/overlay            sensor_msgs/Image
/lunabot/terrain/segmentation/status std_msgs/String
SEGMENTATION_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE E RUN COMPLETE - overall result: PASS
```

Do not treat the segmentation process being alive as a pass. The launcher uses
real `ros2 topic type` and `ros2 topic echo --once` checks and requires the
status content to contain `SEGMENTATION_PASS`.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-e
```

RViz retains the Phase D map, A* path, camera, LiDAR, odometry, and TF displays.
It adds:

- `Terrain Segmentation Overlay` on `/lunabot/terrain/overlay`, enabled by default;
- `Terrain Segmentation Mask` on `/lunabot/terrain/segmentation`, available as an optional display.

Confirm that the overlay publishes continuously while the rover and camera are
running, and that the status reports non-zero frame counts. Press Ctrl+C in the
launcher terminal for clean shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-e-launch-e/`:

| File | Meaning |
|---|---|
| `segmentation.log` | terrain segmentation node diagnostics |
| `depth_sample.txt` | real depth `Image` sample |
| `segmentation_sample.txt` | real `mono8` mask sample |
| `overlay_sample.txt` | real `rgb8` overlay sample |
| `segmentation_status.txt` | real `SEGMENTATION_PASS` status sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launch and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited A* map-motion gate |
| `phase_e_map.yaml` / `phase_e_map.pgm` | final map evidence, when map saver is installed |
| `static_validation.txt` | generated static Phase E report |

Do not hand-edit runtime evidence. If a check fails, inspect the segmentation,
bridge, and Gazebo logs, fix the runtime issue, and repeat the run.

## 9. Clean relaunch gate

After the autonomous run or GUI run:

```text
Launch E environment cleanly closed.
```

Then run the automated command a second time:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

Both runs must start one perception node, produce real segmentation messages,
retain the Phase D A* goal and map gates, and cleanly stop all process groups.
This clean relaunch is part of the Phase E runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-D static and runtime contracts remain passing.
- [ ] Existing RGB and depth topics are real `sensor_msgs/Image` messages.
- [ ] The segmentation node decodes the workstation's depth encoding.
- [ ] A real non-empty `mono8` mask is published.
- [ ] A real `rgb8` overlay is published.
- [ ] Status contains `SEGMENTATION_PASS` with frame/class counts.
- [ ] RViz displays the overlay without changing Phase D displays.
- [ ] A* still reaches its goal through the Phase B controller boundary.
- [ ] Final map evidence is saved when the map-saver package is available.
- [ ] Clean shutdown and a second independent relaunch pass.

Do not begin Phase F (semantic terrain mapping) until these runtime gates are
explicitly approved.
