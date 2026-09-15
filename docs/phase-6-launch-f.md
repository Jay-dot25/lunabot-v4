# Phase F — Semantic Terrain Mapping (`launch-f`)

## 1. Scope and gate

Semantic terrain mapping is the Phase F addition to the approved Phase E
stack. It fuses the real Phase E segmentation mask with the existing depth
stream and projects observations into a fixed grid in the existing `map` frame. It does
not add SLAM, AMCL, Cartographer, a second localization source, a cost-map
planner, or a velocity controller.

The semantic map is an auditable `nav_msgs/OccupancyGrid` with these semantic values:

```text
-1   unknown / not observed
25   terrain candidate
100  obstacle candidate
```

The mapper uses the existing `slam_toolbox` `map -> odom` and DiffDrive
`odom -> chassis` TF chain. It publishes no `/cmd_vel` or `/cmd_vel_in`
messages. Terrain-aware cost fusion remains Phase G; semantic map output is
not silently connected to the approved A* planner.

Phase F is independently launchable as `~/launch-f`. Static validation is not
runtime acceptance. The workstation runtime must produce real semantic-map
and status messages, preserve the approved Phase A-E gates, and cleanly
relaunch before Phase F can be approved.

## 2. Data flow

```text
Phase E /lunabot/terrain/segmentation ──┐
                                        ├──> lunabot_semantic_terrain_mapper
/lunabot/depth/image_raw ──────────────┘        │
                                                ├─ /lunabot/terrain/semantic_map
existing map -> chassis TF ─────────────────────└─ /lunabot/terrain/semantic_map/status

Approved navigation path remains unchanged:
slam_toolbox -> /map -> A* -> /cmd_vel_in -> Phase B controller -> /cmd_vel
```

The mapper samples valid depth pixels at a configurable stride, derives a
horizontal bearing from the existing camera horizontal field of view, and
transforms the observation through the current map-frame robot pose. Obstacle
evidence dominates terrain evidence when both reach the same semantic cell.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/segmentation` | `sensor_msgs/Image` | Phase E labels: 1 terrain, 2 obstacle |
| `/lunabot/depth/image_raw` | `sensor_msgs/Image` | real depth range for projection |
| `map -> chassis` TF | `tf2` | existing SLAM + odometry pose |
| `/lunabot/terrain/semantic_map` | `nav_msgs/OccupancyGrid` | accumulated semantic grid |
| `/lunabot/terrain/semantic_map/status` | `std_msgs/String` | frame, sample, and class-cell statistics |

The semantic map and status are Reliable + Transient Local for late-joining
RViz/runtime diagnostics. A successful status contains `SEMANTIC_MAP_PASS` and
reports accumulated observations, terrain cells, obstacle cells, resolution,
and the map frame.

## 4. Installation

Phase F uses the approved Phase E workstation dependencies. No additional ML
or mapping package is required.

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-f" ~/launch-f
chmod +x launch-f scripts/launch-f.sh scripts/semantic_terrain_mapper.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_f.py
```

The validator checks the approved Phase A-E baseline, independent launch
behavior, semantic projection implementation, map/status QoS, real runtime
checks, RViz output, evidence paths, and honest runtime gating. It never claims
that a real semantic observation was mapped.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, Phase E segmentation, `slam_toolbox`, semantic mapper, and A*
directly. It must validate the inherited Phase A-E interfaces and these Phase F
interfaces:

```text
/lunabot/terrain/semantic_map        nav_msgs/OccupancyGrid
/lunabot/terrain/semantic_map/status std_msgs/String
SEMANTIC_MAP_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE F RUN COMPLETE - overall result: PASS
Launch F environment cleanly closed.
```

The semantic-map check requires real `ros2 topic type` and
`ros2 topic echo --once` messages. A mapper process being alive is not a pass.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-f
```

RViz retains the Phase E map, A* path, camera, LiDAR, odometry, segmentation
overlay, and TF displays. It adds `Semantic Terrain Map` on
`/lunabot/terrain/semantic_map` with a costmap color scheme. The semantic map
is visualization/evidence output only; it does not change A* behavior in Phase
F.

Confirm that the semantic grid updates as segmentation frames arrive, the
status reports increasing frame and observation counts, and the inherited
navigation outputs remain error-free. Press Ctrl+C for clean shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-f-launch-f/`:

| File | Meaning |
|---|---|
| `semantic_mapping.log` | semantic mapper diagnostics |
| `semantic_map_sample.txt` | real semantic `OccupancyGrid` sample |
| `semantic_map_status.txt` | real `SEMANTIC_MAP_PASS` status sample |
| `segmentation_sample.txt` | inherited Phase E mask sample |
| `segmentation_status.txt` | inherited Phase E status sample |
| `depth_sample.txt` | real depth image sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launcher and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited A* map-motion gate |
| `phase_f_map.yaml` / `phase_f_map.pgm` | final SLAM map evidence, when available |
| `static_validation.txt` | generated static Phase F report |

Do not hand-edit runtime evidence. Inspect `semantic_mapping.log`,
`segmentation.log`, bridge/Gazebo logs, and `last_run.log` when a check fails.

## 9. Clean relaunch gate

After the autonomous or GUI run:

```text
Launch F environment cleanly closed.
```

Then repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

Both runs must start one semantic mapper, publish real semantic-map/status
messages, preserve the A* and Phase E gates, and stop all process groups. This
clean relaunch is part of the Phase F runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-E static and runtime contracts remain passing.
- [ ] Segmentation and depth are real input messages.
- [ ] `map -> chassis` TF is used for semantic projection.
- [ ] Semantic map is a real non-empty `nav_msgs/OccupancyGrid`.
- [ ] Semantic map status contains `SEMANTIC_MAP_PASS`.
- [ ] Terrain and obstacle cell counts are reported.
- [ ] RViz displays the semantic map without changing navigation.
- [ ] A* still reaches its goal through the Phase B controller boundary.
- [ ] Final map evidence is saved when the map-saver package is available.
- [ ] Clean shutdown and a second independent relaunch pass.

Do not begin Phase G (terrain cost map) until Phase F has its own explicit
runtime approval.
