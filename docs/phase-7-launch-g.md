# Phase G — Terrain Cost Map (`launch-g`)

## 1. Scope and gate

Phase G adds a numeric terrain-cost layer to the approved Phase F stack. It
converts the real semantic terrain grid into a traversability cost grid with a
conservative obstacle inflation halo. It does not change A*, add a second
planner, alter localization, or bypass the Phase B controller.

The cost grid uses values from 0 to 100:

```text
20   observed terrain candidate
80   unknown / caution
100  obstacle or inflated obstacle
```

Obstacle evidence dominates terrain evidence. The inflation halo decreases
with distance from obstacle cells but remains above the nominal terrain cost.
The output is intentionally not consumed by A* in Phase G; Phase H will add
terrain-aware planning after this cost-map contract is independently validated.

Terrain cost mapping is the Phase G addition to the approved Phase F stack. It
consumes `/lunabot/terrain/semantic_map` and publishes
`/lunabot/terrain/cost_map` in the same map frame.

Phase G is independently launchable as `~/launch-g`. Static validation is not
runtime acceptance. The workstation runtime must produce real cost-map and
status messages, preserve the approved Phase A-F gates, and cleanly relaunch
before Phase G can be approved.

## 2. Data flow

```text
Phase F /lunabot/terrain/semantic_map ──> terrain_cost_mapper
                                             ├─ /lunabot/terrain/cost_map
                                             └─ /lunabot/terrain/cost_map/status

Approved navigation path remains unchanged:
slam_toolbox -> /map -> A* -> /cmd_vel_in -> Phase B controller -> /cmd_vel
```

The cost mapper is a pure ROS node. It republishes the semantic map geometry,
converts labels to costs, and inflates obstacle cells using the input map
resolution. It publishes no velocity, TF, or localization data.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/semantic_map` | `nav_msgs/OccupancyGrid` | Phase F semantic input |
| `/lunabot/terrain/cost_map` | `nav_msgs/OccupancyGrid` | inflated traversability costs |
| `/lunabot/terrain/cost_map/status` | `std_msgs/String` | frame and cost-cell statistics |

The cost map and status are Reliable + Transient Local for late-joining RViz
and runtime diagnostics. A successful status contains `COST_MAP_PASS` and
reports terrain, unknown, obstacle, and inflated-cell counts.

## 4. Installation

Phase G uses the approved Phase F workstation dependencies. No additional
mapping or machine-learning package is required.

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-g" ~/launch-g
chmod +x launch-g scripts/launch-g.sh scripts/terrain_cost_mapper.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_g.py
```

The validator checks the approved Phase A-F baseline, independent launch
behavior, semantic-to-cost conversion, obstacle inflation, output/status QoS,
real runtime checks, RViz output, evidence paths, and honest runtime gating.
It never claims that a real cost map was generated.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, Phase E segmentation, Phase F semantic mapper, cost mapper,
`slam_toolbox`, and A* directly. It must validate the inherited Phase A-F
interfaces and these Phase G interfaces:

```text
/lunabot/terrain/cost_map        nav_msgs/OccupancyGrid
/lunabot/terrain/cost_map/status std_msgs/String
COST_MAP_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE G RUN COMPLETE - overall result: PASS
Launch G environment cleanly closed.
```

The cost-map check requires real `ros2 topic type` and `ros2 topic echo --once`
messages. A cost-mapper process being alive is not a pass.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-g
```

RViz retains the Phase F semantic map, Phase E segmentation overlay, camera,
LiDAR, A* path, odometry, map, and TF displays. It adds `Terrain Cost Map` on
`/lunabot/terrain/cost_map` with a costmap color scheme. The cost map is
visualization/evidence output only; it does not change A* behavior in Phase G.

Confirm that the cost grid updates after semantic-map frames arrive, the status
reports cost-cell counts, and inherited navigation outputs remain error-free.
Press Ctrl+C for clean shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-g-launch-g/`:

| File | Meaning |
|---|---|
| `cost_mapping.log` | cost-map node diagnostics |
| `cost_map_sample.txt` | real cost `OccupancyGrid` sample |
| `cost_map_status.txt` | real `COST_MAP_PASS` status sample |
| `semantic_map_sample.txt` | inherited Phase F input sample |
| `semantic_map_status.txt` | inherited Phase F status sample |
| `segmentation_sample.txt` | inherited Phase E mask sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launcher and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited A* map-motion gate |
| `phase_g_map.yaml` / `phase_g_map.pgm` | final SLAM map evidence, when available |
| `static_validation.txt` | generated static Phase G report |

Do not hand-edit runtime evidence. Inspect `cost_mapping.log`, semantic and
segmentation logs, bridge/Gazebo logs, and `last_run.log` when a check fails.

## 9. Clean relaunch gate

After the autonomous or GUI run:

```text
Launch G environment cleanly closed.
```

Then repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g
```

Both runs must start one cost mapper, publish real cost-map/status messages,
preserve the A* and Phase F gates, and stop all process groups. This clean relaunch is part of the Phase G runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-F static and runtime contracts remain passing.
- [ ] Semantic map is a real input message.
- [ ] Cost map is a real non-empty `nav_msgs/OccupancyGrid`.
- [ ] Cost-map status contains `COST_MAP_PASS`.
- [ ] Terrain, unknown, obstacle, and inflated-cell counts are reported.
- [ ] Obstacle inflation is visible in the cost grid.
- [ ] RViz displays the cost map without changing navigation.
- [ ] A* still reaches its goal through the Phase B controller boundary.
- [ ] Final map evidence is saved when the map-saver package is available.
- [ ] Clean shutdown and a second independent relaunch pass.

Phase G's runtime gate is explicitly approved. A separate Phase H request has
started terrain-aware path-planning implementation; Phase H must complete its
own runtime gate before Phase I.
