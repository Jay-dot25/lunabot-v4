# Phase H — Terrain-Aware Path Planning (`launch-h`)

## 1. Scope and gate

Phase H adds a weighted A* path planner that consumes the approved Phase G
terrain cost map. It publishes a terrain-aware path for inspection and future
integration, but it does not publish velocity. The approved Phase D A* planner
continues to provide the active `/cmd_vel_in` navigation path; Phase I will
integrate terrain-aware planning into autonomous motion.

The weighted planner uses 8-connected grid search. Cells at or above cost 100
are blocked; traversable cells are weighted by their cost so lower-cost
terrain is preferred. Unknown/caution cells remain traversable at a configured
cost of 80, allowing the runtime to produce an auditable path before the map
covers the entire grid.

Terrain-aware path planning is the Phase H addition to the approved Phase G
stack. It consumes `/lunabot/terrain/cost_map`, `/goal_pose`, and the existing
`map -> chassis` localization chain, then publishes `/lunabot/terrain/plan`.

Phase H is independently launchable as `~/launch-h`. Static validation is not
runtime acceptance. The workstation runtime must produce a real terrain-aware
path and `TERRAIN_PLAN_PASS` status, preserve the approved Phase A-G gates, and
cleanly relaunch before Phase H can be approved.

## 2. Data flow

```text
Phase G /lunabot/terrain/cost_map ──┐
                                    ├──> terrain_aware_planner
/goal_pose + map -> chassis TF ─────┘       ├─ /lunabot/terrain/plan
                                             └─ /lunabot/terrain/planner/status

Approved motion path remains unchanged in Phase H:
slam_toolbox -> /map -> Phase D A* -> /cmd_vel_in -> Phase B controller -> /cmd_vel
```

The terrain-aware planner is deliberately an observation/planning output only.
It publishes a path without motion output: it does not connect to `/cmd_vel_in`,
`/cmd_vel`, Gazebo, or a second localization system.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/cost_map` | `nav_msgs/OccupancyGrid` | Phase G traversability input |
| `/goal_pose` | `geometry_msgs/PoseStamped` | existing active navigation goal |
| `/lunabot/odom` | `nav_msgs/Odometry` | current pose source |
| `map -> chassis` TF | `tf2` | existing SLAM + odometry localization |
| `/lunabot/terrain/plan` | `nav_msgs/Path` | weighted terrain-aware path |
| `/lunabot/terrain/planner/status` | `std_msgs/String` | planning result and weighted cost |

The path and status are Reliable + Transient Local for late-joining RViz and
runtime diagnostics. A successful status contains `TERRAIN_PLAN_PASS`, path
cell count, weighted cost, map frame, and cost weight.

## 4. Installation

Phase H uses the approved Phase G workstation dependencies. No additional
planner or navigation stack is required.

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-h" ~/launch-h
chmod +x launch-h scripts/launch-h.sh scripts/terrain_aware_planner.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_h.py
```

The validator checks the approved Phase A-G baseline, independent launch
behavior, weighted cost search, blocked-cell handling, localization inputs,
output/status QoS, real runtime checks, RViz output, evidence paths, and honest
runtime gating. It never claims that a real terrain-aware path was generated.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, Phase E segmentation, Phase F semantic mapper, Phase G cost mapper,
`slam_toolbox`, the active Phase D A*, and the terrain-aware planner directly.
It must validate the inherited Phase A-G interfaces and these Phase H
interfaces:

```text
/lunabot/terrain/plan          nav_msgs/Path
/lunabot/terrain/planner/status std_msgs/String
TERRAIN_PLAN_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE H RUN COMPLETE - overall result: PASS
Launch H environment cleanly closed.
```

The terrain-plan check requires real `ros2 topic type` and
`ros2 topic echo --once` messages. A planner process being alive is not a pass.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-h
```

RViz retains the Phase G cost map, semantic map, Phase E segmentation overlay,
camera, LiDAR, A* path, odometry, map, and TF displays. It adds
`Terrain-aware Plan` on `/lunabot/terrain/plan`.

Confirm that the orange terrain-aware path is visible and updates when the
cost map or goal changes. Confirm that the approved A* path and actual rover
motion remain unchanged in Phase H. Press Ctrl+C for clean shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-h-launch-h/`:

| File | Meaning |
|---|---|
| `terrain_planner.log` | weighted planner diagnostics |
| `terrain_plan.txt` | real terrain-aware `nav_msgs/Path` sample |
| `terrain_planner_status.txt` | real `TERRAIN_PLAN_PASS` status sample |
| `cost_map_sample.txt` | inherited Phase G input sample |
| `cost_map_status.txt` | inherited Phase G status sample |
| `semantic_map_sample.txt` | inherited Phase F input sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launcher and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited A* map-motion gate |
| `phase_h_map.yaml` / `phase_h_map.pgm` | final SLAM map evidence, when available |
| `static_validation.txt` | generated static Phase H report |

Do not hand-edit runtime evidence. Inspect `terrain_planner.log`, cost/semantic
logs, bridge/Gazebo logs, and `last_run.log` when a check fails.

## 9. Clean relaunch gate

After the autonomous or GUI run:

```text
Launch H environment cleanly closed.
```

Then repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h
```

Both runs must start one terrain-aware planner, publish real plan/status
messages, preserve the A* and Phase G gates, and stop all process groups. This
clean relaunch is part of the Phase H runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-G static and runtime contracts remain passing.
- [ ] Cost map is a real input message.
- [ ] Terrain-aware plan is a real non-empty `nav_msgs/Path`.
- [ ] Planner status contains `TERRAIN_PLAN_PASS`.
- [ ] Weighted cost and path-cell statistics are reported.
- [ ] Blocked obstacle cells are not selected by the planner.
- [ ] RViz displays the terrain-aware path without changing active motion.
- [ ] A* still reaches its goal through the Phase B controller boundary.
- [ ] Final map evidence is saved when the map-saver package is available.
- [ ] Clean shutdown and a second independent relaunch pass.

Do not begin Phase I (full autonomous integration) until Phase H has its own
explicit runtime approval.
