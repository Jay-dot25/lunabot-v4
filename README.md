# LunaBot V4 — Autonomous Navigation of a Robot for Lunar Habitats

12-phase development of an autonomous lunar rover, phase-gated: each phase
has its own **independent** launch command, is validated at runtime before
it is approved, and every later phase reuses the validated baseline of the
earlier ones.

## Phases

| Phase | Launch | Scope | Status |
|---|---|---|---|
| A | `~/launch-a` | Simulation & rover foundation (lunar world, rover, sensors, ROS bridge, TF, RViz, teleop) | approved by user |
| B | `~/launch-b` | Control & odometry (safe command boundary, watchdog, odometry monitor) | implemented, pending runtime validation |
| C | `~/launch-c` | SLAM & localization (`slam_toolbox`, map/TF, evidence) | approved by user |
| D | `~/launch-d` | Basic autonomous navigation (A* planner, path follower) | approved by user |
| E | `~/launch-e` | RGB-D terrain perception (segmentation mask and overlay) | approved by user |
| F | `~/launch-f` | Semantic terrain mapping | approved by user |
| G | `~/launch-g` | Terrain cost map | approved by user |
| H | `~/launch-h` | Terrain-aware path planning | approved by user |
| I | `~/launch-i` | Full autonomous integration | approved by user |
| J | `~/launch-j` | Dynamic replanning | approved by user |
| K | `~/launch-k` | Testing & evaluation | implemented, pending runtime validation |
| L | `~/launch-l` | Final mission demonstration | not started |

Each implemented phase has a document under `docs/` covering objective,
inputs, data flow, ROS nodes/topics/services, TF frames, launch command,
validation procedure, success criteria, evidence, and limitations.

## Quickstart (Phase A)

Requirements (workstation):
- Ubuntu 22.04 (or compatible), **ROS 2 Humble**
- **Gazebo Sim** (Fortress — `ros-humble-gazebo-ros-pkgs` — or Garden+)
- **ros_ign bridge** (`ros-humble-ros-ign`)
- `rviz2`

```bash
cd ~/lunabot-v4
ln -s ~/lunabot-v4/launch-a ~/launch-a   # once

~/launch-a                    # interactive: Gazebo GUI + RViz2 + WASD
HEADLESS=1 DEMO=1 ~/launch-a  # automated runtime test (no GUI needed)
EVIDENCE=1 DEMO=1 ~/launch-a  # automated test + record runtime evidence
```

WASD drive: `W` forward, `S` reverse, `A` left, `D` right, `Space` stop,
`Q` quit. `Ctrl+C` stops everything cleanly (Gazebo, bridge, TF, RViz).

## Quickstart (Phase B)

Phase B is independent and keeps the Phase A world/rover baseline. It inserts
an explicit control boundary and odometry monitor:

```bash
cd ~/lunabot-v4
ln -s ~/lunabot-v4/launch-b ~/launch-b   # once; use ln -sf if it exists

~/launch-b                              # Gazebo + RViz + interactive WASD
HEADLESS=1 DEMO=1 ~/launch-b             # automated control-chain test
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-b  # test + odometry evidence
```

The control path is `/cmd_vel_in` → Phase B controller → `/cmd_vel` →
Gazebo. The controller clamps commands, limits acceleration and stops after
0.5 seconds without input. Evidence is written to
`evidence/phase-b-launch-b/`.

## Quickstart (Phase C)

Phase C is independently launchable and adds exactly one SLAM system:
`slam_toolbox`. It preserves the Phase B control, `/lunabot/odom`, sensor
bridge, Fortress IMU, TF, headless unpause, and RViz QoS/topic fixes:

```bash
cd ~/lunabot-v4
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-map-server \
  ros-humble-rviz2 ros-humble-tf2-tools
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-c" ~/launch-c

python3 tools/validate_phase_c.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c        # runtime/map evidence gate
EVIDENCE=1 ~/launch-c                          # GUI + RViz validation
```

The SLAM data path is `/lunabot/lidar/scan` plus `odom → chassis` and
`/lunabot/odom`, producing `/map` and `map → odom`. The Phase C guide is
`docs/phase-3-launch-c.md`; runtime evidence and the workstation checklist
are under `evidence/phase-c-launch-c/`. Phase C was runtime-validated and
explicitly approved before starting Phase D.

## Quickstart (Phase D)

Phase D is independently launchable and adds one custom A* grid planner and
conservative path follower. It consumes `/map`, `map → odom`, and real
`/lunabot/odom`, then publishes only to `/cmd_vel_in` so the validated Phase B
controller remains in the command path:

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-d" ~/launch-d

python3 tools/validate_phase_d.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d        # autonomous runtime gate
EVIDENCE=1 ~/launch-d                          # GUI/RViz path validation
```

Phase D uses no Nav2, AMCL, Cartographer, or second SLAM system. Runtime
evidence is written to `evidence/phase-d-launch-d/`. Phase D was runtime
validated and explicitly approved before Phase E began.

## Quickstart (Phase E)

Phase E preserves the approved Phase D stack and adds a deterministic RGB-D
terrain-perception node. It consumes the existing camera and depth streams and
publishes a `mono8` terrain/obstacle/unknown mask, an RGB overlay, and an
auditable status topic. It does not alter the map, planner, controller, or
velocity path:

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-e" ~/launch-e

python3 tools/validate_phase_e.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e        # RGB-D + A* runtime gate
EVIDENCE=1 ~/launch-e                          # GUI/RViz perception run
```

Phase E outputs are `/lunabot/terrain/segmentation`,
`/lunabot/terrain/overlay`, and `/lunabot/terrain/segmentation/status`.
Runtime evidence is written to `evidence/phase-e-launch-e/`. The Phase E guide
is `docs/phase-5-launch-e.md`. Phase E was runtime-validated and explicitly
approved before Phase F began.

## Quickstart (Phase F)

Phase F preserves the approved Phase E RGB-D perception and adds a semantic
terrain mapper. It projects real segmentation/depth observations through the
existing `map -> chassis` TF into `/lunabot/terrain/semantic_map`. The semantic
map is evidence and visualization output only; it does not alter the A* or
Phase B controller path.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-f" ~/launch-f

python3 tools/validate_phase_f.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f        # semantic-map runtime gate
EVIDENCE=1 ~/launch-f                          # GUI/RViz semantic-map run
```

Phase F outputs `/lunabot/terrain/semantic_map` and
`/lunabot/terrain/semantic_map/status`. Runtime evidence is written to
`evidence/phase-f-launch-f/`. The Phase F guide is
`docs/phase-6-launch-f.md`. Phase F was runtime-validated and explicitly
approved before Phase G began.

## Quickstart (Phase G)

Phase G preserves the approved Phase F semantic terrain map and adds an
inflated traversability cost map. It converts semantic values into numeric
costs but does not yet connect those costs to A*; that is reserved for Phase H.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-g" ~/launch-g

python3 tools/validate_phase_g.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g        # cost-map runtime gate
EVIDENCE=1 ~/launch-g                          # GUI/RViz cost-map run
```

Phase G outputs `/lunabot/terrain/cost_map` and
`/lunabot/terrain/cost_map/status`. Runtime evidence is written to
`evidence/phase-g-launch-g/`. The Phase G guide is
`docs/phase-7-launch-g.md`. Phase G was runtime-validated and explicitly
approved before Phase H began.

## Quickstart (Phase H)

Phase H preserves the approved Phase G cost map and adds a weighted terrain-aware
A* path output. It does not yet connect that path to rover motion; Phase I will
handle full autonomous integration.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-h" ~/launch-h

python3 tools/validate_phase_h.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h        # terrain-aware path gate
EVIDENCE=1 ~/launch-h                          # GUI/RViz path run
```

Phase H outputs `/lunabot/terrain/plan` and
`/lunabot/terrain/planner/status`. Runtime evidence is written to
`evidence/phase-h-launch-h/`. The Phase H guide is
`docs/phase-8-launch-h.md`. Phase H was runtime-validated and explicitly
approved before Phase I began.

## Quickstart (Phase I)

Phase I performs the first full integration: the approved terrain-aware plan
feeds an active path follower, which publishes through `/cmd_vel_in` into the
approved Phase B controller. The diagnostic Phase D A* remains available but
is isolated from the active command boundary.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-i" ~/launch-i

python3 tools/validate_phase_i.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i        # full integration gate
EVIDENCE=1 ~/launch-i                          # GUI/RViz integration run
```

Phase I outputs `/lunabot/autonomy/status` and the active `/cmd_vel_in`
command path. Runtime evidence is written to `evidence/phase-i-launch-i/`.
The Phase I guide is `docs/phase-9-launch-i.md`. Phase I passed static
validation, two independent headless runtime gates, integrated goal
completion, final map evidence, clean shutdown, and clean relaunch. Phase I
was explicitly approved before Phase J began.

## Quickstart (Phase J)

Phase J preserves the approved Phase I autonomous integration and adds a live
replanning monitor. It proves that the terrain-aware planner emits changed
path revisions after real rover motion without adding a competing motion
publisher.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-j" ~/launch-j

python3 tools/validate_phase_j.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j        # dynamic-replanning gate
EVIDENCE=1 ~/launch-j                          # GUI/RViz run
```

Phase J outputs `/lunabot/autonomy/replan_status` in addition to the approved
Phase I interfaces. Runtime evidence is written to
`evidence/phase-j-launch-j/`. The Phase J guide is
`docs/phase-10-launch-j.md`. Phase J passed static validation, two independent
headless runtime gates, dynamic-plan revision evidence, final map evidence,
clean shutdown, and clean relaunch. Phase J was explicitly approved before
Phase K began.

## Quickstart (Phase K)

Phase K preserves the approved Phase J stack and adds an observation-only
runtime evaluator. It aggregates real plan, replanning, goal, controller,
odometry, and command-boundary evidence without publishing motion commands.

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
ln -sfn "$PWD/launch-k" ~/launch-k

python3 tools/validate_phase_k.py             # static gate
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k        # evaluation gate
EVIDENCE=1 ~/launch-k                          # GUI/RViz run
```

Phase K outputs `/lunabot/evaluation/status` in addition to the approved
Phase J interfaces. Runtime evidence is written to
`evidence/phase-k-launch-k/`. The Phase K guide is
`docs/phase-11-launch-k.md`. Do not begin Phase L until the aggregate
evaluation, clean shutdown, and clean relaunch are explicitly approved.

Every later phase will follow the same contract: one command, clean state,
runtime-validated, documented, evidenced.

## Repository layout

```
launch-a                        Phase A entry point (~/launch-a)
scripts/                        launch scripts + teleop node
src/lunabot_gazebo/
  worlds/lunar_world.sdf        lunar world (terrain, horizon, plugins)
  worlds/meshes/                generated lunar terrain meshes
  models/lunabot_v4/model.sdf   rover (rocker-bogie, 6 wheels, sensors)
rviz/                           RViz display configs
tools/                          terrain generator, validators, plotting
docs/                           per-phase documentation
evidence/                       per-phase runtime + static evidence
```

## Rebuild the lunar terrain (only if needed)

```bash
python3 tools/generate_lunar_terrain.py           # deterministic, seed 42
python3 tools/generate_lunar_terrain.py --seed 7  # different crater field
```
After regenerating, update `SPAWN_Z` in `scripts/launch-a.sh` from the
new `evidence/phase-a-launch-a/terrain_stats.txt` (the file prints the
suggested spawn z).
