# LunaBot V4 — Autonomous Navigation of a Robot for Lunar Habitats

12-phase development of an autonomous lunar rover, phase-gated: each phase
has its own **independent** launch command, is validated at runtime before
it is approved, and every later phase reuses the validated baseline of the
earlier ones.

## Phases

| Phase | Launch | Scope | Status |
|---|---|---|---|
| A | `~/launch-a` | Simulation & rover foundation (lunar world, rover, sensors, ROS bridge, TF, RViz, teleop) | implemented, pending runtime approval |
| B | `~/launch-b` | Control & odometry | not started |
| C | `~/launch-c` | SLAM & localization | not started |
| D | `~/launch-d` | Basic autonomous navigation (A*) | not started |
| E | `~/launch-e` | Terrain perception (segmentation) | not started |
| F | `~/launch-f` | Semantic terrain mapping | not started |
| G | `~/launch-g` | Terrain cost map | not started |
| H | `~/launch-h` | Terrain-aware path planning | not started |
| I | `~/launch-i` | Full autonomous integration | not started |
| J | `~/launch-j` | Dynamic replanning | not started |
| K | `~/launch-k` | Testing & evaluation | not started |
| L | `~/launch-l` | Final mission demonstration | not started |

Each phase has a full document under `docs/` (23 sections: objective,
inputs, data flow, ROS nodes/topics/services, TF frames, launch command,
validation procedure, success criteria, evidence, limitations).

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
