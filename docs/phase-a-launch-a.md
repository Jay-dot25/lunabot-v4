# Phase A — Simulation & Rover Foundation (`launch-a`)

## 1. Objective

Establish the complete simulated LunaBot foundation: a realistic lunar
environment (large cratered terrain, monochrome grey appearance, lunar
gravity) with the LunaBot V4 rover (rocker-bogie, six wheels, RGB camera,
depth camera, LiDAR, IMU) running in Gazebo Sim, exposed through ROS 2
topics and TF, viewable in RViz2, and drivable by manual teleoperation.

Phase A is the baseline that every later phase builds on. It must launch
independently from a clean state with a single command.

## 2. Previous Phase Dependency

None. Phase A is the first phase. It reuses the existing repository assets
(rover model, world skeleton, teleop script) and completes them.

## 3. New Functionality

| Item | What was added/fixed |
|---|---|
| Lunar terrain asset | **Created** `worlds/meshes/lunar_terrain.obj` (visual, 320×320 grid, 1.25 m, per-vertex grey albedo) + `lunar_terrain_collision.obj` (100×100, 4 m). The world referenced `meshes/lunar_terrain.obj` but **no mesh existed** — the world previously had no ground at all. Generated deterministically by `tools/generate_lunar_terrain.py` (seed 42). |
| Terrain geometry | 400 m × 400 m crater field: 220 small craters (3–10 m), 70 medium (12–30 m), 7 large (44–90 m, with rims and central peaks), multi-octave rolling relief (~14 m total relief), fine rocky roughness, smooth spawn pad at the origin, flat boundary shelf. |
| World physics | Terrain split into high-detail visual + low-detail collision mesh (fast DART physics). Lunar gravity `0 0 -1.62` (already present, preserved). Horizon catch-plane added so driving past the terrain edge does not void the world. |
| Launch script | `scripts/launch-a.sh` rewritten: 10 staged startup steps with live `OK/FAIL`, real runtime validation (topic + TF checks, not assumptions), `HEADLESS`/`DEMO`/`EVIDENCE` modes, trap-based clean shutdown with orphan guard, log capture to `evidence/phase-a-launch-a/`. |
| Sensor bridge | Bridge extended from 5 to 15 topic mappings: added depth camera, IMU, joint states, steer joints, mast pan/tilt. |
| TF | Replaced the single approximate `chassis→lidar` static TF with an accurate 4-frame static tree taken from the model SDF (`chassis→sensor_head→{rgb_camera, depth_camera, lidar}`). |
| Entry point | `launch-a` at the repo root (canonical `~/launch-a`), works from any directory. |
| RViz | New `rviz/phase_a.rviz` config: Grid + TF + LaserScan + Camera image, fixed frame `odom` (previously RViz started with an empty default display). |
| Teleop | `scripts/wasd_teleop.py`: same WASD/Space/Q controls, speeds corrected to the DiffDrive plugin limits (0.45 m/s / 1.0 rad/s — the old 2.0 m/s request was silently clamped), plus `--demo` automated drive test for headless validation. |
| Docs + evidence | This document, `evidence/phase-a-launch-a/` (terrain previews, stats, static validation, runtime evidence collector). |

## 4. Inputs

- `~/launch-a` (single command; environment: `HEADLESS`, `DEMO`, `EVIDENCE`)
- Keyboard: `W/S/A/D` (drive), `Space` (stop), `Q` (quit)
- `/cmd_vel` (`geometry_msgs/Twist`) — the only runtime drive input
- System requirements: ROS 2 Humble, Gazebo Sim (Fortress `ign` or Garden+ `gz`), `ros_ign_bridge`, `rviz2`

## 5. Processing/Data Flow

```
lunar_world.sdf + terrain meshes ──> Gazebo Sim (lunar gravity, OG2 renderer)
                                           │
lunabot_v4/model.sdf ──(EntityFactory spawn at z=-2.308)──> Gazebo
                                           │
        ┌──────────────┬──────────────┬────┴─────────┬──────────────┐
   rgb_camera     depth_camera     gpu_lidar         IMU      DiffDrive plugin
   /lunabot/camera  /lunabot/depth  /lunabot/lidar    /lunabot/imu   /lunabot/odom
   /image_raw       /image_raw      /scan                            /tf (Pose_V)
        └──────────────┴──────────────┴──────────────┴──────────────┘
                                           │  ros_ign_bridge (15 mappings)
                                           v
                                  ROS 2 (Humble)
        /cmd_vel (Twist) ─────────────────────────> DiffDrive plugin (gazebo)
        TF: odom -> chassis (from /tf) + static chassis -> sensor frames
                                           │
                                           v
                              RViz2 (phase_a.rviz) + WASD teleop
```

Drive loop: keypress → `wasd_teleop.py` publishes `/cmd_vel` → bridge →
DiffDrive plugin commands the 6 wheel joints → rover moves on the terrain →
odometry + TF updated → RViz + ROS interfaces reflect the state.

## 6. Outputs

- Gazebo GUI window: lunar terrain + rover (GUI camera at `0 -300 200`)
- RViz2 window: TF tree, LiDAR scan, camera image (fixed frame `odom`)
- All topics in §8
- TF tree in §10
- Runtime logs: `evidence/phase-a-launch-a/{last_run,gazebo,bridge}.log`
- Runtime evidence (with `EVIDENCE=1`): `topics.txt`, `topic_info.txt`,
  `tf_odom_chassis.txt`, `odom_sample.txt`, `lidar_scan_sample.yaml`,
  `lidar_scan_preview.png`, `demo_drive_result.txt`

## 7. ROS Nodes

| Node | Package | Purpose |
|---|---|---|
| `parameter_bridge` | `ros_ign_bridge` (auto-detected; `ros_gz_bridge` on Garden+) | 15 ROS↔Gazebo topic mappings |
| `wasd_teleop` | this repo (`scripts/wasd_teleop.py`) | keyboard → `/cmd_vel`; `--demo` drive test |
| 4× `static_transform_publisher` | `tf2_ros` | sensor-frame static TF |

Gazebo-side (not ROS nodes, run inside Gazebo Sim):
`Physics`, `SceneBroadcaster`, `UserCommands`, `Sensors`, `Contact`
systems + per-model `DiffDrive`, `JointPositionController` (×6),
`JointStatePublisher`.

## 8. ROS Topics

| Topic | Type | Dir | Hz | Notes |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in | user | drive command (clamped by DiffDrive: 0.45 m/s, 1.0 rad/s) |
| `/lunabot/odom` | `nav_msgs/Odometry` | out | 30 | wheel odometry, frame `odom` |
| `/lunabot/camera/image_raw` | `sensor_msgs/Image` | out | 20 | RGB 640×480 |
| `/lunabot/depth/image_raw` | `sensor_msgs/Image` | out | 15 | depth 640×480 |
| `/lunabot/lidar/scan` | `sensor_msgs/LaserScan` | out | 10 | gpu_lidar, 720 beams, 0.15–60 m |
| `/lunabot/imu` | `sensor_msgs/Imu` | out | 100 | with configured noise |
| `/lunabot/joint_states` | `sensor_msgs/JointState` | out | – | 16 joints |
| `/lunabot/steer/front_left`, `/lunabot/steer/front_right`, `/lunabot/steer/rear_left`, `/lunabot/steer/rear_right` | `std_msgs/Float64` | in | user | steering joint positions (rad) |
| `/lunabot/mast/pan`, `/lunabot/mast/tilt` | `std_msgs/Float64` | in | user | mast joint positions (rad) |
| `/tf` | `tf2_msgs/TFMessage` | out | – | `odom→chassis` (DiffDrive) + static sensor frames |

## 9. ROS Services

No ROS 2 services are used in Phase A.

Gazebo Sim services used by the launch script:
- `ign service -s /world/lunar_world/info` (startup readiness)
- `ign service -s /world/lunar_world/create` (rover spawn, `EntityFactory`)

## 10. TF Frames

```
odom ──(DiffDrive plugin, dynamic)──> chassis
                                        │ static (0.18, 0, 0.85)
                                        v
                                   sensor_head
                                        │ static
              ┌─────────────────────────┼──────────────────────────┐
              v                         v                          v
        rgb_camera                depth_camera                   lidar
   (0.14, 0, 0)                (0.14, 0, -0.04)          (0.02, 0, 0.09, pitch 0.5)
```

All offsets are taken from `model.sdf` link poses (joints at rest).
The `map` frame does not exist yet — it is introduced in Phase C (SLAM).

## 11. Launch Command

```bash
~/launch-a                    # interactive: Gazebo GUI + RViz2 + WASD
HEADLESS=1 DEMO=1 ~/launch-a  # fully automated runtime test (no GUI)
EVIDENCE=1 DEMO=1 ~/launch-a  # automated test + record runtime evidence
```

`~/launch-a` → `lunabot-v4/launch-a` → `scripts/launch-a.sh`.
Setup on a fresh machine: `ln -s ~/lunabot-v4/launch-a ~/launch-a`.

The command is **independent**: it checks files, sources ROS 2, kills stale
`lunar_world` processes, starts Gazebo, spawns the rover, starts the
bridge/TF/RViz, validates at runtime, and shuts everything down cleanly on
Ctrl+C (or after the demo run).

## 12. Expected Terminal Output

```
============================================================
                 LUNABOT V4
                 PHASE A
                 SIMULATION & ROVER FOUNDATION
============================================================

Launch mode: GUI | demo=no | evidence=no
[1/10] Checking project files....................
      world, rover model, terrain meshes, teleop, rviz: OK
[2/10] Checking ROS 2 / Gazebo environment.......
      ROS 2 Humble: OK | Gazebo Sim: ign (ignition.msgs) | ros_ign_bridge: OK
[3/10] Checking for stale processes................
      clean state: OK
[4/10] Starting Gazebo (lunar world)...............
      Gazebo running (PID ...), lunar_world loaded
[5/10] Spawning LunaBot V4.........................
      LunaBot V4 spawned at (0.0, 0.0, -2.308)
[6/10] Starting ROS 2 <-> Gazebo bridge............
      bridge running (PID ...), 15 topic mappings
[7/10] Starting static TF (sensor frames)...........
      4 static transforms published (chassis -> sensor frames)
[8/10] Starting RViz2...............................
      RViz2 running (PID ...), fixed frame: odom
[9/10] Runtime validation...........................
      Gazebo process alive: PASS
      topic /lunabot/odom: PASS
      topic /lunabot/camera/image_raw: PASS
      topic /lunabot/lidar/scan: PASS
      topic /lunabot/imu: PASS
      TF odom -> chassis: PASS
      TF chassis -> sensor_head: PASS

------------------------------------------------------------
PHASE STATUS
------------------------------------------------------------
Environment       : RUNNING
LunaBot           : RUNNING
Sensors           : RUNNING (camera, depth, lidar, imu)
Bridge            : RUNNING (15 mappings)
TF                : RUNNING (odom->chassis + 4 static)
Phase Component   : RUNNING
RViz2             : RUNNING

------------------------------------------------------------
AVAILABLE OUTPUTS
------------------------------------------------------------
(... topics, TF frames, camera poses ...)

------------------------------------------------------------
VALIDATION
------------------------------------------------------------
  - all project files present
  - Gazebo + lunar_world started (lunar terrain + horizon)
  - LunaBot V4 spawned on terrain (spawn z=-2.308)
  - bridge + TF + sensor topics verified at runtime
  - overall: PASS

============================================================
 Launch A environment is running.
  Controls:  W = Forward  S = Reverse  A = Turn Left  D = Turn Right
             Space = Stop  Q = Quit
  Press Ctrl+C to stop LunaBot safely.
============================================================
```

DEMO mode additionally prints the drive test result, e.g.:

```
forward 3 s @ 0.45 m/s : distance = 1.284 m (expect > 0.3 m)
turn 3 s @ 0.6 rad/s   : yaw delta  = +1.544 rad (expect > 0.3 rad)
result                : PASS
AUTO RUN COMPLETE - overall result: PASS
```

## 13. Expected Gazebo Output

- Gazebo opens the `lunar_world` world: a 400 m × 400 m **grey, cratered,
  rolling lunar surface** (NOT a flat plane): dense small craters, medium
  craters, several large craters with rims/central peaks, smooth spawn pad
  at the origin.
- GUI camera at `(0, -300, 200)` looks over the crater field toward the
  origin; the rover appears small against the terrain.
- LunaBot V4 (rocker-bogie, six wheels, solar panels, sensor mast) drops
  from ~5 cm and settles on the spawn pad; wheels may articulate slightly
  on the rough surface.
- Sensor visualization gizmos on the sensor head (camera frustum, LiDAR
  fan) when `<visualize>true</visualize>`.
- With `HEADLESS=1`: no window; `gazebo.log` shows sensor/physics startup.

## 14. Expected RViz Output

Fixed frame `odom`:
- **TF**: `odom → chassis` arrow moving with the rover; static
  `chassis → sensor_head → {rgb_camera, depth_camera, lidar}`.
- **LaserScan** (`/lunabot/lidar/scan`): red flat squares forming a 360°
  fan of the crater field around the rover; as the rover moves/turns the
  scan rotates.
- **Camera** panel (`/lunabot/camera/image_raw`): live RGB view of the
  lunar terrain in front of the rover (grey, cratered).
- **Grid**: 10 m cells for scale.

## 15. Validation Procedure

1. Run `EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-a` (or GUI interactive).
2. Confirm all 10 startup stages print OK and runtime validation is all
   `PASS` (topics actually deliver messages; TF actually resolves).
3. Confirm the demo drive test reports `distance > 0.3 m` and
   `|yaw delta| > 0.3 rad` (rover actually moved and turned).
4. Visually check Gazebo against `verification_checklist.md` (lunar
   terrain appearance, rover on terrain).
5. Inspect `evidence/phase-a-launch-a/`: `last_run.log`,
   `lidar_scan_preview.png` (crater ring structure visible),
   `demo_drive_result.txt`, `odom_sample.txt`.
6. Press Ctrl+C: confirm clean shutdown, no orphan
   `gz-sim`/`ignition-gazebo` processes (`pgrep -f lunar_world`).
7. Run the launch a second time from the clean state → must work.

## 16. Success Criteria

- [ ] `~/launch-a` starts the whole stack alone from a clean state
- [ ] Gazebo shows the lunar terrain (visual checklist PASS)
- [ ] Rover spawns on the terrain and stays stable
- [ ] `/lunabot/odom`, `/lunabot/camera/image_raw`,
      `/lunabot/lidar/scan`, `/lunabot/imu` deliver messages
- [ ] `odom → chassis` TF resolves; sensor TF tree complete
- [ ] WASD (or demo) drives the rover; odometry follows motion
- [ ] RViz shows TF + scan + camera image
- [ ] Ctrl+C shuts everything down cleanly; second run works

## 17. Repository Files

```
launch-a                                  # Phase A entry point (~/launch-a)
scripts/launch-a.sh                       # launch logic (staged, validated, clean shutdown)
scripts/wasd_teleop.py                    # teleop + demo drive test
src/lunabot_gazebo/worlds/lunar_world.sdf # lunar world (terrain, horizon, plugins, GUI)
src/lunabot_gazebo/worlds/meshes/lunar_terrain.obj           # visual terrain (16.3 MB)
src/lunabot_gazebo/worlds/meshes/lunar_terrain_collision.obj # collision terrain (1.2 MB)
src/lunabot_gazebo/models/lunabot_v4/model.sdf               # rover (unchanged)
rviz/phase_a.rviz                         # RViz display config
tools/generate_lunar_terrain.py           # deterministic terrain generator (seed 42)
tools/validate_phase_a.py                 # static validation (SDF/mesh/script checks)
tools/plot_lidar_scan.py                  # LiDAR scan -> PNG
docs/phase-a-launch-a.md                  # this document
evidence/phase-a-launch-a/                # terrain previews, stats, validation, runtime logs
```

## 18. Files Created

| File | Reason |
|---|---|
| `src/lunabot_gazebo/worlds/meshes/lunar_terrain.obj` | world referenced a terrain mesh that did not exist; creates the visual lunar terrain |
| `src/lunabot_gazebo/worlds/meshes/lunar_terrain_collision.obj` | low-res physics mesh (fast DART) |
| `launch-a` | canonical `~/launch-a` entry point |
| `rviz/phase_a.rviz` | proper RViz displays (was: bare `rviz2 -f`) |
| `tools/generate_lunar_terrain.py` | reproducible terrain (deterministic, seed 42) |
| `tools/validate_phase_a.py` | static validation without Gazebo |
| `tools/plot_lidar_scan.py` | runtime evidence: LiDAR scan plot |
| `evidence/phase-a-launch-a/*` | terrain previews/stats, validation report, runtime evidence, checklist, collector |

## 19. Files Modified

| File | Change |
|---|---|
| `scripts/launch-a.sh` | rewritten: staged output, real runtime validation, 15-mapping bridge, accurate static TF, resource-path export, `HEADLESS/DEMO/EVIDENCE`, trap-based clean shutdown + orphan guard, evidence logging, corrected spawn `z=-2.308` (was `z=80.2`, which assumed the missing mesh's coordinate system) |
| `scripts/wasd_teleop.py` | speeds corrected to DiffDrive limits; `--demo` headless drive test; graceful handling of non-TTY; same WASD controls preserved |
| `src/lunabot_gazebo/worlds/lunar_world.sdf` | collision/visual mesh separation; horizon catch-plane; stale "missing plugin" comment replaced (plugin kept); gravity/plugins/GUI camera unchanged |
| `docs/phase-1-launch-a.md` → `docs/phase-a-launch-a.md` | renamed to master-prompt naming convention; content written |

## 20. Files Reused

| File | Purpose |
|---|---|
| `src/lunabot_gazebo/models/lunabot_v4/model.sdf` | rover model — **unchanged**: rocker-bogie, 6 wheels, sensors, DiffDrive, joint controllers all work as designed |
| `worlds/lunar_world.sdf` (structure) | world name, GUI camera, gravity, light, physics + 5 systems kept as-is |
| `scripts/wasd_teleop.py` (controls) | WASD/Space/Q interaction model preserved |
| Bridge pattern | `ros_ign_bridge parameter_bridge` pattern (from the original script) extended, not replaced |

## 21. Output Passed to Next Phase

- Running Gazebo lunar environment + rover (Phase B reuses via its own
  independent launch that starts the same world/model)
- `/cmd_vel` in, `/lunabot/odom` + `odom→chassis` TF out — Phase B's
  control & odometry validation pipeline
- Complete sensor ROS interfaces (camera/depth/LiDAR/IMU/joint states)
  for Phases C–L
- Verified baseline: any later phase that breaks these must be detected
  against this baseline

## 22. Evidence

`evidence/phase-a-launch-a/`:

| Artifact | Status |
|---|---|
| `terrain_preview_topdown.png` | generated (mesh render, 400 m view) |
| `terrain_preview_perspective.png` | generated (view ≈ Gazebo GUI camera) |
| `terrain_stats.txt` | generated (extent, relief, craters, spawn height) |
| `static_validation.txt` | generated (SDF/mesh/script cross-checks) |
| `last_run.log`, `gazebo.log`, `bridge.log` | produced by every launch run |
| `topics.txt`, `topic_info.txt`, `tf_odom_chassis.txt`, `odom_sample.txt`, `lidar_scan_sample.yaml`, `lidar_scan_preview.png`, `demo_drive_result.txt` | produced by `EVIDENCE=1 DEMO=1 ~/launch-a` **on a machine with ROS 2 + Gazebo** (this development sandbox has no ROS 2/Gazebo and no access to their package mirrors — see §23) |
| `verification_checklist.md` | visual acceptance checklist (user gatekeeper) |
| `collect_evidence.sh` | one command to regenerate all runtime evidence |

## 23. Known Limitations

1. **Runtime verification in the development sandbox is not possible**:
   the sandbox (Debian 12, 2 CPU / 4 GB, restricted network) has no ROS 2
   or Gazebo and no reachable package mirrors to install them. Everything
   that can be verified statically **has been** (`static_validation.txt`);
   the runtime items in §15/§16 must be confirmed on the workstation with
   `EVIDENCE=1 DEMO=1 ~/launch-a`. Phase A is therefore reported
   `PARTIAL` until that run is shown.
2. Terrain is procedural (deterministic, seed 42), not photogrammetric.
   It matches the written visual spec (grey monochrome, dense crater
   field, large visible craters, rough surface, large area, small rover);
   exact pixel-match with the reference screenshot is not claimed. The
   generator is in-repo — regenerating with a different `--seed` is
   trivial if the field needs to change (regenerate, then update
   `SPAWN_Z` in `launch-a.sh` from the new `terrain_stats.txt`).
3. Collision mesh is 4 m-res: small craters (3–10 m) are smoothed in
   physics (visual-only detail). DiffDrive odometry is kinematic, so this
   does not corrupt odometry; it becomes relevant when terrain-conforming
   control is added (Phase B+).
4. Rover is driven by the DiffDrive plugin (kinematic wheel commands);
   rocker-bogie joints are free but uncontrolled in Phase A (they react
   to contact). Steering/mast joint controllers exist and are bridged but
   not used until control is implemented (Phase B).
5. If the rover is driven more than ~50 m beyond the 400 m × 400 m
   terrain edge it falls onto the horizon catch-plane ~12 m below the
   crater-field surface (flat grey, drivable). This is a safety net, not
   part of the mission area.
6. RGB/depth images are rendered by Gazebo (OGre2); visual quality in
   headless mode depends on the workstation's GL stack.
