# Phase B — Control & Odometry (`launch-b`)

## 1. Objective

Phase B adds a ROS-side control boundary and an auditable odometry monitor to
the validated Phase A lunar rover. The control boundary accepts commands on
`/cmd_vel_in`, clamps them to the rover limits, applies acceleration limits,
and publishes the safe command on `/cmd_vel` for the existing Gazebo DiffDrive
plugin. The monitor measures the real `/lunabot/odom` stream; it does not
invent or replace odometry.

## 2. Previous Phase Dependency

Phase B reuses the Phase A lunar world, crater meshes, rover model, sensor
bridge, static sensor TF, RViz baseline, spawn height and WASD implementation.
Phase A remains independently launchable with `~/launch-a` and its default
teleop topic remains `/cmd_vel`. Phase B is an independent launch and does not
invoke `launch-a`.

## 3. New Functionality

| Component | New behavior |
|---|---|
| Control node | `/cmd_vel_in` → clamp → acceleration-limited `/cmd_vel` |
| Watchdog | If no input arrives for 0.5 s, the output ramps to zero and status becomes `WATCHDOG_STOP` |
| Control status | `/lunabot/control/status` reports input age, target/output values, limits and message counts |
| Odometry monitor | Consumes real `/lunabot/odom`, checks frame IDs, sample rate, continuity and motion, and publishes quality status |
| Odometry evidence | Writes `odometry_samples.csv` and `odometry_report.txt` from received runtime messages |
| Demo path | `wasd_teleop.py --topic /cmd_vel_in --demo` exercises the complete control chain |

## 4. Inputs

| Input | Source | Interface |
|---|---|---|
| `/cmd_vel_in` | teleop or future planner | `geometry_msgs/msg/Twist` |
| `/lunabot/odom` | Phase A DiffDrive bridge | `nav_msgs/msg/Odometry` |
| `/lunabot/joint_states` | Phase A model | `sensor_msgs/msg/JointState` |
| `/clock` | Gazebo bridge | `rosgraph_msgs/msg/Clock` |
| Phase A world/model | repository | SDF + terrain meshes |

The control node uses wall time for its safety watchdog so it remains a safety
mechanism even if simulation time pauses. The demo uses `/clock` to measure
motion duration fairly when Gazebo runs slower than real time.

## 5. Processing/Data Flow

```
WASD/demo/planner
      │ geometry_msgs/Twist
      ▼
/cmd_vel_in ──> lunabot_control
                 │ clamp ±0.45 m/s, ±1.0 rad/s
                 │ acceleration limit 0.4 m/s², 0.8 rad/s²
                 │ 0.5 s input watchdog
                 ▼
             /cmd_vel ──> ROS↔Gazebo bridge ──> DiffDrive
                                             └─> six wheel joints
                                             └─> /lunabot/odom + /tf
                                                       │
                                                       ▼
                                           lunabot_odometry_monitor
                                           ├─ /lunabot/odometry/status
                                           ├─ odometry_samples.csv
                                           └─ odometry_report.txt
```

## 6. Outputs

| Output | Meaning |
|---|---|
| `/cmd_vel` | Safe, smoothed drive command consumed by the existing DiffDrive plugin |
| `/lunabot/control/status` | Controller state, watchdog age, target/output and counters |
| `/lunabot/odom` | Phase A wheel odometry, preserved as the source of truth |
| `/lunabot/odometry/status` | Live monitor quality state |
| `odometry_samples.csv` | Every odometry sample observed by the monitor |
| `odometry_report.txt` | Runtime rate, duration, distance, yaw, continuity and quality result |

## 7. ROS Nodes

| Node | Executable | Purpose |
|---|---|---|
| `lunabot_control` | `scripts/control_odometry.py` | command clamp, acceleration limiting and watchdog |
| `lunabot_odometry_monitor` | `scripts/odometry_monitor.py` | measurement and evidence of real odometry |
| `wasd_teleop` | `scripts/wasd_teleop.py` | interactive or automated input producer |
| `parameter_bridge` | `ros_ign_bridge`/`ros_gz_bridge` | Phase A ROS↔Gazebo transport |
| `static_transform_publisher` ×4 | `tf2_ros` | Phase A sensor frames |
| `rviz2` | `rviz2` | Phase B visualization |

## 8. ROS Topics

| Topic | Type | Direction | Role |
|---|---|---|---|
| `/cmd_vel_in` | `geometry_msgs/Twist` | in to control | raw command source |
| `/cmd_vel` | `geometry_msgs/Twist` | control to bridge | safe command to DiffDrive |
| `/lunabot/control/status` | `std_msgs/String` | out | controller/watchdog status |
| `/lunabot/odom` | `nav_msgs/Odometry` | out | DiffDrive wheel odometry |
| `/lunabot/odometry/status` | `std_msgs/String` | out | monitor quality status |
| `/clock` | `rosgraph_msgs/Clock` | out | Gazebo simulation time |
| `/tf` | `tf2_msgs/TFMessage` | out | odom/chassis and sensor TF |
| Phase A sensor topics | Image/LaserScan/Imu/JointState | out | preserved baseline interfaces |

## 9. ROS Services

Phase B introduces no custom services. Gazebo world startup and entity spawn
continue to use the Gazebo Sim services already used by Phase A. The control
and odometry interfaces are topics so later planners can replace the teleop
source without changing the rover or bridge.

## 10. TF Frames

The Phase A TF contract is unchanged:

```
odom → chassis → sensor_head → rgb_camera
                             ├→ depth_camera
                             └→ lidar
```

The monitor requires `/lunabot/odom` to identify `odom` as its header frame and
`chassis` as its child frame. The four sensor transforms remain static and the
`odom → chassis` transform remains produced by the DiffDrive system.

## 11. Launch Command

```bash
~/launch-b
```

Automated evidence run:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-b
```

The launch is standalone: it starts the world, spawns the rover, starts the
bridge, static TF, Phase B nodes and RViz directly. It does not call
`launch-a` or depend on a previous launch process.

## 12. Expected Terminal Output

The standardized output has a Phase B banner and twelve stages:

```text
============================================================
                 LUNABOT V4
                 PHASE B
                 CONTROL & ODOMETRY
============================================================
[1/12] Checking Phase A baseline + Phase B files.....
[2/12] Checking ROS 2 / Gazebo environment........
[3/12] Checking for stale Phase A/B processes.......
[4/12] Starting Gazebo lunar world..................
[5/12] Spawning LunaBot V4..........................
[6/12] Starting ROS 2 <-> Gazebo bridge.............
[7/12] Starting static TF (sensor frames)............
[8/12] Starting Phase B control layer................
[9/12] Starting odometry monitor....................
[10/12] Starting RViz2..............................
[11/12] Runtime validation..........................
[12/12] status printed (overall PASS)
```

The automated run then prints a real demo result and exits nonzero if the
control chain or motion acceptance fails.

## 13. Expected Gazebo Output

Gazebo must show the same grey, monochrome, dense crater field as Phase A.
LunaBot V4 must spawn on the terrain at `z=-2.308 m`. The rover receives
commands only after they pass through the Phase B controller. When the input
stops, the controller's watchdog ramps `/cmd_vel` to zero rather than leaving
a stale command active.

## 14. Expected RViz Output

RViz uses fixed frame `odom`. The Phase B configuration shows the Phase A TF,
LiDAR and camera displays plus an Odometry display on `/lunabot/odom`. The
odometry display is visualization only; the monitor's report is calculated
from received messages.

## 15. Validation Procedure

On an Ubuntu 22.04 workstation with ROS 2 Humble and Gazebo Sim:

1. Pull the approved branch and create the `~/launch-b` symlink.
2. Run `EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-b`.
3. Confirm startup checks pass for `/clock`, `/lunabot/odom`, control status,
   odometry status, sensors and TF.
4. Confirm `demo_drive_result.txt` reports `result : PASS`.
5. Confirm `odometry_report.txt` has nonzero duration, a real sample rate,
   `continuity_frames : PASS` and `quality_result : PASS`.
6. Inspect `diag_drive.csv` and `odometry_samples.csv`; these must contain
   real workstation measurements, not hand-written values.
7. Run `~/launch-b`, press W/S/A/D/Space, and check the Gazebo motion.
8. Wait without input and confirm the control status reports `WATCHDOG_STOP`.
9. Press Q or Ctrl+C; run `~/launch-b` a second time to verify clean restart.
10. Use `verification_checklist.md` and provide a Gazebo/RViz screenshot for
    visual approval.

## 16. Success Criteria

- Phase A world, rover, sensors, TF and terrain remain present.
- `launch-b` is independently runnable from a clean state.
- `/cmd_vel_in` is visibly connected to `/cmd_vel` through the control node.
- Commands are clamped to ±0.45 m/s and ±1.0 rad/s and ramped by configured
  acceleration limits.
- A missing input for 0.5 s produces a safe stop.
- Real `/lunabot/odom` messages arrive with `odom → chassis` identity,
  nonzero runtime duration and continuous pose samples.
- The automated control-chain demo passes and writes both diagnostics files.
- Ctrl+C/Q terminates Gazebo, bridge, control, monitor, TF and RViz with no
  orphaned Phase B processes.

## 17. Repository Files

| File | Role |
|---|---|
| `launch-b` | canonical root entry point |
| `scripts/launch-b.sh` | independent twelve-stage Phase B launcher |
| `scripts/control_odometry.py` | clamp, acceleration limit and watchdog |
| `scripts/odometry_monitor.py` | odometry measurement and report |
| `scripts/wasd_teleop.py` | Phase A-compatible teleop with topic override |
| `rviz/phase_b.rviz` | Phase B RViz layout |
| `tools/validate_phase_b.py` | static Phase B gate |
| `evidence/phase-b-launch-b/` | runtime collector, checklist and evidence |

## 18. Files Created

- `launch-b`
- `scripts/launch-b.sh`
- `scripts/control_odometry.py`
- `scripts/odometry_monitor.py`
- `rviz/phase_b.rviz`
- `tools/validate_phase_b.py`
- `evidence/phase-b-launch-b/README.md`
- `evidence/phase-b-launch-b/verification_checklist.md`
- `evidence/phase-b-launch-b/collect_evidence.sh`

## 19. Files Modified

- `scripts/wasd_teleop.py`: added `--topic`; default `/cmd_vel` is preserved
  for Phase A and Phase B passes `/cmd_vel_in`.
- `README.md`: Phase B quickstart and status updated after validation.
- `docs/phase-b-launch-b.md`: this document.

No Phase A world, terrain, rover geometry, sensor, bridge or TF values were
changed for Phase B.

## 20. Files Reused

- Phase A `lunar_world.sdf`, both terrain meshes and the validated `model.sdf`.
- Phase A `launch-a` remains an independent baseline.
- Phase A bridge mappings, spawn height, static TF values and teleop behavior.
- Phase A `rviz/phase_a.rviz` as the source layout for the Phase B RViz config.

## 21. Output Passed to Next Phase

Phase C receives these stable interfaces:

- validated lunar world and rover baseline;
- `/cmd_vel_in` as the planner/control input boundary;
- safe `/cmd_vel` output into the rover;
- `/lunabot/odom` and `odom → chassis` TF;
- control and odometry status/evidence interfaces;
- camera, depth, LiDAR and IMU topics from Phase A.

Phase C must reuse these interfaces and must not replace the lunar terrain.

## 22. Evidence

Static evidence:

```text
evidence/phase-b-launch-b/static_validation.txt
```

Runtime evidence is generated only on the user's workstation:

```text
evidence/phase-b-launch-b/last_run.log
evidence/phase-b-launch-b/control_status.txt
evidence/phase-b-launch-b/odometry_status.txt
evidence/phase-b-launch-b/odometry_samples.csv
evidence/phase-b-launch-b/odometry_report.txt
evidence/phase-b-launch-b/demo_drive_result.txt
evidence/phase-b-launch-b/diag_drive.csv
```

These files must be inspected as produced. No runtime values are claimed by
this repository before the workstation run.

## 23. Known Limitations

1. The development sandbox cannot execute ROS 2/Gazebo, so static validation
   is local but runtime validation remains workstation-gated.
2. The odometry source is still the Gazebo DiffDrive wheel odometry; Phase B
   does not fuse IMU or estimate localization. Sensor fusion is later scope.
3. The control node is a safety/smoothing boundary, not a path-following
   controller. Autonomous command generation begins in later phases.
4. The watchdog uses wall time intentionally. The demo's motion duration uses
   Gazebo `/clock` intentionally; these are different clocks for different
   safety/measurement purposes.
5. The reference lunar screenshot was not attached in this conversation, so
   visual acceptance remains the user's workstation gate.

============================================================
PHASE B COMPLETE — PENDING USER VALIDATION
============================================================

Does this match the expected Phase B control and odometry behavior? Please run
the automated and interactive commands, inspect the evidence and screenshot,
and explicitly approve Phase C only after validation.
