# Phase 1 — Lunar Base Camp

**Theme:** Build the world before teaching the rover to explore it.
**Deliverable:** A working Lunar Base Camp simulation.
**Gate status:** see `docs/certification-record.md`.

## Objective

A functioning simulated environment: lunar-like ground, mobile rover,
working sensors, and a reliable communication interface between components.
No AI and no autonomous navigation yet — manual control is explicitly
acceptable.

## What was built

| Component | Location | Interface |
|---|---|---|
| Lunar world (gravity 1.62 m/s², regolith ground, rock field, demo triad) | `src/lunabot_gazebo/worlds/lunar_base_camp.sdf` | Gazebo world `lunar_base_camp` |
| Rover model (skid-steer, sensor mast) | `src/lunabot_description/models/lunabot_v4/` | model `lunabot_v4` |
| RGB camera (640×480 @ 15 Hz) | sensor `lunabot_camera` | `/lunabot/camera/image_raw` (`sensor_msgs/Image`) |
| LiDAR (720 beams, 360°, 0.15–40 m @ 10 Hz) | sensor `lunabot_lidar` | `/lunabot/lidar/scan` (`sensor_msgs/LaserScan`) |
| IMU | sensor `lunabot_imu` | `/lunabot/imu` (`sensor_msgs/Imu`) |
| Wheel odometry (DiffDrive, encoder-based) | rover plugin | `/lunabot/odom` (`nav_msgs/Odometry`), `/tf` |
| Manual control | `src/lunabot_teleop/` | `/cmd_vel` (`geometry_msgs/Twist`) |
| ROS 2 ↔ Gazebo bridge + static sensor TFs | `src/lunabot_gazebo/launch/sim.launch.py`, `scripts/run_phase1_sim.sh` | — |
| Automated acceptance test | `src/lunabot_tests/` | writes `evidence/phase1/` |

**Ground-truth policy:** `/model/lunabot_v4/pose` exists as an
evaluation-only reference stream. No component reads it to make the
simulation work; the acceptance test only consumes rover interfaces.

## How to run

Prerequisites: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress,
`ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-tf2-ros`.

```bash
# Terminal 1 — simulation + bridge + teleop
scripts/run_phase1_sim.sh

# Terminal 2 — automated acceptance test (run while the sim is up)
scripts/run_phase1_acceptance.sh
```

ROS-native alternative (after `colcon build`):

```bash
ros2 launch lunabot_gazebo sim.launch.py
ros2 run lunabot_tests phase1_base_camp
```

## Test procedure

### A. Automated acceptance (repeatable)

1. Start the simulation with `scripts/run_phase1_sim.sh --no-teleop`.
2. Run `scripts/run_phase1_acceptance.sh`.
3. The test verifies, in order:
   * `topics_active` — camera, scan, odom, imu all publish;
   * `camera_frames_valid` — well-formed non-empty images;
   * `range_sensor_valid` — ≥180 beams with finite returns;
   * `obstacle_detectable` — min forward-sector range < 5 m (demo triad);
   * `motion_forward` — odometry displacement ≥ 0.30 m for a 3 s forward cmd;
   * `motion_rotate` — |Δyaw| ≥ 0.30 rad for a 3 s rotate cmd;
   * `motion_stop` — commanded velocities decay below threshold after stop.
4. Evidence is written to `evidence/phase1/` (JSON + markdown; commit it).
5. Restart the simulation and repeat — results must reproduce.

### B. Manual checks (human evidence)

1. `ros2 topic echo /lunabot/odom` — values change while driving.
2. `ros2 run rqt_image_view rqt_image_view` — live camera while driving.
3. Drive with the teleop: forward/backward (w/s), rotate (a/d), stop (space).

### C. Creative demonstration — "The obstacle course"

1. Keep the default world: three rocks sit directly ahead of the spawn pose.
2. Start the sim and show the live camera feed beside the LiDAR readout
   (`ros2 topic echo /lunabot/lidar/scan` or RViz LaserScan).
3. Manually drive around the three rocks.
4. Narrate: as each rock enters the camera view, the range sensor reports a
   closing distance — the same physical obstacle seen by two sensors.

## Pass gate: Base Camp Certification

| # | Criterion | Evidence | Status |
|---|---|---|---|
| 1 | The world launches successfully | run log + GUI | [ ] |
| 2 | The rover moves and responds to commands | `motion_forward`, `motion_rotate`, `motion_stop` in `evidence/phase1/` | [ ] |
| 3 | Sensor data available through configured interfaces | `topics_active`, `camera_frames_valid`, `range_sensor_valid` | [ ] |
| 4 | Simulation restartable; same tests repeat | two consecutive acceptance runs | [ ] |
| 5 | Physical obstacle detectable via range sensor | `obstacle_detectable` | [ ] |
| 6 | No component reads Gazebo ground-truth pose | architecture + test policy field `ground_truth_used: false` | [ ] |
| 7 | Creative demonstration performed | demo notes / recording | [ ] |

Do not proceed to Phase 2 until this table is complete and the evidence is
committed.

## Notes / known limitations of this phase

* The compact world uses primitive geometry (sphere rocks, flat regolith).
  The full settlement (Habitat Hub, Solar Farm, Communications Relay,
  Science Station, Mining Site, Landing Zone, Shadowed Research Site) is a
  later expansion, as allowed by the brief.
* Wheel odometry comes from the DiffDrive plugin's wheel-rotation
  integration — it drifts, which is exactly why Phase 3 exists.
