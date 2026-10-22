# LunaBot V4 — The Lunar Autonomy Challenge

Autonomous navigation of a rover for lunar habitats. LunaBot must receive a
mission, perceive terrain, estimate its position **without GPS/GNSS**,
identify hazards, select a safe route, navigate autonomously, and report its
performance — earning engineering certifications along the way.

* **Project synopsis (requirements):** `docs/project-synopsis.md`
* **Certification record (phase gates):** `docs/certification-record.md`

## The golden rule

> A phase is complete only when its output works and its pass criteria have
> been demonstrated. Writing code is not sufficient. Training a model is not
> sufficient. Creating an attractive simulation is not sufficient.

If a phase fails: fix that phase, repeat its tests, earn the clearance.

## Certification programme

| Phase | Theme | Certification | Status |
|---|---|---|---|
| 1 | Lunar Base Camp | Base Camp | IN PROGRESS |
| 2 | Eyes of Luna | Perception | LOCKED |
| 3 | Lost Without GPS | GPS-Independent Navigation | LOCKED |
| 4 | The Terrain Intelligence System | Terrain Intelligence | LOCKED |
| 5 | The Route Architect | Route Planning | LOCKED |
| 6 | The Autonomous Pilot | Autonomous Pilot | LOCKED |
| 7 | Lunar Mission Control | Mission Control | LOCKED |
| 8 | The Lunar Proving Ground | Proving Ground | LOCKED |
| Final | Mission LUNA-007 | Full acceptance | LOCKED |

## Repository layout

    docs/                     requirements, certification record, phase procedures
    evidence/                 measured acceptance reports (committed)
    scripts/                  run scripts (simulation, acceptance tests)
    src/
      lunabot_description/    rover model (SDF): camera, LiDAR, IMU, odometry
      lunabot_gazebo/         lunar worlds + simulation launch
      lunabot_teleop/         manual keyboard teleop (development control)
      lunabot_tests/          repeatable acceptance tests per phase

## Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo Fortress (Ignition) with the ROS bridge:

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                 ros-humble-tf2-ros ros-humble-rviz2
```

> Using Gazebo Garden or Harmonic instead? Set
> `LUNABOT_GZ_MSGS=gz.msgs` and `LUNABOT_GZ_CMD="gz sim"` before running the
> scripts.

## Quick start — Phase 1 (Lunar Base Camp)

```bash
# Terminal 1: simulation + rover + sensors + teleop
scripts/run_phase1_sim.sh

# Terminal 2: automated pass-gate checks (writes evidence/phase1/)
scripts/run_phase1_acceptance.sh
```

Drive: `w/s` forward/backward, `a/d` turn, `space` stop, `q` quit.

Interfaces exposed from day one (consumed by every later phase):

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | drive commands |
| `/lunabot/odom` | `nav_msgs/Odometry` | wheel odometry (motion estimate) |
| `/lunabot/camera/image_raw` | `sensor_msgs/Image` | terrain perception input |
| `/lunabot/lidar/scan` | `sensor_msgs/LaserScan` | range sensing / mapping input |
| `/lunabot/imu` | `sensor_msgs/Imu` | supplementary motion info |
| `/model/lunabot_v4/pose` | `tf2_msgs/TFMessage` | **evaluation-only** ground truth |

## Engineering rules (from the synopsis)

1. **Ground truth is for evaluation only.** The controller, mapper, and
   planner must run on estimated pose and sensor data — never on Gazebo's
   true pose.
2. **Evidence or it didn't happen.** Every gate needs a repeatable test and
   saved output in `evidence/`.
3. **One working capability per phase**, consumable by the next phase.
4. **Scope discipline:** one correctly integrated algorithm beats several
   incomplete ones.

## Development workflow

1. Read the phase section in `docs/project-synopsis.md`.
2. Implement against the phase's pass gate in `docs/phase-<n>-*.md`.
3. Run the repeatable test procedure; commit the evidence.
4. Flip the phase to `PASSED` in `docs/certification-record.md` — only then.
