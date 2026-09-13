# Phase C — SLAM & Localization (`launch-c`)

## 1. Scope and gate

Phase C adds one mapping/localization system, `slam_toolbox`, to the validated
LunaBot V4 Phase A/Phase B runtime. It does not replace the lunar terrain,
rover model, Gazebo Fortress sensor interfaces, controller, wheel odometry,
IMU bridge, or sensor TF. Phase C is an independent launch: it starts Gazebo,
the rover, bridge, static sensor TF, control node, odometry monitor,
`slam_toolbox`, and RViz directly. It never invokes `launch-a` or `launch-b`.

This repository contains the implementation and static checks, but the Phase C
runtime gate is workstation-only. Do not call Phase C complete or begin Phase D
until a real Ubuntu ROS 2/Gazebo run confirms `/map`, `map -> odom`, motion
while the map updates, the IMU, RViz topics, and clean relaunch.

## 2. Data flow

```text
/lunabot/lidar/scan ───────────────┐
                                   │
odom -> chassis TF + scan          ▼
/lunabot/odom motion source -> slam_toolbox
                                   │
                    /map + map -> odom TF

/cmd_vel_in -> controller -> /cmd_vel -> Gazebo DiffDrive -> /lunabot/odom
```

`slam_toolbox` is configured with simulated time, `map` as its map frame,
`odom` as its odometry frame, `chassis` as its base frame, and the explicit
scan topic `/lunabot/lidar/scan`. `/lunabot/odom` remains the motion source;
Phase C does not add a second odometry or localization stack.

## 3. Ubuntu 22.04 installation

The following commands assume Ubuntu 22.04 with ROS 2 Humble and the Phase A
Gazebo Fortress packages already installed. Install the Phase C packages in a
terminal:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2 \
  ros-humble-tf2-tools
```

Keep the bridge package used by the validated Phase B setup. If it is not
already installed, install the Fortress bridge package:

```bash
sudo apt install -y ros-humble-ros-ign-bridge
```

If this machine uses the newer `ros_gz_bridge` package instead, the launcher
detects it and uses it automatically. Source ROS in every terminal that will
inspect the run:

```bash
source /opt/ros/humble/setup.bash
```

From the repository checkout, install the canonical home command. This is a
symlink, so later edits and the fixed repository path remain in use:

```bash
cd ~/lunabot-v4       # adjust to the checkout location
ln -sfn "$PWD/launch-c" ~/launch-c
chmod +x launch-c scripts/launch-c.sh
```

The launcher resolves its own path safely, so it does not depend on the
current working directory and does not call another phase launcher.

## 4. Static validation before running ROS

Run the Phase C static gate from the checkout:

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_c.py
```

This checks file presence, executable permissions, shell/Python syntax, the
single `slam_toolbox` contract, explicit bridge and frame names, headless
unpause, real `ros2 topic echo --once` validation, watchdog/control carryover,
IMU checks, RViz topic/QoS settings, map evidence handling, and independent
launch behavior. Static output is not runtime acceptance.

## 5. Automated headless runtime/evidence run

Run the complete Phase C gate on a machine with a graphical session not
required:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c
```

The launcher explicitly unpauses `/world/lunar_world/control`; server-only
Gazebo otherwise starts paused. It waits for real messages rather than using a
pipe that can exit before a message arrives. The startup validation must show
PASS for at least:

```text
/map                         nav_msgs/OccupancyGrid
map -> odom                  slam_toolbox TF
/lunabot/odom               nav_msgs/Odometry
odom -> chassis              DiffDrive TF
/lunabot/lidar/scan         LaserScan
/lunabot/camera/image_raw   Image
/lunabot/imu                Imu
```

The demo drives through `/cmd_vel_in`, the hardened controller, `/cmd_vel`,
and the existing Gazebo DiffDrive. With `EVIDENCE=1`, the launcher captures a
real `/map` sample before and after the drive and requires distinct samples as
a motion-map-update gate. It also records the final map with
`nav2_map_server` when that package is installed.

A successful runtime must end with:

```text
live map updates during rover motion: PASS
PHASE C RUN COMPLETE - overall result: PASS
```

Do not manufacture or edit runtime evidence. If a check fails, inspect the
logs in `evidence/phase-c-launch-c/`, fix the runtime issue, and repeat the
run.

## 6. GUI/RViz visual run

After the headless run passes, launch the visual configuration:

```bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-c
```

The Phase C RViz layout uses fixed frame `odom` and has explicit displays for:

- `SLAM Map` on `/map`;
- `LaserScan` on `/lunabot/lidar/scan` with **Best Effort** reliability;
- `Camera` on `/lunabot/camera/image_raw` with **Best Effort** reliability;
- `TF` with `map -> odom -> chassis` and sensor frames;
- `Odometry` on `/lunabot/odom`.

Press `W`, `A`, `S`, `D`, and `Space` in the teleop window. Confirm the rover
moves in Gazebo, the LaserScan and camera remain error-free, RViz shows the map
being built, and the TF tree contains both `map -> odom` and
`odom -> chassis`. Press `Q` or Ctrl+C to exercise clean shutdown.

## 7. Evidence files

The launcher writes runtime evidence under
`evidence/phase-c-launch-c/`:

| File | Meaning |
|---|---|
| `last_run.log` | ordered launcher and validation log |
| `topics.txt` | topic graph observed during the run |
| `slam.log` | `slam_toolbox` output |
| `map_sample.txt` | real `/map` OccupancyGrid sample |
| `map_before_motion.txt` / `map_after_motion.txt` | motion update gate samples |
| `tf_map_odom.txt` | real `map -> odom` lookup |
| `tf_odom_chassis.txt` | real `odom -> chassis` lookup |
| `odom_sample.txt` / `imu_sample.txt` | real motion and IMU messages |
| `odometry_samples.csv` / `odometry_report.txt` | Phase B odometry evidence |
| `demo_drive_result.txt` / `diag_drive.csv` | real control-chain demo evidence |
| `phase_c_map.yaml` / `phase_c_map.pgm` | saved map, when map saver succeeds |
| `map_saver.log` | map-saver result or explicit optional-package note |

The map saver is optional at startup so mapping can still be diagnosed on a
minimal installation. Install `ros-humble-nav2-map-server` before a final
evidence run if `phase_c_map.yaml` and `.pgm` are required.

## 8. Clean relaunch test

After Q or Ctrl+C, verify that the launcher closed its process group and that
no old lunar Gazebo or Phase C nodes remain. Then run the automated command a
second time:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c
```

Both runs must independently unpause Gazebo, spawn one rover, start one
`slam_toolbox`, validate the live map/TF/sensors, complete the drive, and shut
down without orphaned processes. A clean relaunch is part of the Phase C gate.

## 9. Acceptance checklist

- [ ] `/map` publishes a real `nav_msgs/OccupancyGrid`.
- [ ] `slam_toolbox` publishes `map -> odom`.
- [ ] DiffDrive continues to publish `odom -> chassis`.
- [ ] `chassis -> sensor_head -> lidar` remains available.
- [ ] `/lunabot/lidar/scan` feeds the one SLAM system.
- [ ] `/lunabot/odom` remains the motion source.
- [ ] The rover drives while the map produces distinct before/after samples.
- [ ] Camera and LaserScan RViz displays use explicit topics and stay error-free.
- [ ] `/lunabot/imu` is bridged and receives real `sensor_msgs/Imu` data.
- [ ] Headless launch unpauses Gazebo and uses real message waits.
- [ ] Map files are saved when `nav2_map_server` is installed and succeeds.
- [ ] Ctrl+C/Q shutdown and a second launch both work cleanly.

## 10. Repository files

| File | Role |
|---|---|
| `launch-c` | canonical `~/launch-c` entry point |
| `scripts/launch-c.sh` | independent Phase C startup, validation, evidence and shutdown |
| `config/slam_toolbox_phase_c.yaml` | only SLAM configuration |
| `rviz/phase_c.rviz` | map, TF, odometry, camera and LiDAR visualization |
| `tools/validate_phase_c.py` | static Phase C gate |
| `evidence/phase-c-launch-c/` | runtime evidence directory and checklist |

============================================================
PHASE C IMPLEMENTATION — PENDING RUNTIME VALIDATION
============================================================

Runtime approval belongs to the Ubuntu ROS 2/Gazebo workstation run described
above. Stop at this gate and obtain explicit approval before Phase D.
