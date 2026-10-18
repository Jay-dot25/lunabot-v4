# LunaBot — Phase 1: Lunar Base Camp

A small, reproducible ROS 2 + Gazebo simulation foundation. Phase 1 provides a physically driven rover in a compact lunar test yard, live camera and 2D LiDAR observations, wheel-based Gazebo odometry, bounded manual control, and a sensor-only forward obstacle status node. The project does **not** implement autonomy.

> **Certification status for this checkout:** implementation and offline/unit validation are present, but live acceptance is **BLOCKED** in the supplied environment. The host is Debian 12 with no ROS 2, Gazebo, `colcon`, or ROS integration packages installed. See [PHASE1_ACCEPTANCE.md](PHASE1_ACCEPTANCE.md) for per-test outcomes. No live simulation behavior is claimed as tested.

## Project overview

LunaBot is a future autonomous lunar-habitat rover. This phase intentionally establishes only its physical and communication baseline so later work can consume real images, scans, transforms, and odometry. It does not contain Phase 2 perception or any later autonomy feature.

## Architecture

```text
Keyboard / ROS publisher
       │ geometry_msgs/Twist  /cmd_vel
       ▼
command_gate (limit + 0.30 s watchdog)
       │ ROS→Gazebo Twist /lunabot/gz_cmd_vel
       ▼
Gazebo Harmonic DiffDrive ── wheel joints ── rover motion
       ├── wheel odometry ── /odom + odom→base_link TF
       ├── camera sensor ── /camera/image_raw
       └── GPU 2D LiDAR ─── /scan
                              │
                              ▼
                   obstacle_monitor
                   ├── /lunabot/obstacle_ahead (Bool)
                   └── /lunabot/obstacle_status (String)

Gazebo world (/clock) ── ros_gz_bridge ── ROS 2
base_link ── static TF ── camera_link, lidar_link
```

The sole drive mechanism is Gazebo's `gz::sim::systems::DiffDrive`. ROS commands do not set model poses. Odometry is the DiffDrive system's wheel-derived estimate, bridged from Gazebo, not Gazebo world ground truth. Gazebo Harmonic sensors provide the camera and scan. `ros_gz_bridge` carries their real messages to ROS 2. A small Python command gate bounds planar speeds and publishes a stop after stale input. A second Python node examines only the incoming LaserScan; it does not steer or avoid obstacles.

The initially selected target was **ROS 2 Jazzy + Gazebo Harmonic + `ros_gz_bridge`**. The developer's actual machine is now reported as **Ubuntu 22.04 + ROS 2 Humble**. There, `/usr/bin/gz` exists but `gz sim --versions` and the launch command `gz sim -r ...` returned `Invalid arguments`; the installed Gazebo implementation/version is not yet identified. Therefore the current Gazebo launch profile is **not runtime-compatible as observed**, and needs to be aligned with the installed simulator before acceptance. Do not install Jazzy setup commands on this Humble machine. The SDF uses Gazebo Sim `gz-sim-*` system filenames; it is not Gazebo Classic configuration.

## Requirements

### Environment evidence and current compatibility issue

- Agent checkout: Debian GNU/Linux 12, x86_64, kernel `6.1.158`, Python 3.11.2; no ROS/Gazebo tools or display.
- Developer-reported host: Ubuntu 22.04.5 with ROS 2 Humble and `/usr/bin/gz`. `gz sim --versions` and the attempted `gz sim -r ...` both returned `Invalid arguments`, so the installed Gazebo implementation/version is not yet identified.
- This repository currently uses Gazebo Sim `gz-sim-*` system filenames and a `gz sim` launch command. Runtime compatibility with the reported Humble host is therefore **not established**. Do not source Jazzy on that host or assume that having a `gz` executable means it supports `gz sim`.

Please identify the installed simulator before installing or replacing packages:

```bash
type -a gz ign
gz --help
gz sim --help
ign gazebo --help
dpkg-query -W -f='${Package} ${Version}\\n' 'gz-*' 'ignition-*' 'ros-humble-ros-gz*' 'ros-humble-ros-ign*' 2>/dev/null
ros2 pkg list | grep -E '^(ros_gz|ros_ign)'
```

Use installation instructions matching the detected ROS/Gazebo pairing. Do not upgrade the OS or install Jazzy just to match the initial target assumption.

## Installation and build

The developer-reported machine has ROS 2 Humble. In a clean terminal that does not source the old `~/lunabot-v4/install` overlay, build this checkout with:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4-phase1
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source install/setup.bash
./scripts/phase1_validate.sh static
```

The reported colcon build completed both packages, but `rosdep install` previously failed to resolve `ament_python`, and colcon warned that packages from the old checkout were already in an underlay. Run `rosdep update` and build in a clean Humble-only shell to verify these issues. No workspace-wide or developer-specific absolute paths are embedded in model resources.

## Launch

The primary one-command entry point is **Launch 1** (`scripts/launch-1.sh`). `scripts/launch-a.sh` remains as a compatibility alias for the repository's former Launch A name.

```bash
./scripts/launch-1.sh
```

The primary launch starts a single Gazebo world, one bridge, the command gate, obstacle monitor, and the two fixed sensor transforms. The world contains the rover and five deterministic collision-bearing rocks. The optional headless mode uses the same world/model:

```bash
./scripts/launch-1.sh headless:=true
# equivalent after sourcing and building:
ros2 launch lunabot_gazebo launch_1.launch.py headless:=true
```

Stop with **Ctrl+C** in the launch terminal. ROS launch terminates its child processes; relaunch the same command for a reset world. For a custom sensor threshold:

```bash
./scripts/launch-1.sh obstacle_threshold:=1.0 obstacle_forward_half_angle_deg:=12.0
```

Defaults: obstacle threshold 1.5 m, forward half-sector 15 degrees, stale scan timeout 0.75 s.

## Manual control

Open another terminal and source the ROS and workspace setups:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/lunabot-v4/install/setup.bash
ros2 run lunabot_tools keyboard_teleop
```

- `W` / `S`: bounded forward / reverse pulses
- `A` / `D`: rotate left / right
- Hold or repeat a direction key to renew commands; otherwise the utility stops renewing after 0.30 s
- `Space`: immediate stop
- `Q` or Ctrl+C: quit (publishes stop)

The command gate also accepts any standard `/cmd_vel` publisher. It clamps linear x to ±0.35 m/s and angular z to ±0.8 rad/s, and publishes a stop if input has been silent for 0.30 s. The Gazebo drive system further bounds motion and acceleration. No command directly changes the rover's world pose.

## Sensor and motion interfaces

| Topic | ROS type | Source / details |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Manual/operator input to command gate |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive wheel odometry; frames `odom` → `base_link` |
| `/tf` | `tf2_msgs/msg/TFMessage` | bridged Gazebo odom transform plus static sensor transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `base_link` → `camera_link`, `lidar_link` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 640×480 RGB camera at 15 Hz configured, `camera_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | 720-beam, 360° 2D LiDAR at 10 Hz configured, `lidar_link` |
| `/clock` | `rosgraph_msgs/msg/Clock` | simulation clock |
| `/lunabot/obstacle_ahead` | `std_msgs/msg/Bool` | published only for valid, fresh `CLEAR`/`OBSTACLE` scans |
| `/lunabot/obstacle_status` | `std_msgs/msg/String` | `CLEAR`, `OBSTACLE; nearest=… m`, `INVALID`, or `STALE` |

The configured camera and scan rates are targets, not measured claims. Check live rates on the installed target stack. For a quick inspection:

```bash
ros2 topic info -v /camera/image_raw
ros2 topic hz /camera/image_raw
ros2 topic info -v /scan
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /lunabot/obstacle_status
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

For an image-aware display, install the matching ROS `rqt_image_view` package, then run `ros2 run rqt_image_view rqt_image_view` and select `/camera/image_raw`. The static TFs should agree with the sensor poses in `model.sdf`; Gazebo's odometry plugin is the only intended publisher of `odom` → `base_link`.

## Obstacle monitoring

The monitor subscribes to the real `/scan` data and evaluates the angular sector ±15° around scan angle zero (forward is +X in `lidar_link`). It uses the nearest valid finite measurement in that sector and marks `OBSTACLE` at or below the configurable 1.5 m threshold. Positive infinity is treated as a valid no-return; NaN, negative infinity, zero/out-of-range, and empty sectors are invalid. If no valid scan has arrived or the last one is older than 0.75 s, status becomes `STALE`. Invalid/stale data are **not** reported as clear, and the Bool publisher emits no new sample in those states. Consumers should monitor the String status/scan freshness as well as the Bool topic. The monitor reports only; it never changes `/cmd_vel`.

## Validation and acceptance tests

Offline validation, with no ROS install required:

```bash
./scripts/phase1_validate.sh static
```

This checks XML/SDF well-formedness, Python syntax, required files, absence of developer-specific resource paths, and six unit tests for obstacle evaluation and command clamping. It does **not** certify Gazebo runtime behavior.

After successful dependency resolution and build:

```bash
./scripts/phase1_validate.sh build
```

With the launch still running in another terminal, run the live acceptance probe:

```bash
./scripts/phase1_validate.sh runtime
# or: ros2 run lunabot_tools runtime_probe
```

The live probe listens to actual camera, scan, odometry, obstacle and status messages; records message metadata and rates; performs low-speed bounded forward/reverse/left/right tests; drives cautiously toward the predefined rock to test detection; retreats to test clearing; and measures stationary odometry drift. It emits a JSON record and exits unsuccessfully if a check fails. Review `PHASE1_ACCEPTANCE.md` for the current test outcomes. Run the probe only in the provided bounded test yard, with a working simulation and a clear operator area.

## Troubleshooting

- **`ros2`/`gz` not found:** source the supported ROS setup and install the Jazzy + Harmonic integration packages; do not substitute Gazebo Classic commands.
- **Wrong/unsourced workspace:** source `/opt/ros/jazzy/setup.bash`, build, then source `install/setup.bash` in every terminal.
- **Bridge fails to load:** confirm `ros_gz_bridge` matches the installed ROS/Gazebo release and check `ros2 launch` output. Do not silently change the bridge message mapping.
- **Model/world not found:** use the package launch, not a hand-built path; confirm package was installed by `colcon build` and the model path is under `share/lunabot_gazebo/models`.
- **Topics exist but no data:** inspect Gazebo startup errors, bridge process health, simulation pause state, and `ros2 topic hz`; a topic name alone is not evidence of sensor output.
- **Camera messages exist but image is blank/unrendered:** ensure Gazebo's `Sensors` system loaded with Ogre2 and the environment has GPU/software-rendering support; inspect dimensions/encoding with the probe before assuming a viewer issue.
- **Wheels spin but rover barely moves:** inspect wheel-ground collision contact, gravity, friction and simulation warnings. Physics was not executable in this environment, so these settings need runtime tuning if this occurs.
- **TF conflict or missing sensor transforms:** there should be exactly one odom/base publisher from DiffDrive and one static publisher per sensor transform. Inspect `ros2 topic info -v /tf` and `tf2_echo`.
- **GUI unavailable in WSL/headless systems:** start `headless:=true` and inspect ROS messages; use a configured display server for Gazebo GUI or an image viewer. This host has no display variables, so GUI behavior is unverified.
- **Build `rosdep` failure:** verify ROS distribution setup and package repositories, read the named dependency failure, and install only the matching release package. No dependency errors have been observed here because `rosdep` is absent.

## Current limitations

This is a small flat lunar test yard (30 m ground plane) with simple rocks and a compact differential rover. It is not a high-fidelity terrain, full habitat, or validated lunar mobility model. There is no deep-learning terrain segmentation, dataset pipeline, SLAM, semantic map, global/local path planning, autonomous obstacle avoidance, mission execution, full settlement, dashboard, natural-language control, or return-to-base behavior. Those capabilities remain out of scope by design.
