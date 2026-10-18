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

The selected target is **ROS 2 Jazzy + Gazebo Harmonic + `ros_gz_bridge`**. This is the intended modern Gazebo pairing for Ubuntu 24.04. It is **not verified by execution on this host**. The SDF declares format 1.9 and uses the Gazebo Sim `gz-sim-*` systems (not Gazebo Classic or the old Ignition launch commands).

## Requirements

### Target system

- Ubuntu 24.04 (recommended for packaged ROS 2 Jazzy)
- ROS 2 Jazzy, sourced in the current shell
- Gazebo Harmonic (`gz sim`) and its rendering/sensors systems
- `ros_gz_bridge`, `tf2_ros`, `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `tf2_msgs`, `rosgraph_msgs`
- `colcon`, `rosdep`, Python 3, and `python3-pytest` or Python `unittest`
- Optional GUI: a working desktop/display; optional image view: `rqt_image_view`

**Verified here:** Debian GNU/Linux 12 (Bookworm), x86_64; kernel `6.1.158`; Python 3.11.2. This is a hosted Linux execution environment (not detected as WSL; the underlying container/VM boundary cannot be confirmed from the available evidence). `DISPLAY` and `WAYLAND_DISPLAY` are unset. `ros2`, `gz`, `gazebo`, `colcon`, `rosdep`, and `xacro` are absent. ROS/Gazebo version compatibility and GUI operation therefore remain unverified locally.

### Safe installation guidance

Do not upgrade this Debian environment or replace its system packages to make the simulation run. For an operational setup, use a supported Ubuntu 24.04 ROS 2 Jazzy environment and follow the official ROS 2 Jazzy installation instructions and Gazebo Harmonic/ROS integration package instructions for that release. Install the matching packages (`ros-jazzy-ros-gz-bridge`, `ros-jazzy-tf2-ros`, `python3-colcon-common-extensions`, `python3-rosdep`; plus the Jazzy desktop or base packages appropriate to your machine). Resolve remaining declared dependencies with `rosdep` below. Exact package availability can vary by the configured ROS/Gazebo repositories; the target stack was not installed here to verify package resolution.

## Installation and build

From the repository root, after installing and sourcing the selected ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/lunabot-v4
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source install/setup.bash
```

No workspace-wide or developer-specific absolute paths are embedded in model resources. The launch file sets `GZ_SIM_RESOURCE_PATH` from the installed package share directory.

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
