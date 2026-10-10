# LunaBot — Phase 1: Lunar Base Camp (clean restart)

This directory is a **stand-alone ROS 2 workspace** for the Phase 1 simulation foundation. It reuses the rover/world ideas in the repository as reference, but does not depend on the old multi-phase launch files, ML code, planners, maps, or ROS packages. The previous project files outside `lunabot_phase1/` are retained and left untouched as reference material.

The scope is manual operation only: a six-wheel rover moves in Gazebo, publishes a live RGB camera, a simulated 2D LiDAR scan, wheel-model odometry and TF, and a diagnostics-only obstacle monitor. The side rails are rocker-bogie-inspired visuals, while the six wheels are driven in left/right groups; this is not a fully articulated suspension. There is no SLAM, semantic perception, route planning, autonomous obstacle avoidance, or mission dashboard.

> **Verification status:** This repository's current host is Debian 12 with Python 3.11.2. ROS 2, Gazebo, `colcon`, and the ROS-Gazebo bridge are not installed here. Static tests can run on this host; Gazebo/ROS runtime acceptance remains **NOT VERIFIED** until the project is built and driven on the supported stack below. See [`PHASE1_ACCEPTANCE.md`](PHASE1_ACCEPTANCE.md).

## 1. Software stack and prerequisites

The new workspace targets the compatible binary stack **Ubuntu 22.04 (Jammy) + ROS 2 Humble + Gazebo Fortress (`ign gazebo`, Gazebo Sim 6) + `ros_ign_bridge`**. Fortress uses `ignition-gazebo-*` system-plugin libraries and `ignition.msgs.*` transport types. On Humble, `ros_ign_bridge` is the compatibility shim for `ros_gz_bridge`; this workspace intentionally targets that Fortress-compatible API.

Required packages on the target workstation:

- ROS 2 Humble desktop or base installation.
- Gazebo Fortress (`ign gazebo`) and `ros-humble-ros-ign-bridge` (the Humble bridge shim; it brings in `ros_gz_bridge`). The launch starts `ign gazebo` directly and does not require `ros_ign_gazebo`.
- `tf2_ros` and `tf2_tools`.
- `rqt_image_view` for convenient camera viewing (optional; `ros2 topic echo` is sufficient to inspect messages).
- `colcon` and `rosdep` build tools.
- A Gazebo-capable display/rendering setup for live camera and GPU LiDAR sensors.

On an Ubuntu 22.04 ROS 2 Humble installation, install the ROS-side tools with:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-ign-bridge ros-humble-tf2-ros \
  ros-humble-tf2-tools ros-humble-rqt-image-view \
  python3-colcon-common-extensions python3-rosdep
```

These are **recommended ROS-side install commands**, not claims that the agent host has these packages. The target workstation supplied by the user already reports Ubuntu 22.04.5, ROS 2 Humble, and Gazebo Sim 6.18.0 via `ign gazebo`. If `rosdep` has never been initialized, follow the ROS 2 Humble instructions. Do not install another Gazebo generation alongside the Fortress binaries.

## 2. Project structure

```text
lunabot_phase1/
├── README.md
├── PHASE1_ACCEPTANCE.md
├── scripts/
│   └── phase1_smoke_test.sh
├── tests/
│   ├── test_core.py
│   └── test_project_files.py
└── src/
    ├── lunabot_phase1_description/
    │   └── models/lunabot_rover/   # SDF rover and model.config
    ├── lunabot_phase1_simulation/
    │   └── worlds/lunar_base_camp.sdf
    ├── lunabot_phase1_tools/
    │   └── lunabot_phase1_tools/  # command guard, teleop, obstacle monitor
    └── lunabot_phase1_bringup/
        ├── launch/                # integrated, monitor-only and teleop launches
        └── config/                # bridge and node parameters
```

`lunabot_phase1_description` owns the physical rover model and `model://lunabot_rover` asset. `lunabot_phase1_simulation` owns the lunar world and its colliding obstacles. `lunabot_phase1_tools` contains the ROS nodes. `lunabot_phase1_bringup` joins those packages and installs the launch/configuration files. Separate camera and LiDAR bridge processes are used because Humble applies `override_frame_id` at the bridge-node level.

The world includes the rover model directly at a deterministic spawn pose. There is no separate asynchronous spawn step, so the rover and world are loaded together. The level central lane remains flat for baseline drive/sensor tests; four shallow physical regolith domes sit outside that lane. The rover follows the reference silhouette: three wheels per side, fixed rocker-bogie-style side rails, a camera mast, LiDAR, blue solar panel, and front bumper. It is about 1.2 × 0.8 × 0.6 m overall with an approximately 80 kg simulated mass. These are simple Phase 1 primitives, not a high-resolution terrain DEM, articulated suspension, or flight-hardware model. The installed model directory is added to Fortress’s `IGN_GAZEBO_RESOURCE_PATH` by the launch file.

## 3. Build and environment setup

This restart workspace targets **ROS 2 Humble + Gazebo Fortress**. Use a clean Humble shell, not a Jazzy shell, and avoid sourcing unrelated/stale workspace overlays while building. The older project files outside `lunabot_phase1/` remain reference-only.

First locate the clean restart workspace on your machine. This avoids guessing the repository directory (and avoids running `colcon` from `$HOME`):

```bash
mapfile -t PHASE1_MANIFESTS < <(find "$HOME" -maxdepth 8 -type f -path '*/lunabot_phase1/src/lunabot_phase1_bringup/package.xml' -print 2>/dev/null)
printf '%s\n' "${PHASE1_MANIFESTS[@]}"
```

There should be exactly one result. If there is none, the local checkout does not contain the `lunabot_phase1/` restart directory; sync/download the updated project before building. If there are several, choose the intended checkout and set its directory manually. Do **not** type an example placeholder such as `/path/to/...` literally.

With the unique manifest path found, use a clean terminal and build from the workspace root:

```bash
mapfile -t PHASE1_MANIFESTS < <(find "$HOME" -maxdepth 8 -type f -path '*/lunabot_phase1/src/lunabot_phase1_bringup/package.xml' -print 2>/dev/null)
if (( ${#PHASE1_MANIFESTS[@]} != 1 )); then
  printf 'Expected exactly one Phase 1 workspace; found %s. Sync/select the intended checkout first.\n' "${#PHASE1_MANIFESTS[@]}" >&2
  printf '%s\n' "${PHASE1_MANIFESTS[@]}" >&2
  exit 2
fi
PROJECT_DIR="${PHASE1_MANIFESTS[0]%/src/lunabot_phase1_bringup/package.xml}"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
test "${ROS_DISTRO:-}" = humble || { echo "Expected ROS_DISTRO=humble; got ${ROS_DISTRO:-unset}" >&2; exit 2; }
test -f src/lunabot_phase1_bringup/package.xml || { echo "Wrong PROJECT_DIR: Phase 1 packages not found" >&2; exit 2; }
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

For a fresh terminal, repeat the workspace locator block to set `PROJECT_DIR`, then source `/opt/ros/humble/setup.bash` and `install/setup.bash`.

Run the ROS-independent static/unit suite from the project directory:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q src tests
bash -n scripts/phase1_smoke_test.sh
```

The static suite validates XML well-formedness, package/launch/topic contracts, the rover/world component inventory, command-watchdog policy, and obstacle-sector filtering. It does **not** validate Gazebo's SDF schema, physics, rendering, bridge runtime, TF connectivity, or actual rover motion.

## 4. Launch

Start the world, included rover, bridge, command guard, static transforms, and obstacle monitor:

```bash
ros2 launch lunabot_phase1_bringup phase1.launch.py
```

Gazebo Fortress is started with `ign gazebo -r` and its GUI. The world uses lunar gravity (`-1.62 m/s²`), a level central test lane, four shallow physical regolith domes outside the lane, directional lighting, and static obstacles with collision geometry. Press `Ctrl+C` in the launch terminal to shut down the complete launch.

The monitor can also be started by itself if another simulator already publishes `/scan`:

```bash
ros2 launch lunabot_phase1_bringup monitor.launch.py
```

Do not launch a second monitor alongside `phase1.launch.py` unless you intentionally want duplicate diagnostic publishers. Optional keyboard launch:

```bash
ros2 launch lunabot_phase1_bringup teleop.launch.py
```

For reliable keyboard input, use the `ros2 run` command from its own interactive terminal as described below.

## 5. Manual rover control

Open a **second terminal** and change into the `lunabot_phase1/` workspace root (the directory containing `install/setup.bash`). In that terminal, source both environments and confirm the package resolves from this workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix lunabot_phase1_tools
```

The prefix should point under this workspace's `install/` directory. Then, in the same terminal, start teleop:

```bash
ros2 run lunabot_phase1_tools keyboard_teleop
```

Controls (the command remains active until another key is pressed):

| Key | Action |
|---|---|
| `W` | Forward |
| `S` | Reverse |
| `A` | Rotate left (positive angular Z) |
| `D` | Rotate right (negative angular Z) |
| `Space` or `X` | Publish an explicit stop |
| `Q` | Publish a stop and exit |
| `Ctrl+C` | Exit; the node attempts a stop burst and the watchdog stops stale input |

While `keyboard_teleop` is running, it switches the terminal to raw keyboard input: pressed keys do **not** echo, and the shell prompt does not return until you press `Q` or `Ctrl+C`. This is expected. Press one control key at a time without Enter; do not type shell commands into that terminal until teleop exits. Use a separate terminal for ROS inspection. Because movement commands persist until changed, press `X` or `Space` after each short movement.

The teleop node publishes at 20 Hz. The separate `command_guard` clamps commands and publishes zero to Gazebo when its `/cmd_vel` input has been stale for 0.50 seconds. That timeout is a simulation command watchdog, not a certified emergency stop; it depends on the guard and bridge process remaining alive. Always stop manually before approaching a collision object.

A direct command can also be sent from a terminal:

```bash
# Drive at 0.20 m/s until Ctrl+C stops this publisher.
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"

# Send an explicit zero command.
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 6. Sensor, odometry, TF and status inspection

Run the interactive camera viewer and select `/camera/image_raw` from the topic list:

```bash
ros2 run rqt_image_view rqt_image_view
```

Other inspection commands:

```bash
ros2 topic list -t
ros2 topic echo /camera/camera_info --qos-reliability best_effort
ros2 topic echo /scan --qos-reliability best_effort
ros2 topic echo /odom
ros2 topic echo /obstacle_monitor/status
ros2 topic echo /obstacle_monitor/closest_range
ros2 run tf2_ros tf2_echo odom lidar_link
ros2 run tf2_ros tf2_echo odom camera_optical_frame
ros2 run tf2_tools view_frames
```

The LiDAR scan is 360 degrees with 720 samples, 0.12–15 m nominal range, 10 Hz update, and a small Gaussian range-noise setting. The monitor evaluates only the forward cone (`±0.5236 rad`, approximately ±30 degrees). At least one valid forward return farther than 1.50 m produces `CLEAR`; a valid return at or below 1.50 m produces `OBSTACLE_DETECTED`; invalid/absent scans or no usable forward returns produce `NO_VALID_MEASUREMENTS` and a NaN closest-range diagnostic. The scan watchdog reports an absent stream after 1.00 s. No monitor state publishes a velocity command or changes rover motion.

Run the interface smoke check after launch:

```bash
./scripts/phase1_smoke_test.sh
```

It checks node presence, ROS message types, and live camera/scan/odometry/TF/status messages. It intentionally does not claim to prove drive motion or obstacle-dependent sensor response; follow the manual acceptance procedures below and record the observations.

## 7. ROS interfaces and coordinate frames

| Function | ROS topic | Type | Publisher / bridge direction | Required |
|---|---|---|---|---|
| Human velocity input | `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop/operator → `command_guard` | Yes for manual drive |
| Bounded simulation velocity | `/cmd_vel_sim` | `geometry_msgs/msg/Twist` | `command_guard` → `ros_ign_bridge` → Gazebo `/cmd_vel_sim` | Yes for safe drive path |
| Wheel-model odometry | `/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive → ROS bridge | Yes |
| RGB image | `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo camera → ROS bridge | Yes |
| Camera calibration | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo camera → ROS bridge | Yes |
| 2D range scan | `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo GPU LiDAR → ROS bridge | Yes |
| Dynamic transform | `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo DiffDrive → ROS bridge | Yes |
| Rigid transforms | `/tf_static` | `tf2_msgs/msg/TFMessage` | Four `tf2_ros/static_transform_publisher` processes | Yes |
| Obstacle state | `/obstacle_monitor/status` | `std_msgs/msg/String` | `obstacle_monitor` | Yes for monitor test |
| Nearest forward return | `/obstacle_monitor/closest_range` | `std_msgs/msg/Float32` | `obstacle_monitor`; NaN if no usable return | Yes for monitor test |
| Simulation time | `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS bridge | Yes when `use_sim_time` is enabled |

The frame tree is:

```text
odom
└── base_footprint        dynamic, only from Gazebo DiffDrive
    └── base_link          fixed chassis-centre offset (z = 0.28 m)
        ├── camera_link    fixed sensor mount
        │   └── camera_optical_frame  fixed REP-103 optical rotation
        └── lidar_link     fixed sensor mount
```

The SDF camera looks along local `+X`; the fixed camera-to-optical transform and bridge frame override expose ROS optical convention. The LiDAR uses `lidar_link`. Gazebo DiffDrive is the only dynamic odom/TF publisher. The static publishers own only the listed rigid transforms; there is no map frame, ground-truth pose feed, `robot_localization`, or second odometry source in this Phase 1 restart.

## 8. Configuration

- `src/lunabot_phase1_description/models/lunabot_rover/model.sdf`: chassis dimensions `1.00 × 0.62 × 0.20 m`, nominal body mass `64.8 kg` (about `80 kg` total simulated mass), six `0.15 m` radius driven wheels on a `0.68 m` track, rocker-bogie-style side-rail visuals, camera mast, solar panel and front bumper, camera and LiDAR settings, and Gazebo DiffDrive velocity/acceleration limits.
- `src/lunabot_phase1_bringup/config/command_guard.yaml`: linear limit `0.35 m/s`, angular limit `0.80 rad/s`, 30 Hz guard output, and 0.50 s input watchdog. Keep its speed limits aligned with the DiffDrive SDF values.
- `src/lunabot_phase1_bringup/config/obstacle_monitor.yaml`: 1.50 m initial warning threshold, forward half-angle, and 1.00 s no-scan timeout. Tune the threshold to rover geometry and the intended operator reaction distance; it is not a universal safety distance.
- `src/lunabot_phase1_bringup/config/bridge.yaml`, `camera_bridge.yaml`, and `lidar_bridge.yaml`: Humble/Fortress topic/type mappings; camera and LiDAR frame overrides are set per bridge node in `phase1.launch.py`.
- `src/lunabot_phase1_simulation/worlds/lunar_base_camp.sdf`: lunar gravity, ground appearance, physical obstacle poses/geometries, and the rover's initial world pose.

## 9. Acceptance tests

The agent host cannot execute ROS or Gazebo runtime tests. On 2026-10-10, the user pulled commit `cfb75da`, built all four Humble packages, and launched Fortress 6.18.0 with the six-wheel model. The GUI screenshot shows the world and rover; logs show DiffDrive/bridges started and a live forward scan return at 0.34 m. The first launch shut down cleanly. This confirms build and load/spawn, not six-wheel visibility or dynamic stability; the near-front view overlaps the wheel axles. The user first reported that `W` appeared to move backward, then suspected the rover was already reversing. Teleop maps `W` to positive `linear.x`; compare Gazebo world pose and wheel-model `/odom` separately after a short controlled pulse before changing joint-axis signs. Earlier live smoke, CameraInfo, and `/odom` observations are documented in [`PHASE1_ACCEPTANCE.md`](PHASE1_ACCEPTANCE.md). Continue:

1. Build with the commands in §3; expected: all four packages build.
2. Launch with the command in §4; expected: world and rover appear and remain stable.
3. Run the smoke script; expected: all listed ROS messages arrive with the listed types.
4. Drive forward, reverse, rotate left/right, and stop using the keyboard controls; compare `/odom` before/after each motion.
5. In one terminal, run `ros2 topic echo /odom`; in another, run `ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}}"`. Press `Ctrl+C` in the publisher terminal without sending zero. The guard should publish zero after 0.50 s, and odometry should settle after modeled deceleration (allow roughly 1–1.5 s). Then issue an explicit zero command.
6. Verify the clear initial forward sector in `/obstacle_monitor/status`; manually approach the physical `forward_blocker` until status changes to `OBSTACLE_DETECTED`, and confirm `/obstacle_monitor/closest_range` agrees with the forward portion of `/scan`. Stop before collision and reverse; confirm range increases.
7. Rotate the rover and verify the camera viewpoint and LiDAR scan change; inspect `odom -> camera_optical_frame` and `odom -> lidar_link` with `tf2_echo`.
8. Stop and relaunch, then repeat the interface and movement checks without repairing generated files.

This check sequence does **not** test or imply autonomous stopping or obstacle avoidance. The monitor is observational only. Do not mark runtime criteria `PASS` from static inspection or compilation alone.

## 10. Troubleshooting

- **Gazebo will not start:** confirm `ROS_DISTRO=humble`, source `/opt/ros/humble/setup.bash`, and install `ros-humble-ros-ign-bridge`. Check `ign gazebo --version` (not `gz sim --version`) and launch output. This workspace uses Fortress plugins and SDF 1.8.
- **Rover model is missing:** verify the `lunabot_phase1_description` package built and sourced. The launch adds its installed `models/` directory to `IGN_GAZEBO_RESOURCE_PATH`; inspect `echo "$IGN_GAZEBO_RESOURCE_PATH"` and the world include `model://lunabot_rover`.
- **ROS topics do not appear or the smoke test misses a node:** confirm the launch is still running and both terminals use the same `ROS_DOMAIN_ID`/RMW environment. Restart the ROS CLI graph daemon with `ros2 daemon stop`, then check `ros2 node list` and `ros2 topic list -t`. The smoke script waits up to 20 seconds for each expected node. Also inspect the Gazebo log, `/clock` bridge, and installed bridge YAML.
- **Camera image is absent or blank:** confirm the world Sensors system loaded with `ogre2`, that a rendering-capable display/GPU is available, and that Gazebo Transport advertises `/camera/image_raw`. The camera and LiDAR bridges are launched separately with `override_frame_id` set to `camera_optical_frame` and `lidar_link`, respectively. Use `rqt_image_view` or a best-effort subscriber when inspecting sensor topics.
- **LiDAR returns are invalid:** check the GPU sensor's `ignition-gazebo-sensors-system`, `ogre2` rendering, `/scan` topic type, scan `range_min`/`range_max`, and that physical collision objects intersect the horizontal scan plane. `NO_VALID_MEASUREMENTS` is intentionally distinct from `CLEAR`.
- **Odometry or TF is missing:** inspect `/odom`, `/tf`, and `/tf_static`; check that one DiffDrive plugin loaded and the Humble bridge maps `ignition.msgs.Odometry` and `ignition.msgs.Pose_V`. Use `tf2_echo odom base_footprint`. Do not add a second publisher for `odom -> base_footprint`.
- **Rover does not move:** check that `command_guard` receives `/cmd_vel`, publishes `/cmd_vel_sim`, and that the bridge maps the identical `/cmd_vel_sim` name to the Fortress DiffDrive plugin. Check the wheel joint names, wheel radius/separation, and speed limits in the SDF. The input command expires after 0.50 s, so test with a continuing publisher or the included teleop.
- **Launch cannot find resources:** rebuild, source `install/setup.bash`, and check `ros2 pkg prefix lunabot_phase1_simulation`, `ros2 pkg prefix lunabot_phase1_description`, and `ros2 pkg prefix lunabot_phase1_bringup`.
- **Static TF seems absent:** inspect `ros2 topic info /tf_static --verbose`, then try `ros2 topic echo /tf_static --qos-reliability reliable --qos-durability transient_local`. Confirm the four static-transform nodes are alive and verify frame connectivity with `tf2_echo`/`view_frames`; static transforms are transient-local, unlike dynamic `/tf`.
- **Shutdown prints `publisher's context is invalid`:** update and rebuild the workspace; the command guard and teleop now defer ROS context shutdown until after their best-effort stop burst.

## 11. Known limitations and scope boundary

- The agent host has no ROS 2, Gazebo, bridge, or `colcon`. The user built and launched the current six-wheel revision on Humble/Fortress; a screenshot confirms the GUI, world, and rover are visible at rest. The user reports that `W` appears to move backward; verify Gazebo world pose separately from wheel-model odometry before changing joint signs. Direction-specific motion, all-six-wheel visibility from a side view, dynamic/uneven-terrain stability, current image/CameraInfo matching, full TF connectivity, watchdog timing, the remaining monitor case, and controlled obstacle-in/out range behavior remain unverified.
- The code offers static XML/topic checks and pure-Python unit tests only; these are not a substitute for the ROS/Gazebo acceptance gate.
- The command watchdog protects against a stale upstream velocity publisher while `command_guard` is alive. It is not an independent hardware stop and cannot guarantee a safe stop if the guard, bridge, or simulator itself terminates unexpectedly.
- Lunar gravity, rigid obstacles and wheel friction are simplified simulation assumptions. The rocker-bogie-style side rails are visual/fixed, not an articulated suspension; the terrain is a low-relief primitive test surface, not deformable regolith or a detailed heightmap. The current model has no SLAM, localization fusion, mapping, deep-learning perception, path planning, autonomy or mission-control UI.
- GPU camera/LiDAR rendering may need graphics-driver or headless-rendering configuration on a workstation/CI host.

No Phase 2 work is included. Phase 2 is not authorized until every mandatory P1 criterion is actually verified and marked `PASS`.
