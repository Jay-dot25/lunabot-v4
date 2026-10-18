# LunaBot Phase 1 — Acceptance and Engineering Report

**Assessment date:** 2026-10-18 (UTC)  
**Result:** **BLOCKED / partially implemented — NOT CERTIFIED**

This report distinguishes repository/offline checks from live simulation acceptance. A valid XML file, unit test, or build would not prove Gazebo sensor or rover behavior. In this environment the ROS 2/Gazebo runtime is absent, so none of the dynamic acceptance tests is claimed as passed.

## Environment inspected

| Item | Observation |
|---|---|
| OS | Debian GNU/Linux 12 (Bookworm), x86_64 |
| Kernel | Linux `6.1.158` (`e2b.local`) |
| Execution context | Hosted Linux sandbox; `WSL_DISTRO_NAME` is unset. No container marker was present, so container-versus-VM hosting cannot be determined confidently. |
| ROS 2 | `ROS_DISTRO` unset; `ros2` absent; `/opt/ros` absent |
| Gazebo | `gz`, `gazebo` absent; no version can be queried |
| Build/dependency tools | `colcon`, `rosdep` absent |
| GUI | `DISPLAY` and `WAYLAND_DISPLAY` unset; no rendering device/tool detected |
| Selected target | ROS 2 Jazzy + Gazebo Harmonic + `ros_gz_bridge`; appropriate installation/compatibility is documented as a target, not verified in this checkout |

No system-wide install or upgrade was attempted. This Debian sandbox is not a suitable place to substitute unverified packages for the declared ROS/Gazebo pairing.

## Implementation delivered

- A 30 m × 30 m static lunar test ground under approximately 1.62 m/s² gravity, directional light, dark background, and five repeatably placed rocks with visual and collision geometry.
- A compact, nonholonomic, two-driven-wheel rover with fixed passive support balls, collision/inertial properties, a camera, and a horizontal GPU LiDAR.
- One authoritative Gazebo DiffDrive system. `/cmd_vel` enters a ROS command gate with ±0.35 m/s and ±0.8 rad/s limits and a 0.30 s stale-command watchdog; it forwards only to the Gazebo DiffDrive transport topic. No direct model-pose control or ground-truth odometry path is implemented.
- Gazebo-side wheel odometry intended for `/odom` and `odom` → `base_link` TF. Sensor messages and TF are bridged via `ros_gz_bridge`; static base-to-sensor transforms are launched for `camera_link` and `lidar_link`.
- A live 640×480 RGB camera configured at 15 Hz and 720-ray 360° 2D LiDAR configured at 10 Hz. Those rates and frame identifiers are configuration values, not local observed measurements.
- Python obstacle monitor with configurable forward sector, range threshold, and stale timeout; it publishes `Bool` only for fresh valid clear/obstacle observations and uses a String status (`CLEAR`, `OBSTACLE`, `INVALID`, `STALE`) to distinguish unavailable data.
- Primary launch `lunabot_gazebo/launch/launch_1.launch.py`, wrapper `scripts/launch-1.sh`, legacy Launch A alias, interactive keyboard teleop, static/offline/build/runtime validation entrypoints, and documentation.

## Offline verification executed

Command:

```bash
./scripts/phase1_validate.sh static
```

**PASS:** XML parser accepted both SDF files and both package manifests; Python AST parsing passed for launch, ROS utility, setup, and test files; required file structure and resource-path scan passed. Six `unittest` cases passed, covering nearest forward reading, positive-infinity clear observations, off-sector rejection, invalid scans, empty scans, and velocity clamping/NaN handling.

This is syntax/logic validation only. XML well-formedness is not equivalent to validation by libsdformat, and it cannot test Gazebo plugin loading, ROS bridge type support, physics, image rendering, transforms, or movement.

## Build and runtime attempts

- `./scripts/phase1_validate.sh build` → **BLOCKED** before dependency installation: `ROS_DISTRO` is unset; `ros2`, `colcon`, and `rosdep` are unavailable.
- `./scripts/phase1_validate.sh runtime` → **BLOCKED**: ROS 2 is unavailable.
- `./scripts/launch-1.sh` → **BLOCKED**: `/opt/ros/jazzy/setup.bash` and ROS 2 are absent. Gazebo is absent too.
- No package build, Gazebo start, topic message, GUI, or runtime motion/sensor test was performed.

## Formal acceptance matrix

| Test | Status | Procedure / observed result | Remaining evidence needed |
|---|---|---|---|
| **A — Environment startup** | **BLOCKED** | Launch attempt exited with the documented ROS 2-not-installed blocker. No Gazebo world loaded. | Install supported ROS 2 Jazzy/Gazebo Harmonic environment; launch and observe stable operation. |
| **B — Rover stability** | **BLOCKED** | No simulator available; stability has not been observed. | Observe stationary rover/contact and check oscillation/drift in Gazebo. |
| **C — Forward movement** | **BLOCKED** | No `/cmd_vel` publisher/drive system could be run. | Live probe verifies physical and odometric forward motion. |
| **D — Reverse movement** | **BLOCKED** | Not run. | Live probe verifies reverse displacement. |
| **E — Rotation** | **BLOCKED** | Not run. | Live probe verifies positive and negative yaw changes. |
| **F — Stop behavior** | **BLOCKED** | Watchdog code and clamp unit logic were inspected/tested offline; no stale-command behavior was observed on a running drive system. | Live probe verifies command timeout and stationary odometry after stop. |
| **G — Live camera** | **BLOCKED** | No camera message or rendered image received. | Check actual frame, encoding, dimensions, content, topic frequency, and viewer output. |
| **H — Live range sensing** | **BLOCKED** | No Gazebo scan received. The pure algorithm unit tests are not a live sensor test. | Check open-yard and obstacle ranges while approaching/reorienting rover. |
| **I — Motion estimates** | **BLOCKED** | `/odom` and TF were not published in this environment. | Validate frame IDs, motion signs, and stationary stability from actual wheel odometry. |
| **J — Obstacle detection** | **BLOCKED** | Six algorithm-level tests passed, but actual `/scan`-driven obstacle assertion/clearing was not run. | Run live obstacle approach and retreat in the world; verify status transition and data staleness behavior. |
| **K — Coordinate frames** | **BLOCKED** | SDF/launch text defines intended names and sensor mount offsets; no TF tree/message was available to inspect. | Use `tf2_echo` / `view_frames` and confirm message frame IDs and a single odom/base publisher. |
| **L — Repeatability** | **BLOCKED** | No first launch/shutdown/relaunch was possible. | Complete two launch cycles and repeat basic drive/sensor checks. |

### Unit-level evidence (not a substitute for Test J)

| Check | Status | Result |
|---|---|---|
| Valid finite obstacle within front sector | **PASS** | Nearest range triggers `OBSTACLE`. |
| Positive infinite no-return readings | **PASS** | Valid observation resolves to `CLEAR`. |
| Obstacle outside sector | **PASS** | Does not trigger. |
| All invalid / empty sector | **PASS** | Resolves to `INVALID`, not clear. |
| Speed bounds and non-finite command handling | **PASS** | Commands clamp to configured limits; NaN/Inf become zero. |

## Certification checklist

- [x] Repository contains world/model, packages, launch, tools, tests, and developer instructions.
- [x] Offline Python/XML checks and six pure-logic tests pass.
- [x] Target stack and host incompatibility are explicitly documented; no destructive environment change made.
- [ ] ROS packages resolve and workspace builds on target.
- [ ] Gazebo loads and remains operational.
- [ ] Rover stability, forward/reverse/turn/stop, and repeat startup are demonstrated.
- [ ] Actual camera, LiDAR, odometry, clock, and TF streams are verified at runtime.
- [ ] Live sensor-based obstacle status transitions are demonstrated.

**Phase 1 certification: NOT PASSED.** The implementation is a handover-ready repository baseline, but it has not earned Lunar Base Camp certification because mandatory runtime evidence is blocked by the absent ROS 2/Gazebo environment. Do not describe the simulation as operational until the blocked tests pass on the target stack.

## Handover boundary

Once built and runtime-validated on ROS 2 Jazzy/Gazebo Harmonic, the planned Phase 2 input contracts are `/camera/image_raw`, `/scan`, `odom`/`base_link`/sensor frames, `/odom`, and the same deterministic simulated environment. This report does not implement or claim any Phase 2 perception, mapping, localization, planning, or avoidance behavior.
