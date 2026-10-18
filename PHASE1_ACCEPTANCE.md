# LunaBot Phase 1 — Acceptance and Engineering Report

**Assessment date:** 2026-10-18 (UTC)  
**Result:** **BLOCKED / partially implemented — NOT CERTIFIED**

This report distinguishes repository/offline checks from live simulation acceptance. A valid XML file, unit test, or build would not prove Gazebo sensor or rover behavior. In this environment the ROS 2/Gazebo runtime is absent, so none of the dynamic acceptance tests is claimed as passed.

## Environment inspected

| Item | Observation |
|---|---|
Two environments have now been observed and must not be conflated:

| Item | Agent checkout | Developer-reported machine (from terminal transcript) |
|---|---|---|
| OS | Debian 12, x86_64, kernel `6.1.158` | Ubuntu 22.04.5 LTS (Jammy) |
| ROS 2 | Absent | ROS 2 Humble (`/opt/ros/humble`) |
| Gazebo | Absent | Ignition Fortress `1.0.3-2~jammy`; `ign gazebo --help` works. The separate `gz` dispatcher rejects `gz sim` with `Invalid arguments`. |
| Build tools | `colcon`, `rosdep` absent | Build ran; `rosdep install` reported no definition for `ament_python` |
| GUI | No display variables in the agent sandbox | Not yet reported |
| Compatibility | Agent sandbox has no ROS/Gazebo runtime | Developer host is Humble/Jammy + Ignition Fortress 1.0.3; launch/SDF switched to `ign gazebo` and Fortress system plugins, but updated configuration still needs a runtime test |

No system-wide install or upgrade was attempted in the agent environment. The project is now aligned to the reported Fortress installation (`ign gazebo` and `ignition-gazebo-*` systems); the corrected runtime has not yet been launched.

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

- On the agent checkout, `./scripts/phase1_validate.sh static` → **PASS**, with all six unit tests passing.
- On the developer-reported Ubuntu 22.04/Humble checkout, `rosdep update` completed, but `rosdep install` still **failed** with `Cannot locate rosdep definition for [ament_python]`. A prior direct `colcon build --symlink-install` completed both packages successfully, with warnings that the old checkout and new install were both in the underlay. The build validator now skips only this unmapped rosdep build-type key and relies on colcon to verify the build plugin and complete the build; this updated validator has not yet been run on the developer machine.
- The first launch attempt called unsupported `gz sim` and failed. After switching to Fortress, the developer launched the updated branch: `[ign-1]` remained running; the obstacle monitor changed from `STALE` to `CLEAR; nearest=2.659 m`. This demonstrates the world/simulation and a live scan observation. **Test A PASS** for basic startup. Camera, odometry, motion, and sustained runtime have not yet been independently verified.
- The observed scan range is consistent with the predefined rock in front of the rover. It is initial-position evidence only; no approach/retreat range change or obstacle state transition has been captured yet.
- The runtime validation command was not run because it was invoked from `~`, where `./scripts/phase1_validate.sh` does not exist. Run it from `~/lunabot-v4-phase1` after sourcing the workspace.
- A Ctrl+C trace exposed a command-gate shutdown error (`publisher's context is invalid`). A defensive shutdown fix is now in commit `c0fb19f`; it has not yet been verified on the developer machine.

## Formal acceptance matrix

| Test | Status | Procedure / observed result | Remaining evidence needed |
|---|---|---|---|
| **A — Environment startup** | **PASS** | Updated `ign gazebo` launch remained running; obstacle monitor received `/scan` and reported `CLEAR; nearest=2.659 m`, showing a world sensor observation. | Retest after a clean build and verify camera/odometry topics and sustained operation. |
| **B — Rover stability** | **BLOCKED** | Updated launch produced a live scan, but no visual/physical stationary-stability observation was reported. | Observe chassis/wheel contact, oscillation, and stationary drift in the running world. |
| **C — Forward movement** | **BLOCKED** | No bounded movement command or resulting odometry change was captured. | Run the live motion probe and verify forward displacement. |
| **D — Reverse movement** | **BLOCKED** | Not run. | Live probe verifies reverse displacement. |
| **E — Rotation** | **BLOCKED** | Not run. | Live probe verifies positive and negative yaw changes. |
| **F — Stop behavior** | **BLOCKED** | Watchdog code and clamp unit logic were inspected/tested offline; no stale-command behavior was observed on a running drive system. | Live probe verifies command timeout and stationary odometry after stop. |
| **G — Live camera** | **BLOCKED** | No camera message or rendered image received. | Check actual frame, encoding, dimensions, content, topic frequency, and viewer output. |
| **H — Live range sensing** | **BLOCKED** | Obstacle monitor received a real scan and reported a nearest forward return of 2.659 m. No scan rate or change during approach/turn was recorded. | Record `/scan` rate and verify range changes with approach and orientation. |
| **I — Motion estimates** | **BLOCKED** | The bridge is configured for `/odom` and `/tf`, but no actual odometry sample or TF tree was inspected. | Validate frames, motion signs, and stationary stability from live wheel odometry. |
| **J — Obstacle detection** | **BLOCKED** | Monitor consumed `/scan` and reported `CLEAR; nearest=2.659 m`. Six algorithm-level tests pass, but no live `OBSTACLE` assertion or clear-after-retreat transition was recorded. | Run live approach/retreat; verify Bool and String status transitions plus stale-data handling. |
| **K — Coordinate frames** | **BLOCKED** | SDF/launch text defines intended names and sensor mount offsets; no TF tree/message was available to inspect. | Use `tf2_echo` / `view_frames` and confirm message frame IDs and a single odom/base publisher. |
| **L — Repeatability** | **BLOCKED** | One updated launch was started and produced a scan, but no clean shutdown/relaunch cycle was reported. | Complete a clean stop/start cycle and repeat the basic sensor/motion checks. |

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
- [x] `colcon build --symlink-install` completed both packages on Ubuntu 22.04 / ROS Humble, with an old-workspace overlay warning.
- [ ] `rosdep install` resolves cleanly (`ament_python` definition failed in the reported run).
- [x] Updated Fortress launch started and produced a live scan observation (full runtime validation still pending).
- [ ] Rover stability, forward/reverse/turn/stop, and repeat startup are demonstrated.
- [ ] Actual camera, LiDAR, odometry, clock, and TF streams are verified at runtime.
- [ ] Live sensor-based obstacle status transitions are demonstrated.

**Phase 1 certification: NOT PASSED.** The first launch failed, but the corrected Fortress launch subsequently started and produced a real scan observation. The original colcon build completed; the updated build validator has not yet completed because rosdep failed. Camera/odometry inspection, movement, obstacle state transitions, clean shutdown/relaunch, and other live acceptance evidence remain outstanding.

## Handover boundary

Once built and runtime-validated on ROS 2 Humble/Ignition Fortress, the planned Phase 2 input contracts are `/camera/image_raw`, `/scan`, `odom`/`base_link`/sensor frames, `/odom`, and the same deterministic simulated environment. This report does not implement or claim any Phase 2 perception, mapping, localization, planning, or avoidance behavior.
