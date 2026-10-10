# Phase 1 acceptance record — Lunar Base Camp restart

## Result

**Gate status: `NOT VERIFIED`.** The user built all four packages and launched the Fortress world. Runtime evidence confirms live sensor/odometry/dynamic-TF topics and four transient-local static-TF publishers; a static transform message was received. On the latest launch, `command_guard` failed at import because Humble does not export `RCLError` from `rclpy.exceptions`; the shutdown fix has been corrected to catch a general exception around the best-effort stop, and requires another target rebuild and launch. Full Phase 1 acceptance remains incomplete.

## Environments

| Item | Agent host used for implementation | User target workstation |
|---|---|---|
| Operating system | Debian GNU/Linux 12 (Bookworm), x86_64 | Ubuntu 22.04.5 LTS |
| ROS 2 | `ros2` not found; `ROS_DISTRO` unset | ROS 2 Humble (`ROS_DISTRO=humble`) |
| Gazebo | `ign` / `gz` not found | `/usr/bin/ign`; Gazebo Sim 6.18.0 (`ign gazebo --version`) |
| Build tools | `colcon` not found | `colcon build` available and completed for all four Phase 1 packages |
| Phase 1 workspace on workstation | Not applicable | User cloned pushed branch `arena/6daadcf5-lunabot-v4` under a temporary `$HOME/lunabot-v4-humble.*/repo` path |

The target for this isolated workspace is **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress 6 + `ros_ign_bridge`**. The port uses SDF 1.8, Fortress `ignition-gazebo-*` system-plugin library names with `gz::sim::systems::*` classes, Fortress `ogre2` sensor rendering, and Humble bridge configuration with `ignition.msgs.*` types. GitHub source/docs inspection confirmed the relevant Humble bridge shim, Fortress plugin examples, and SDF 1.8 camera `optical_frame_id` field. That is compatibility research, not a simulator runtime test.

## Executed checks

| Check | Command | Result |
|---|---|---|
| Pure Python policy and static project tests | `python3 -m unittest discover -s tests -v` (from `lunabot_phase1/`) | `PASS` — 23 tests, 0 failures; user ran after building on Humble |
| Python syntax compilation | `python3 -m compileall -q src tests` (from `lunabot_phase1/`) | `PASS` — exit code 0 on the agent host |
| Bringup/tools setup metadata | `python3 src/lunabot_phase1_tools/setup.py --name` and `python3 src/lunabot_phase1_bringup/setup.py --name` | `PASS` — expected package names printed on the agent host |
| Package manifest XML | Parse all four `src/*/package.xml` files | `PASS` — all four parsed |
| Smoke-script syntax | `bash -n scripts/phase1_smoke_test.sh` (from `lunabot_phase1/`) | `PASS` — exit code 0 |
| New-workspace whitespace scan | Check text files under `lunabot_phase1/` | `PASS` — no trailing whitespace |
| Repository whitespace check | `git diff --check` | `PASS` — no whitespace errors |
| Existing reference repository tests | `python3 -m unittest discover -s tests -v` (repository root) | Before isolated changes: 141 tests passed, 4 skipped because optional NumPy is not installed; not rerun for this workspace |
| ROS dependency resolution | `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble` | Completed; warned that rosdep could not resolve `ament_python`, then reported resolvable dependencies installed. This did not stop the build. |
| ROS workspace build | `colcon build --symlink-install --event-handlers console_direct+` | `PASS` — user log reports 4 packages finished on Humble |
| Initial Fortress launch | `ros2 launch lunabot_phase1_bringup phase1.launch.py` | Partial `PASS` — Gazebo Sim 6.18.0 world initialized; bridges started and DiffDrive subscribed to `/cmd_vel_sim` |
| Live scan / obstacle-monitor evidence | Launch log from obstacle monitor | Partial evidence — it reported a valid nearest forward return of 2.75 m, then an obstacle at 1.42 m |
| Live smoke script | `./scripts/phase1_smoke_test.sh` | `NOT VERIFIED` — first run reached `/tf_static` but timed out; latest run stopped at `command_guard` missing because its new shutdown module import failed |
| Static TF inspection | `ros2 topic info /tf_static --verbose` and transient-local echo | Partial evidence — four reliable/transient-local publishers were listed and one static transform was received; full tree/uniqueness still needs `tf2_echo`/`view_frames` |
| Shutdown behavior | Ctrl+C in integrated launch | `NOT VERIFIED` — first shutdown raised `RCLError`; the attempted fix then failed to import `RCLError` on Humble. Source now catches `Exception` only around best-effort shutdown publish; requires another rebuild and runtime check |
| Manual drive, watchdog timing and repeatability | README runtime procedures | `NOT VERIFIED` — no commanded motion, timeout measurement, or second clean launch reported |

Static checks do not establish SDF schema acceptance by Fortress, plugin loading, bridge operation, sensor output, TF connectivity, physics stability, or rover motion. Runtime criteria remain `NOT VERIFIED` until executed on the target workstation.

## Mandatory acceptance gate

`PASS` means the criterion was executed and evidence supports it. `NOT VERIFIED` means runtime behavior has not been demonstrated; code inspection, schema references, compilation, and unit tests alone are not a pass.

| ID | Criterion | Required runtime evidence | Status | Actual result / reason |
|---|---|---|---|---|
| P1-01 | Lunar world loads | Gazebo Fortress starts and renders the world | PASS | User log reports Gazebo Sim 6.18.0 GUI/server and `World [lunar_base_camp] initialized` |
| P1-02 | Rover spawns correctly | Rover is visible and physically stable on the surface | NOT VERIFIED | DiffDrive and sensors initialize, but the user has not confirmed visual spawn and physical stability |
| P1-03 | Forward command works | Observed forward motion and odometry change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-04 | Reverse command works | Observed reverse motion and odometry change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-05 | Both rotations work | Observed left and right yaw change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-06 | Explicit stop works | Rover settles after zero command | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-07 | Command safety works | Measured stop after upstream publisher disappears and 0.50 s expires | NOT VERIFIED | Pure policy is unit-testable; no live timeout measurement |
| P1-08 | Camera publishes valid data | Live image and matching camera-info dimensions; viewpoint changes on motion | NOT VERIFIED | Smoke received live image and CameraInfo messages; dimensions and viewpoint change on motion were not checked |
| P1-09 | LiDAR publishes valid data | Live LaserScan with valid scan metadata/readings | PASS | Monitor received live scan data and reported valid nearest forward returns (2.75 m and later 1.42 m) |
| P1-10 | Physical obstacles affect range | Compare scan/forward range with obstacle in/out of sensor view | NOT VERIFIED | Range/status changed, but no controlled obstacle-in/out comparison was reported |
| P1-11 | Motion estimates are available | Odometry changes for forward, reverse and rotation | NOT VERIFIED | Smoke received `/odom` messages, but no before/after motion comparison was reported |
| P1-12 | Coordinate frames are valid | Inspect complete tree and confirm each transform once | NOT VERIFIED | `/tf_static` reports four transient-local publishers and an echo received `camera_link -> camera_optical_frame`; full tree/uniqueness still needs `tf2_echo` or `view_frames` |
| P1-13 | Obstacle monitor works | Observe CLEAR, OBSTACLE_DETECTED and NO_VALID_MEASUREMENTS cases | NOT VERIFIED | User logs show CLEAR and OBSTACLE_DETECTED readings; NO_VALID_MEASUREMENTS has not been tested, and smoke script node discovery failed |
| P1-14 | Manual teleoperation works | Complete keyboard control sequence, including safe stop/exit | NOT VERIFIED | No keyboard drive/stop sequence reported |
| P1-15 | Integrated system works | Drive with camera, LiDAR, odometry, TF and monitor operating together | NOT VERIFIED | Live camera, scan, odom, and dynamic TF messages were observed; static TF, commanded motion, and coordinated behavior remain unverified |
| P1-16 | Rebuild/relaunch is repeatable | Build, restart Gazebo and repeat the integrated check | NOT VERIFIED | One build and launch completed; a second clean rebuild/relaunch cycle has not been reported |
| P1-17 | Documentation is complete | Humble/Fortress reproduction guide, interfaces, acceptance procedures and limitations are present | PASS | README and this acceptance record describe the selected target stack and retain the runtime gate |

## Final gate decision

**`NOT VERIFIED`** — P1-01 and P1-09 have runtime evidence and are marked `PASS`; additional live topics are observed, but `/tf_static` and the full behavior checks remain incomplete. Diagnose static-TF QoS/publication, rebuild with the Ctrl+C shutdown fix, verify frame connectivity and camera dimensions, then perform manual driving, command-timeout, all monitor states, and repeatability checks. Do not declare Phase 1 passed or begin Phase 2 until every mandatory row is `PASS`.
