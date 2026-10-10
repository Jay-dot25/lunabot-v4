# Phase 1 acceptance record — Lunar Base Camp restart

## Result

**Gate status: `NOT VERIFIED`.** The user rebuilt all four packages on Humble, passed the live-interface smoke test (including `/tf_static`), and the 2026-10-10 Fortress relaunch again shut down all launch children cleanly. The latest output confirms `/camera/camera_info` uses `camera_optical_frame` at 640×480. Two `/odom` samples changed from `(x=6.6780, y=-0.0002, yaw≈-0.6603 rad)` to `(x=8.7431, y=-0.6970, yaw≈0.9573 rad)`; this is evidence that the reported odometry pose changed, but the submitted log does not identify which key inputs caused the change or document each direction separately. Teleop's raw-key terminal intentionally does not echo pressed keys. Image/CameraInfo dimension matching, camera viewpoint change, direction-specific driving and stopping, watchdog timing, full TF connectivity, and the remaining obstacle-monitor cases are not all verified, so Phase 1 is not yet accepted.

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
| Latest integrated relaunch and shutdown | `ros2 launch lunabot_phase1_bringup phase1.launch.py`, then Ctrl+C | `PASS` — 2026-10-10 log shows Fortress GUI/server and `lunar_base_camp` initialized, bridges and DiffDrive started, and all launch children exited cleanly |
| Keyboard teleop package resolution/startup | `ros2 pkg prefix lunabot_phase1_tools`; `ros2 run lunabot_phase1_tools keyboard_teleop` | Partial evidence — package resolved under the workspace `install/` directory and teleop printed its controls. A later session remained in raw keyboard mode; keypresses are intentionally not echoed. No complete per-key outcome sequence was recorded. |
| Odometry snapshots | Two user `ros2 topic echo /odom --once` samples | Partial evidence — pose changed from `(6.6780, -0.0002, yaw≈-0.6603 rad)` to `(8.7431, -0.6970, yaw≈0.9573 rad)` (Δx≈2.0651 m, Δy≈-0.6968 m, Δyaw≈1.6176 rad); both sampled twists were zero. This confirms changing pose samples but not the effect of each individual direction command. |
| CameraInfo dimensions | `ros2 topic echo /camera/camera_info --qos-reliability best_effort --once` | Partial evidence — user observed `frame_id: camera_optical_frame`, `height: 480`, `width: 640`; the log does not show image-message dimensions or a camera-viewpoint comparison during rover motion. |
| Live scan / obstacle-monitor evidence | Launch log from obstacle monitor | Partial evidence — it reported a valid nearest forward return of 2.75 m, then an obstacle at 1.42 m |
| Live smoke script | `./scripts/phase1_smoke_test.sh` | `PASS` — latest user run passed all expected node and message-type checks and received live messages on all smoke-test topics, including `/tf_static` |
| Static TF inspection | `ros2 topic info /tf_static --verbose` and reliable/transient-local echo | `PASS` — four reliable/transient-local publishers were listed and a static transform was received; the complete tree/uniqueness criterion still requires `tf2_echo`/`view_frames` |
| Shutdown behavior | Ctrl+C in integrated launch | `PASS` — after rebuilding the Humble-safe correction, `command_guard` and all launch children exited cleanly without the prior import/context errors |
| Manual drive and watchdog timing | README runtime procedures | `NOT VERIFIED` — teleop started and odometry samples differed, but no direction-by-direction key/visual record or measured publisher-loss-to-stop interval was supplied |

Static checks do not establish SDF schema acceptance by Fortress, plugin loading, bridge operation, sensor output, TF connectivity, physics stability, or rover motion. Runtime criteria remain `NOT VERIFIED` until executed on the target workstation.

## Mandatory acceptance gate

`PASS` means the criterion was executed and evidence supports it. `NOT VERIFIED` means runtime behavior has not been demonstrated; code inspection, schema references, compilation, and unit tests alone are not a pass.

| ID | Criterion | Required runtime evidence | Status | Actual result / reason |
|---|---|---|---|---|
| P1-01 | Lunar world loads | Gazebo Fortress starts and renders the world | PASS | User log reports Gazebo Sim 6.18.0 GUI/server and `World [lunar_base_camp] initialized` |
| P1-02 | Rover spawns correctly | Rover is visible and physically stable on the surface | NOT VERIFIED | DiffDrive and sensors initialize, but the user has not confirmed visual spawn and physical stability |
| P1-03 | Forward command works | Observed forward motion and odometry change | NOT VERIFIED | `/odom` samples differ, but the log does not identify a `W`-only interval or confirm observed forward motion against before/after samples |
| P1-04 | Reverse command works | Observed reverse motion and odometry change | NOT VERIFIED | No `S`-only test and corresponding before/after observation were recorded |
| P1-05 | Both rotations work | Observed left and right yaw change | NOT VERIFIED | Yaw differs between the two samples, but separate observed `A` and `D` tests were not recorded |
| P1-06 | Explicit stop works | Rover settles after zero command | NOT VERIFIED | Teleop printed its exit message once, but an explicit stop and the rover settling afterward were not documented |
| P1-07 | Command safety works | Measured stop after upstream publisher disappears and 0.50 s expires | NOT VERIFIED | Guard logged safe-stop messages after stale input, but publisher-loss-to-zero timing and rover settling were not measured |
| P1-08 | Camera publishes valid data | Live image and matching camera-info dimensions; viewpoint changes on motion | NOT VERIFIED | Earlier smoke received image and CameraInfo messages; the latest CameraInfo sample is 640×480, but image dimensions were not compared and viewpoint change on motion was not checked |
| P1-09 | LiDAR publishes valid data | Live LaserScan with valid scan metadata/readings | PASS | Monitor received live scan data and reported valid nearest forward returns (2.75 m and later 1.42 m) |
| P1-10 | Physical obstacles affect range | Compare scan/forward range with obstacle in/out of sensor view | NOT VERIFIED | Range/status changed, but no controlled obstacle-in/out comparison was reported |
| P1-11 | Motion estimates are available | Odometry changes for forward, reverse and rotation | NOT VERIFIED | Two `/odom` poses differ by about `(+2.0651, -0.6968) m` and `+1.6176 rad` yaw, but the test does not attribute those changes to separately recorded forward, reverse, and rotation commands |
| P1-12 | Coordinate frames are valid | Inspect complete tree and confirm each transform once | NOT VERIFIED | `/tf_static` has four transient-local publishers and one transform was received; full tree/uniqueness still needs `tf2_echo` or `view_frames` |
| P1-13 | Obstacle monitor works | Observe CLEAR, OBSTACLE_DETECTED and NO_VALID_MEASUREMENTS cases | NOT VERIFIED | User logs show CLEAR and OBSTACLE_DETECTED readings and smoke received status/range messages; NO_VALID_MEASUREMENTS has not been tested |
| P1-14 | Manual teleoperation works | Complete keyboard control sequence, including safe stop/exit | NOT VERIFIED | Teleop resolved and started; the log does not confirm the full `W/S/A/D`, explicit-stop, and quit sequence or its visible outcomes. Raw-mode keypresses do not echo. |
| P1-15 | Integrated system works | Drive with camera, LiDAR, odometry, TF and monitor operating together | NOT VERIFIED | Live camera, scan, odom, and TF messages were observed, and odometry poses differed; direction-specific drive, camera viewpoint, complete TF, and coordinated behavior remain unverified |
| P1-16 | Rebuild/relaunch is repeatable | Build, restart Gazebo and repeat the integrated check | PASS | User rebuilt all four packages, relaunched repeatedly, and most recently passed the full live-interface smoke test with clean shutdown |
| P1-17 | Documentation is complete | Humble/Fortress reproduction guide, interfaces, acceptance procedures and limitations are present | PASS | README and this acceptance record describe the selected target stack and retain the runtime gate |

## Final gate decision

**`NOT VERIFIED`** — P1-01, P1-09, P1-16, and P1-17 have runtime/documentation evidence and are marked `PASS`. The live-interface smoke test passes and the latest Ctrl+C shutdown was clean. A camera-info resolution and changing odometry samples are recorded, but the required key-specific drive/stop behavior and matching image/viewpoint checks are incomplete. Remaining work includes visual rover stability, forward/reverse/left/right and explicit-stop evidence, measured watchdog timing, full TF connectivity, controlled obstacle-in/out range behavior, and `NO_VALID_MEASUREMENTS`. Do not declare Phase 1 passed or begin Phase 2 until every mandatory row is `PASS`.
