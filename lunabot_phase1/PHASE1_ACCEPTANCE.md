# Phase 1 acceptance record — Lunar Base Camp restart

## Result

**Gate status: `NOT VERIFIED`.** The user previously built all four packages on Humble, passed the live-interface smoke test (including `/tf_static`), and observed a clean Fortress shutdown. Subsequent `/odom` samples show translations aligned with the rover's unchanged heading in both forward and reverse directions, but the exact key sequence and visual response were not explicitly confirmed. CameraInfo was observed at `camera_optical_frame`, 640×480. After the user shared a rover infographic as a design reference, the current branch was updated with four shallow physical regolith domes and a six-wheel rover silhouette: three driven wheels per side, fixed rocker-bogie-inspired side rails, a raised camera mast, LiDAR, solar panel, and front bumper. Its approximate envelope and simulated mass are 1.2 × 0.8 × 0.6 m and 80 kg. The side rails are visual/fixed; this is not an articulated rocker-bogie suspension. These latest changes have static tests only and still require a fresh Humble/Fortress build and runtime check. Do not mark the gate passed until the updated rover is visibly stable and every mandatory runtime criterion is verified.

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
| Pure Python policy and static project tests | `python3 -m unittest discover -s tests -v` (from `lunabot_phase1/`) | `PASS` — 26 tests, 0 failures on the current six-wheel/terrain files; static checks only, not Gazebo runtime validation |
| Python syntax compilation | `python3 -m compileall -q src tests` (from `lunabot_phase1/`) | `PASS` — exit code 0 on the agent host |
| Bringup/tools setup metadata | `python3 src/lunabot_phase1_tools/setup.py --name` and `python3 src/lunabot_phase1_bringup/setup.py --name` | `PASS` — expected package names printed on the agent host |
| Package manifest XML | Parse all four `src/*/package.xml` files | `PASS` — all four parsed |
| Smoke-script syntax | `bash -n scripts/phase1_smoke_test.sh` (from `lunabot_phase1/`) | `PASS` — exit code 0 |
| New-workspace whitespace scan | Check text files under `lunabot_phase1/` | `PASS` — no trailing whitespace |
| Repository whitespace check | `git diff --check` | `PASS` — no whitespace errors |
| Existing reference repository tests | `python3 -m unittest discover -s tests -v` (repository root) | `PASS` — 141 tests, 4 skipped because optional NumPy is not installed; rerun after the isolated rover update, no failures |
| ROS dependency resolution | `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble` | Completed; warned that rosdep could not resolve `ament_python`, then reported resolvable dependencies installed. This did not stop the build. |
| ROS workspace build | `colcon build --symlink-install --event-handlers console_direct+` | `PASS` — user log reports 4 packages finished on Humble |
| Initial Fortress launch | `ros2 launch lunabot_phase1_bringup phase1.launch.py` | Partial `PASS` — Gazebo Sim 6.18.0 world initialized; bridges started and DiffDrive subscribed to `/cmd_vel_sim` |
| Latest prior integrated relaunch and shutdown (before the six-wheel redesign) | `ros2 launch lunabot_phase1_bringup phase1.launch.py`, then Ctrl+C | `PASS` — 2026-10-10 log shows Fortress GUI/server and `lunar_base_camp` initialized, bridges and DiffDrive started, and all launch children exited cleanly |
| Keyboard teleop package resolution/startup | `ros2 pkg prefix lunabot_phase1_tools`; `ros2 run lunabot_phase1_tools keyboard_teleop` | Partial evidence — package resolved under the workspace `install/` directory and teleop printed its controls. A later session remained in raw keyboard mode; keypresses are intentionally not echoed. No complete per-key outcome sequence was recorded. |
| Odometry snapshots | Two user `ros2 topic echo /odom --once` samples | Partial evidence — pose changed from `(6.6780, -0.0002, yaw≈-0.6603 rad)` to `(8.7431, -0.6970, yaw≈0.9573 rad)` (Δx≈2.0651 m, Δy≈-0.6968 m, Δyaw≈1.6176 rad); both sampled twists were zero. This confirms changing pose samples but not the effect of each individual direction command. |
| Later odometry pair | Two user `ros2 topic echo /odom --once` samples at sim stamps 397.872 s and 477.864 s | Partial evidence — pose moved `(-0.1826, +0.9333) m` with unchanged yaw (about `101.07°`), aligned with the forward axis; this is consistent with forward input but the exact keys/visual motion were not explicitly confirmed. |
| Reverse-consistent odometry pair | User samples at sim stamps 477.864 s and 1757.376 s | Partial evidence — pose moved `(+0.3423, -1.7498) m` (1.783 m) with unchanged yaw, aligned with the reverse axis; user has not yet explicitly confirmed that `S` alone was pressed in this interval. |
| CameraInfo dimensions | `ros2 topic echo /camera/camera_info --qos-reliability best_effort --once` | Partial evidence — user observed `frame_id: camera_optical_frame`, `height: 480`, `width: 640`; the log does not show image-message dimensions or a camera-viewpoint comparison during rover motion. |
| Live scan / obstacle-monitor evidence | Launch log from obstacle monitor | Partial evidence — it reported a valid nearest forward return of 2.75 m, then an obstacle at 1.42 m |
| Live smoke script | `./scripts/phase1_smoke_test.sh` | `PASS` — latest user run passed all expected node and message-type checks and received live messages on all smoke-test topics, including `/tf_static` |
| Static TF inspection | `ros2 topic info /tf_static --verbose` and reliable/transient-local echo | `PASS` — four reliable/transient-local publishers were listed and a static transform was received; the complete tree/uniqueness criterion still requires `tf2_echo`/`view_frames` |
| Shutdown behavior | Ctrl+C in integrated launch | `PASS` — after rebuilding the Humble-safe correction, `command_guard` and all launch children exited cleanly without the prior import/context errors |
| Terrain / rover appearance revision | SDF: four perimeter mounds, six driven wheels, fixed rocker-bogie-style side rails, camera mast, solar panel, and bumper; 26 static tests | `PASS` static checks only — this is a code-level change; the updated model has not yet been rebuilt or run on the user's Fortress workstation |
| Manual drive and watchdog timing | README runtime procedures | `NOT VERIFIED` — teleop started and odometry samples differed, but no direction-by-direction key/visual record or measured publisher-loss-to-stop interval was supplied |

Static checks do not establish SDF schema acceptance by Fortress, plugin loading, bridge operation, sensor output, TF connectivity, physics stability, or rover motion. Runtime criteria remain `NOT VERIFIED` until executed on the target workstation.

## Mandatory acceptance gate

`PASS` means the criterion was executed and evidence supports it. `NOT VERIFIED` means runtime behavior has not been demonstrated; code inspection, schema references, compilation, and unit tests alone are not a pass.

| ID | Criterion | Required runtime evidence | Status | Actual result / reason |
|---|---|---|---|---|
| P1-01 | Lunar world loads | Gazebo Fortress starts and renders the world | NOT VERIFIED | The prior revision loaded; the current terrain and six-wheel rover revision has not yet been run on Fortress |
| P1-02 | Rover spawns correctly | Rover is visible and physically stable on the surface | NOT VERIFIED | Current SDF has six driven wheels and fixed rocker-bogie-style visual rails; no target runtime has confirmed spawning or stability |
| P1-03 | Forward command works | Observed forward motion and odometry change | NOT VERIFIED | A later pose delta of `(-0.1826, +0.9333) m` aligns with the unchanged forward heading; exact `W` input and visible response were not explicitly confirmed, and the updated revision is untested |
| P1-04 | Reverse command works | Observed reverse motion and odometry change | NOT VERIFIED | A later `(+0.3423, -1.7498) m` delta aligns with reverse at unchanged yaw; exact `S`-only input and visible response were not explicitly confirmed, and the updated revision is untested |
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
| P1-16 | Rebuild/relaunch is repeatable | Build, restart Gazebo and repeat the integrated check | NOT VERIFIED | The prior revision built and relaunched; the latest terrain and six-wheel rover revision still needs a target build and smoke run |
| P1-17 | Documentation is complete | Humble/Fortress reproduction guide, interfaces, acceptance procedures and limitations are present | PASS | README and this acceptance record describe the selected target stack and retain the runtime gate |

## Final gate decision

**`NOT VERIFIED`** — the earlier revision passed the live-interface smoke test and shut down cleanly. The current revision adds low-relief terrain and the six-wheel rover design, so P1-01, P1-02, and P1-16 need a fresh target build/launch before being marked `PASS`. Forward/reverse-consistent odometry deltas are recorded but need explicit key/visual confirmation. Remaining work includes both rotations, explicit stop, measured watchdog timing, camera dimension matching/viewpoint, full TF connectivity, controlled obstacle-in/out range behavior, and `NO_VALID_MEASUREMENTS`. Do not declare Phase 1 passed or begin Phase 2 until every mandatory row is `PASS`.
