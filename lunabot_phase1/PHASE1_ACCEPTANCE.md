# Phase 1 acceptance record — Lunar Base Camp restart

## Result

**Gate status: `NOT VERIFIED`.** The Humble/Fortress compatibility port is implemented and will be checked with ROS-independent static tests on the agent host. No ROS build, simulator launch, live sensor, TF, or rover-motion run has been performed. Source inspection and unit tests do not prove runtime behavior.

## Environments

| Item | Agent host used for implementation | User target workstation |
|---|---|---|
| Operating system | Debian GNU/Linux 12 (Bookworm), x86_64 | Ubuntu 22.04.5 LTS |
| ROS 2 | `ros2` not found; `ROS_DISTRO` unset | ROS 2 Humble (`ROS_DISTRO=humble`) |
| Gazebo | `ign` / `gz` not found | `/usr/bin/ign`; Gazebo Sim 6.18.0 (`ign gazebo --version`) |
| Build tools | `colcon` not found | Not yet checked for this workspace |
| Phase 1 workspace on workstation | Not applicable | The user's local checkout was searched and had zero `lunabot_phase1_bringup/package.xml` manifests; this updated tree must be synced there before local tests |

The target for this isolated workspace is **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress 6 + `ros_ign_bridge`**. The port uses SDF 1.8, Fortress `ignition-gazebo-*` system-plugin library names with `gz::sim::systems::*` classes, Fortress `ogre2` sensor rendering, and Humble bridge configuration with `ignition.msgs.*` types. GitHub source/docs inspection confirmed the relevant Humble bridge shim, Fortress plugin examples, and SDF 1.8 camera `optical_frame_id` field. That is compatibility research, not a simulator runtime test.

## Executed checks

| Check | Command | Result |
|---|---|---|
| Pure Python policy and static project tests | `python3 -m unittest discover -s tests -v` (from `lunabot_phase1/`) | `PASS` — 23 tests, 0 failures |
| Python syntax compilation | `python3 -m compileall -q src tests` (from `lunabot_phase1/`) | `PASS` — exit code 0 |
| Bringup/tools setup metadata | `python3 src/lunabot_phase1_tools/setup.py --name` and `python3 src/lunabot_phase1_bringup/setup.py --name` | `PASS` — expected package names printed |
| Package manifest XML | Parse all four `src/*/package.xml` files | `PASS` — all four parsed |
| Smoke-script syntax | `bash -n scripts/phase1_smoke_test.sh` (from `lunabot_phase1/`) | `PASS` — exit code 0 |
| New-workspace whitespace scan | Check text files under `lunabot_phase1/` | `PASS` — no trailing whitespace |
| Repository whitespace check | `git diff --check` | `PASS` — no whitespace errors in tracked diff |
| Existing reference repository tests | `python3 -m unittest discover -s tests -v` (repository root) | Prior to the Humble/Fortress port: 141 tests passed, 4 skipped because optional NumPy is not installed; not rerun for these isolated changes |
| Live smoke script | `./scripts/phase1_smoke_test.sh` | `NOT VERIFIED` — attempted; exited 2 with `ros2 is not available`, before live checks |
| ROS workspace build | `colcon build --symlink-install` | `NOT VERIFIED` — ROS 2 and `colcon` are not installed on the agent host; user workstation does not yet have this workspace synced |
| Gazebo startup, sensors, TF, motion and timeout | `ros2 launch ...` and README runtime procedures | `NOT VERIFIED` — no runtime execution has occurred |

Static checks do not establish SDF schema acceptance by Fortress, plugin loading, bridge operation, sensor output, TF connectivity, physics stability, or rover motion. Runtime criteria remain `NOT VERIFIED` until executed on the target workstation.

## Mandatory acceptance gate

`PASS` means the criterion was executed and evidence supports it. `NOT VERIFIED` means runtime behavior has not been demonstrated; code inspection, schema references, compilation, and unit tests alone are not a pass.

| ID | Criterion | Required runtime evidence | Status | Actual result / reason |
|---|---|---|---|---|
| P1-01 | Lunar world loads | Gazebo Fortress starts and renders the world | NOT VERIFIED | No world launched; agent host has no Gazebo and the target checkout lacks the synced workspace |
| P1-02 | Rover spawns correctly | Rover is visible and physically stable on the surface | NOT VERIFIED | No Gazebo runtime |
| P1-03 | Forward command works | Observed forward motion and odometry change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-04 | Reverse command works | Observed reverse motion and odometry change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-05 | Both rotations work | Observed left and right yaw change | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-06 | Explicit stop works | Rover settles after zero command | NOT VERIFIED | No ROS/Gazebo runtime |
| P1-07 | Command safety works | Measured stop after upstream publisher disappears and 0.50 s expires | NOT VERIFIED | Pure policy is unit-testable; no live timeout measurement |
| P1-08 | Camera publishes valid data | Live image and matching camera-info dimensions; viewpoint changes on motion | NOT VERIFIED | No camera renderer or ROS topics |
| P1-09 | LiDAR publishes valid data | Live LaserScan with valid scan metadata/readings | NOT VERIFIED | No sensor runtime |
| P1-10 | Physical obstacles affect range | Compare scan/forward range with obstacle in/out of sensor view | NOT VERIFIED | No simulated range readings |
| P1-11 | Motion estimates are available | Odometry changes for forward, reverse and rotation | NOT VERIFIED | No ROS topics or drive system |
| P1-12 | Coordinate frames are valid | Inspect complete tree and confirm each transform once | NOT VERIFIED | No TF runtime inspection |
| P1-13 | Obstacle monitor works | Observe CLEAR, OBSTACLE_DETECTED and NO_VALID_MEASUREMENTS cases | NOT VERIFIED | Pure logic is covered by unit tests; ROS subscriptions/publications have not run |
| P1-14 | Manual teleoperation works | Complete keyboard control sequence, including safe stop/exit | NOT VERIFIED | No ROS node can run on the agent host; target checkout not synced |
| P1-15 | Integrated system works | Drive with camera, LiDAR, odometry, TF and monitor operating together | NOT VERIFIED | No integrated runtime |
| P1-16 | Rebuild/relaunch is repeatable | Build, restart Gazebo and repeat the integrated check | NOT VERIFIED | No build or simulator run |
| P1-17 | Documentation is complete | Humble/Fortress reproduction guide, interfaces, acceptance procedures and limitations are present | PASS | README and this acceptance record describe the selected target stack and retain the runtime gate |

## Final gate decision

**`NOT VERIFIED`** — P1-01 through P1-16 have not been demonstrated. The next step is to sync this `lunabot_phase1/` directory into the user's local checkout, build it in a clean Ubuntu 22.04 / ROS 2 Humble shell, launch it with the installed Fortress 6.18.0 runtime, run the smoke and manual-drive sequence in `README.md`, and update each criterion with actual evidence. Do not declare Phase 1 passed or begin Phase 2 until every mandatory runtime row is `PASS`.
