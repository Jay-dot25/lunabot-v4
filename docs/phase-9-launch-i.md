# Phase I — Full Autonomous Integration (`launch-i`)

## 1. Scope and gate

Phase I integrates the approved terrain-aware planning pipeline with actual
rover motion. It starts the Phase E perception, Phase F semantic mapper, Phase
G cost mapper, Phase H terrain-aware planner, and a conservative terrain-path
follower. The follower publishes only to `/cmd_vel_in`, preserving the approved
Phase B controller/watchdog boundary before safe `/cmd_vel` output.

The original Phase D A* planner remains active as a diagnostic goal/path
source, but its command output is remapped away from `/cmd_vel_in`. This avoids
two competing motion publishers. The active motion source in Phase I is the
terrain-aware follower; Phase I is the first phase that connects the terrain
plan to autonomous motion. Unlike Phase H, this phase does not keep the
terrain-aware path as an output without motion output; it drives through the
controller boundary.

Phase I is independently launchable as `~/launch-i`. Static validation is not
runtime acceptance. The workstation runtime must produce a real integrated
goal completion, preserve the approved Phase A-H gates, and cleanly relaunch
before Phase I can be approved.

## 2. Data flow

```text
RGB-D -> Phase E segmentation -> Phase F semantic map -> Phase G cost map
                                                          │
                                                          ▼
                                                    Phase H weighted A*
                                                          │
                                              /lunabot/terrain/plan
                                                          │
                                                          ▼
                                             Phase I terrain path follower
                                                          │
                                                /cmd_vel_in (only active
                                                planner motion output)
                                                          │
                                                          ▼
                                       Phase B controller/watchdog -> /cmd_vel
```

The diagnostic Phase D A* planner still consumes `/map` and publishes `/plan`
and status, but its command topic is isolated at
`/lunabot/navigation/diagnostic_cmd_vel`. It cannot compete with the active
terrain follower.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/plan` | `nav_msgs/Path` | Phase H active terrain path |
| `/goal_pose` | `geometry_msgs/PoseStamped` | shared navigation goal |
| `/lunabot/odom` | `nav_msgs/Odometry` | real motion pose source |
| `map -> chassis` TF | `tf2` | existing SLAM + odometry localization |
| `/cmd_vel_in` | `geometry_msgs/Twist` | Phase I follower output into Phase B |
| `/lunabot/autonomy/status` | `std_msgs/String` | integrated following and goal status |
| `/cmd_vel` | `geometry_msgs/Twist` | safe controller output to Gazebo |

The follower publishes a conservative maximum of 0.20 m/s linear and 0.60
rad/s angular speed. The Phase B controller continues to clamp, acceleration
limit, watchdog, and publish the final `/cmd_vel`.

A successful integration status contains `INTEGRATION_GOAL_REACHED`.

## 4. Installation

Phase I uses the approved Phase H workstation dependencies. No additional
navigation stack is required.

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-i" ~/launch-i
chmod +x launch-i scripts/launch-i.sh scripts/terrain_path_follower.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_i.py
```

The validator checks the approved Phase A-H baseline, command-topic isolation,
active follower output, terrain plan/status, controller boundary, real runtime
checks, RViz output, evidence paths, and honest runtime gating. It never claims
that the integrated rover reached a goal.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i
```

The run starts the approved world, bridge, static TF, controller, odometry
monitor, perception, semantic map, cost map, terrain-aware planner, diagnostic
A*, and active terrain path follower directly. It must validate the inherited
Phase A-H interfaces and these Phase I interfaces:

```text
/lunabot/autonomy/status       std_msgs/String
INTEGRATION_GOAL_REACHED
/cmd_vel_in                    follower output into Phase B
/cmd_vel                       controller output to Gazebo
A* diagnostic path/status      retained without command conflict
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE I RUN COMPLETE - overall result: PASS
Launch I environment cleanly closed.
```

The integration status and goal checks require real `ros2 topic type` and
`ros2 topic echo --once` messages. A follower process being alive is not a
pass.

## 7. GUI/RViz run

After the headless gate passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-i
```

RViz retains the perception, semantic map, cost map, terrain-aware plan,
diagnostic A* path, camera, LiDAR, odometry, map, and TF displays. Confirm the
terrain-aware path is the active motion path and that the rover moves through
`/cmd_vel_in -> Phase B controller -> /cmd_vel`. Press Ctrl+C for clean
shutdown.

## 8. Evidence

Runtime evidence is written to `evidence/phase-i-launch-i/`:

| File | Meaning |
|---|---|
| `integration.log` | active terrain follower diagnostics |
| `integration_status.txt` | real integrated status sample |
| `goal_wait_status.txt` | real integrated goal wait stream containing `INTEGRATION_GOAL_REACHED` |
| `cmd_vel_in_motion.txt` | captured nonzero active terrain-follower input |
| `cmd_vel_motion.txt` | captured nonzero controller output |
| `controller_boundary_status.txt` | active `/cmd_vel_in` to `/cmd_vel` controller evidence |
| `terrain_plan.txt` | active terrain-aware path sample |
| `navigation.log` | diagnostic A* log |
| `navigation_status.txt` | diagnostic A* status sample |
| `topics.txt` | runtime topic graph |
| `last_run.log` | ordered launcher and validation result |
| `map_before_navigation.txt` / `map_after_navigation.txt` | inherited map-motion gate |
| `phase_i_map.yaml` / `phase_i_map.pgm` | required final SLAM map evidence |
| `static_validation.txt` | generated static Phase I report |

Do not hand-edit runtime evidence. Inspect integration, terrain planner,
controller, diagnostic A*, bridge/Gazebo logs, and `last_run.log` when a check
fails.

## 9. Clean relaunch gate

After the autonomous or GUI run:

```text
Launch I environment cleanly closed.
```

Then repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i
```

Both runs must start one active follower, keep diagnostic A* isolated from the
controller boundary, produce `INTEGRATION_GOAL_REACHED`, and stop all process
groups. This clean relaunch is part of the Phase I runtime gate.

## 10. Acceptance checklist

- [ ] Phase A-H static and runtime contracts remain passing.
- [ ] Terrain-aware plan is a real input message.
- [ ] The active follower publishes real `/cmd_vel_in` commands.
- [ ] Diagnostic A* cannot publish to `/cmd_vel_in`.
- [ ] Phase B controller remains between follower and `/cmd_vel`.
- [ ] Integration status contains `INTEGRATION_GOAL_REACHED`.
- [ ] Rover reaches the goal using the terrain-aware path.
- [ ] Final map evidence is saved as YAML and PGM.
- [ ] Clean shutdown and a second independent relaunch pass.

Phase I was explicitly runtime-approved after two independent passing headless
runs, final map evidence, clean shutdown, and clean relaunch. Phase J may now
begin; Phase K must wait for its own explicit runtime approval.
