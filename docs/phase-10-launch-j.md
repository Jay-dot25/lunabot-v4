# Phase J — Dynamic Replanning (`launch-j`)

## 1. Scope and gate

Phase J preserves the runtime-approved Phase I full-autonomy stack and adds an
independent dynamic-replanning monitor. The Phase H terrain-aware planner
continues to replan from the live terrain cost map, goal, and `/lunabot/odom`.
The new monitor makes changed terrain-plan revisions auditable while the rover
is moving. It observes the active plan and publishes status only; it never
publishes velocity and cannot compete with the Phase I follower.

Phase J is independently launchable as `~/launch-j`. It is not a wrapper around
`launch-i` and does not replace the approved terrain planner, follower,
controller, SLAM, or diagnostic A* interfaces.

Static validation is not runtime acceptance. The runtime gate must prove at
least two changed terrain-plan revisions after real odometry motion, preserve
the Phase A-I runtime contracts, reach the active goal, save final map
 evidence, shut down cleanly, and pass a clean relaunch.

## 2. Data flow

```text
RGB-D -> semantic map -> cost map -> terrain-aware planner
                                      │
                         changed /lunabot/terrain/plan revisions
                                      │
                                      ├──> dynamic_replan_monitor
                                      │       -> /lunabot/autonomy/replan_status
                                      │
                                      └──> terrain_path_follower
                                              -> /cmd_vel_in
                                              -> Phase B controller -> /cmd_vel
```

The monitor requires:

- a shared `/goal_pose`;
- real `/lunabot/odom` motion;
- at least two distinct terrain-plan signatures; and
- a minimum accumulated odometry distance of 0.05 m.

A successful status contains `DYNAMIC_REPLAN_PASS` with revision and motion
counts. The active follower remains the only planner publishing to
`/cmd_vel_in`.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/plan` | `nav_msgs/Path` | live terrain-aware plan input |
| `/lunabot/terrain/planner/status` | `std_msgs/String` | planner status observed by monitor |
| `/goal_pose` | `geometry_msgs/PoseStamped` | active goal |
| `/lunabot/odom` | `nav_msgs/Odometry` | real motion and revision gate |
| `/lunabot/autonomy/replan_status` | `std_msgs/String` | dynamic replanning evidence |
| `/cmd_vel_in` | `geometry_msgs/Twist` | unchanged Phase I follower boundary |
| `/cmd_vel` | `geometry_msgs/Twist` | unchanged controller output |

The monitor uses reliable transient-local QoS for retained plan/status messages
and sensor-compatible best-effort QoS for odometry input. It does not create a
second localization or navigation stack.

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-j" ~/launch-j
chmod +x launch-j scripts/launch-j.sh scripts/dynamic_replan_monitor.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_j.py
```

The validator checks the approved Phase I baseline, independent launch-j
resolution, dynamic monitor implementation, active command isolation, retained
status QoS, evidence paths, and honest runtime gating. It does not claim that
dynamic replanning occurred on a real rover.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j
```

The run must contain:

```text
terrain-aware plan content: PASS
dynamic replan status content: PASS
dynamic terrain-plan replanning: PASS
DYNAMIC_REPLAN_PASS
terrain-integrated goal reached: PASS
nonzero /cmd_vel_in motion evidence: PASS
controller output /cmd_vel motion evidence: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE J RUN COMPLETE - overall result: PASS
Launch J environment cleanly closed.
```

## 7. Evidence

Runtime evidence is written to `evidence/phase-j-launch-j/`:

| File | Meaning |
|---|---|
| `replan.log` | dynamic monitor diagnostics and revision count |
| `replan_status.txt` | retained dynamic-replanning status sample |
| `replan_wait_status.txt` | live stream proving `DYNAMIC_REPLAN_PASS` |
| `terrain_plan.txt` | active terrain-aware plan sample |
| `cmd_vel_in_motion.txt` | nonzero follower input evidence |
| `cmd_vel_motion.txt` | nonzero controller output evidence |
| `controller_boundary_status.txt` | Phase B boundary evidence |
| `phase_j_map.yaml` / `phase_j_map.pgm` | final map evidence |
| `static_validation.txt` | static Phase J report |
| `last_run.log` | ordered launcher and validation result |

Do not hand-edit runtime evidence. Inspect `replan.log`,
`terrain_planner.log`, `integration.log`, `navigation.log`, and `last_run.log`
when a gate fails.

## 8. Clean relaunch gate

After the first autonomous run, repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j
```

Both independent runs must prove changed terrain-plan revisions during motion,
reach the goal, preserve the controller boundary, and stop all process groups.

## 9. Acceptance checklist

- [ ] Phase I static and runtime approval remains intact.
- [ ] Dynamic monitor is independently launched.
- [ ] At least two changed terrain-plan revisions are observed.
- [ ] Revisions occur after real `/lunabot/odom` motion.
- [ ] `DYNAMIC_REPLAN_PASS` is received on `/lunabot/autonomy/replan_status`.
- [ ] The follower remains the only `/cmd_vel_in` motion source.
- [ ] Goal completion, motion, map updates, and final map evidence pass.
- [ ] Clean shutdown and clean relaunch pass.

Do not begin Phase K until Phase J receives explicit runtime approval.
