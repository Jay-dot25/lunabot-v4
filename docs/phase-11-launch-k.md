# Phase K — Testing and Evaluation (`launch-k`)

## 1. Scope and gate

Phase K preserves the explicitly approved Phase J dynamic-replanning stack and
adds an independent runtime evaluator. The evaluator observes the real plan,
dynamic-replan status, integrated goal result, controller status, odometry, and
both sides of the `/cmd_vel_in -> /cmd_vel` boundary. It publishes an aggregate
result but never publishes velocity or changes navigation state.

Phase K is independently launchable as `~/launch-k`. It is not a wrapper
around `launch-j` and does not add another SLAM, localization, or navigation
stack.

Static validation is not runtime acceptance. The runtime gate must produce a
real `EVALUATION_PASS`, preserve the Phase A-J contracts, save final map
evidence, shut down cleanly, and pass a clean relaunch.

## 2. Evaluation data flow

```text
terrain plan ───────────────┐
dynamic replan status ──────┤
integrated autonomy status ┤
controller status ──────────┤
odometry ────────────────────┤──> Phase K evaluator
/cmd_vel_in ────────────────┤        │
/cmd_vel ───────────────────┘        ▼
                              /lunabot/evaluation/status
```

The evaluator requires all of the following:

- a non-empty terrain-aware plan;
- a non-empty geometric `/plan` baseline from diagnostic A*;
- real geometric and terrain-aware path-length measurements in the aggregate
  result;
- `DYNAMIC_REPLAN_PASS`;
- `INTEGRATION_GOAL_REACHED`;
- an active controller status;
- nonzero `/cmd_vel_in` motion;
- nonzero `/cmd_vel` output;
- at least 0.05 m of real odometry motion.

A successful status contains `EVALUATION_PASS`, measured geometric and
terrain-aware path lengths, command samples, and odometry motion. The
comparison is based on paths emitted during the same live scenario; it does
not fabricate success or claim that the two controllers ran simultaneously.
The evaluator is observation-only and cannot compete with the approved Phase J
motion source.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/plan` | `nav_msgs/Path` | terrain-plan evaluation input |
| `/lunabot/autonomy/replan_status` | `std_msgs/String` | dynamic-replanning result |
| `/lunabot/autonomy/status` | `std_msgs/String` | integrated goal result |
| `/lunabot/control/status` | `std_msgs/String` | controller health |
| `/lunabot/odom` | `nav_msgs/Odometry` | measured motion |
| `/cmd_vel_in` | `geometry_msgs/Twist` | follower-side motion sample |
| `/cmd_vel` | `geometry_msgs/Twist` | controller-side motion sample |
| `/lunabot/evaluation/status` | `std_msgs/String` | aggregate evaluation result |

The evaluator uses reliable transient-local QoS for retained status and plan
messages and sensor-compatible best-effort QoS for odometry. It creates no
motion publisher.

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-k" ~/launch-k
chmod +x launch-k scripts/launch-k.sh scripts/phase_k_evaluator.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_k.py
```

The validator checks the approved Phase J baseline, independent launch-k
resolution, evaluator inputs and output, command isolation, evidence paths,
and honest runtime gating. It does not claim that a real evaluation passed.

## 6. Automated headless runtime gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k
```

The run must contain:

```text
 dynamic terrain-plan replanning: PASS
 terrain-integrated goal reached: PASS
 aggregate runtime evaluation: PASS
 EVALUATION_PASS
 nonzero /cmd_vel_in motion evidence: PASS
 controller output /cmd_vel motion evidence: PASS
 live map updates during autonomous navigation: PASS
 saved map evidence (YAML + PGM): PASS
 PHASE K RUN COMPLETE - overall result: PASS
 Launch K environment cleanly closed.
```

## 7. Evidence

Runtime evidence is written to `evidence/phase-k-launch-k/`:

| File | Meaning |
|---|---|
| `evaluation.log` | evaluator diagnostics and measured metrics |
| `evaluation_status.txt` | aggregate evaluation status sample |
| `evaluation_wait_status.txt` | live stream proving `EVALUATION_PASS` |
| `replan_status.txt` | dynamic-replanning status sample |
| `terrain_plan.txt` | active terrain plan sample |
| `cmd_vel_in_motion.txt` | follower input evidence |
| `cmd_vel_motion.txt` | controller output evidence |
| `controller_boundary_status.txt` | Phase B boundary evidence |
| `odometry_report.txt` | measured motion and continuity |
| `phase_k_map.yaml` / `phase_k_map.pgm` | final map evidence |
| `static_validation.txt` | static Phase K report |
| `last_run.log` | ordered launcher and validation result |

Do not hand-edit runtime evidence. Inspect `evaluation.log`,
`replan.log`, `integration.log`, `navigation.log`, and `last_run.log` when a
gate fails.

## 8. Clean relaunch gate

After the first evaluation run, repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k
```

Both independent runs must produce `EVALUATION_PASS`, preserve dynamic
replanning and goal completion, and stop all process groups.

## 9. Acceptance checklist

- [ ] Phase J static and runtime approval remains intact.
- [ ] Evaluator is independently launched and observation-only.
- [ ] Plan, replanning, goal, controller, odometry, and both command-boundary
      samples are real.
- [ ] `EVALUATION_PASS` is received on `/lunabot/evaluation/status`.
- [ ] Final map evidence, clean shutdown, and clean relaunch pass.

Phase K is not considered runtime-approved until two independent headless
runs produce aggregate `EVALUATION_PASS`, final map evidence, clean shutdown,
and clean relaunch after the current evaluator changes. A ROS/Gazebo run must
supply that evidence before Phase L is described as approved.
