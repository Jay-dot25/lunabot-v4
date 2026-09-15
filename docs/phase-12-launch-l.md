# Phase L — Final Mission Demonstration (`launch-l`)

## 1. Scope and gate

Phase L is the final mission demonstration built on the explicitly approved
Phase K runtime-evaluation stack. It has an independent `~/launch-l` command,
preserves the Phase A-K topology, and adds one observation-only mission
supervisor. The supervisor does not publish velocity, goals, navigation state,
or any competing command.

The final mission gate demonstrates, from one clean launch, that the rover can
publish a terrain plan, replan after motion, complete the integrated goal,
produce the Phase K aggregate evaluation, observe the real goal and map, and
publish a retained `MISSION_DEMO_PASS`. The launcher separately verifies live
map updates, final YAML/PGM map evidence, and clean process-group shutdown.

Static validation is not mission acceptance. A successful Phase L gate needs
one real `MISSION_DEMO_PASS`, all inherited Phase K checks, final map files,
and a clean shutdown followed by a clean relaunch.

## 2. Mission data flow

```text
approved Phase A-K autonomous stack
              │
              ├── /lunabot/terrain/plan
              ├── /lunabot/autonomy/replan_status
              ├── /lunabot/autonomy/status
              ├── /lunabot/evaluation/status
              ├── /goal_pose
              └── /map
                       │
                       ▼
              Phase L mission observer
                       │
                       ▼
              /lunabot/mission/status
```

The observer requires:

- a non-empty terrain-aware plan;
- `DYNAMIC_REPLAN_PASS`;
- `INTEGRATION_GOAL_REACHED`;
- the retained Phase K `EVALUATION_PASS`;
- a real `/goal_pose` sample;
- a non-empty `/map` sample.

A successful retained status contains `MISSION_DEMO_PASS` and the observed
map update count. The mission observer is observation-only and creates no
`geometry_msgs/Twist` publisher.

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/plan` | `nav_msgs/Path` | terrain mission plan |
| `/lunabot/autonomy/replan_status` | `std_msgs/String` | dynamic replanning result |
| `/lunabot/autonomy/status` | `std_msgs/String` | integrated goal result |
| `/lunabot/evaluation/status` | `std_msgs/String` | Phase K aggregate evaluation |
| `/goal_pose` | `geometry_msgs/PoseStamped` | real mission goal sample |
| `/map` | `nav_msgs/OccupancyGrid` | mission map sample |
| `/lunabot/mission/status` | `std_msgs/String` | retained final mission result |

Retained status and map/plan inputs use reliable transient-local QoS. The goal
subscription uses the existing compatible command QoS. No second localization,
SLAM, or navigation stack is introduced.

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-l" ~/launch-l
chmod +x launch-l scripts/launch-l.sh scripts/phase_l_mission.py
```

## 5. Static validation

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_l.py
```

The validator checks the approved Phase K baseline, independent launch-l
resolution, stage numbering, mission-observer safety, retained status output,
command isolation, evidence paths, and honest runtime gating. It does not
claim that a final mission passed.

## 6. Automated final mission gate

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-l
```

The run must contain:

```text
 dynamic terrain-plan replanning: PASS
 terrain-integrated goal reached: PASS
 aggregate runtime evaluation: PASS
 final mission demonstration: PASS
 MISSION_DEMO_PASS
 nonzero /cmd_vel_in motion evidence: PASS
 controller output /cmd_vel motion evidence: PASS
 live map updates during autonomous navigation: PASS
 saved map evidence (YAML + PGM): PASS
 PHASE L RUN COMPLETE - overall result: PASS
 Launch L environment cleanly closed.
```

## 7. Evidence

Runtime evidence is written to `evidence/phase-l-launch-l/`:

| File | Meaning |
|---|---|
| `mission.log` | mission observer diagnostics |
| `mission_status.txt` | retained mission status sample |
| `mission_wait_status.txt` | live stream proving `MISSION_DEMO_PASS` |
| `evaluation_status.txt` | retained Phase K evaluation result |
| `replan_status.txt` | dynamic-replanning result |
| `terrain_plan.txt` | active terrain plan sample |
| `phase_l_map.yaml` / `phase_l_map.pgm` | final map evidence |
| `last_run.log` | ordered final-gate result |
| `static_validation.txt` | static Phase L report |

Do not hand-edit generated runtime evidence. Inspect `mission.log`,
`evaluation.log`, `replan.log`, `integration.log`, and `last_run.log` when a
gate fails.

## 8. Clean relaunch gate

After the first final mission demonstration, repeat:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-l
```

Both independent runs must produce `MISSION_DEMO_PASS`, final map evidence,
clean shutdown, and no stale Gazebo, bridge, evaluator, mission observer,
replan monitor, follower, planner, SLAM, or controller process.

## 9. Acceptance checklist

- [ ] Phase K static and runtime approval remains intact.
- [ ] Launch L is independent and preserves the approved command topology.
- [ ] Mission observer is observation-only and retained-status safe.
- [ ] Real plan, replan, goal, evaluation, goal-pose, and map evidence pass.
- [ ] `MISSION_DEMO_PASS` is received on `/lunabot/mission/status`.
- [ ] Final YAML/PGM map evidence passes.
- [ ] Two independent runs, clean shutdown, and clean relaunch pass.

Phase L is the final phase; no later phase is defined.
