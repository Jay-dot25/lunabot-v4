# Phase L — Final Mission Demonstration (`launch-l`)

## 1. Revised presentation target

The final project demonstration is a GUI mission, not only a headless
regression. It must show a Gazebo lunar habitat scene beside RViz and let the
operator select the rover's destination. The rover must sense a real obstacle
in front of it, update the obstacle/cost map, replan around it, and reach the
selected goal.

The final screen should contain:

- Gazebo: lunar terrain, habitat structures, rover, and physical obstacles;
- RViz: SLAM map, LiDAR returns, sensed obstacle map, cost map, goal marker,
  terrain-aware path, and active replanned path;
- visible status/evidence for goal selection, obstacle detection, replanning,
  and goal completion.

Phase L preserves the approved Phase A-K control, SLAM, mapping, planning,
replanning, evaluation, and shutdown infrastructure. It adds a real LiDAR
obstacle detector and a retained final mission supervisor. Both are
observation/perception or status nodes and do not publish velocity.

## 2. Final modes

### Manual presentation mode

This is the acceptance mode shown to a person:

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

Gazebo and RViz open. In RViz, select the `Set Goal` tool, click a destination
on the map, and drag to choose the desired final heading. The rover then
plans and drives to that manually selected goal.

### Automated regression mode

This mode is retained for repeatable infrastructure testing. It deliberately
uses an automatic test goal and does not replace the manual presentation gate:

```bash
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 HEADLESS=1 ~/launch-l
```

## 3. Data flow

```text
Gazebo lunar habitat + physical obstacle
              │
              ├── /lunabot/lidar/scan
              ├── /lunabot/depth/image_raw
              └── /lunabot/camera/image_raw
                       │
                       ▼
              LiDAR obstacle detector
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
 /lunabot/obstacles/status  /lunabot/obstacles/map
          │                         │
          └────────────┬────────────┘
                       ▼
              terrain cost map overlay
                       │
                       ▼
              terrain-aware planner
                       │
                       ▼
       /cmd_vel_in -> controller -> /cmd_vel

RViz Set Goal -> /goal_pose -> planner/follower
```

The physical forward obstacle is part of the Gazebo world. It is not a
synthetic status message. The detector observes real LaserScan returns,
publishes a map-frame obstacle overlay, and reports distance/bearing evidence.
The cost mapper incorporates those cells and inflates them before planning.

## 4. Mission acceptance contract

The retained `/lunabot/mission/status` publisher requires:

- a non-empty terrain plan;
- `DYNAMIC_REPLAN_PASS`;
- `INTEGRATION_GOAL_REACHED`;
- retained Phase K `EVALUATION_PASS`;
- a real `/goal_pose` sample;
- a real `/map` sample;
- `OBSTACLE_DETECTED` from the LiDAR detector;
- `MANUAL_GOAL_SELECTED` in manual presentation mode.

A successful result contains:

```text
MISSION_DEMO_PASS plan=1 replan=1 goal=1 evaluation=1 \
  goal_pose=1 map=1 obstacle=1 manual_goal=1
```

The automated regression mode sets `REQUIRE_MANUAL_GOAL=false` only so it can
exercise the rest of the stack without an operator. The final presentation
must use manual mode.

## 5. RViz presentation configuration

`rviz/phase_l.rviz` provides:

- `map` fixed frame;
- SLAM map;
- LiDAR scan;
- sensed obstacle map;
- forward obstacle markers;
- terrain semantic and cost maps;
- selected goal pose;
- diagnostic A* path;
- terrain-aware and active paths;
- camera and terrain overlay views;
- RViz `Set Goal` tool publishing `/goal_pose`.

## 6. Validation

Static validation:

```bash
python3 tools/validate_phase_l.py
```

Manual final mission gate:

```bash
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

The run must contain:

```text
manual RViz goal selection: PASS
obstacle detector content: PASS
OBSTACLE_DETECTED
DYNAMIC_REPLAN_PASS
final mission demonstration: PASS
MISSION_DEMO_PASS
saved map evidence (YAML + PGM): PASS
PHASE L RUN COMPLETE - overall result: PASS
Launch L environment cleanly closed.
```

The final gate must be repeated after a clean shutdown. Two independent GUI
runs are required for final presentation approval. Automated headless runs are
useful regression checks but do not substitute for the manual-goal gate.

## 7. Evidence

Runtime evidence is written to `evidence/phase-l-launch-l/`:

| File | Meaning |
|---|---|
| `obstacles.log` | real LiDAR obstacle detector diagnostics |
| `obstacle_status.txt` | detected/clear obstacle status |
| `obstacle_map.txt` | sensed obstacle map sample |
| `mission.log` | final mission observer diagnostics |
| `mission_status.txt` | retained mission status sample |
| `mission_wait_status.txt` | live `MISSION_DEMO_PASS` stream |
| `goal_selection_wait_status.txt` | real manual goal sample |
| `evaluation_status.txt` | aggregate Phase K evaluation |
| `replan_status.txt` | dynamic replanning result |
| `phase_l_map.yaml` / `phase_l_map.pgm` | final map evidence |
| `static_validation.txt` | static Phase L report |
| `last_run.log` | ordered launcher result |

Do not hand-edit runtime evidence. Inspect `obstacles.log`, `mission.log`,
`evaluation.log`, `replan.log`, `integration.log`, and `last_run.log` when a
gate fails.

## 8. Acceptance checklist

- [ ] Habitat scene, rover, physical obstacles, and lunar terrain are visible in Gazebo.
- [ ] RViz provides a working Set Goal tool and displays the selected goal.
- [ ] Manual goal is selected with `AUTO_GOAL=false`.
- [ ] Real LiDAR obstacle detection reports a forward obstacle.
- [ ] Obstacle cells appear in the obstacle/cost map.
- [ ] The terrain path changes around the sensed obstacle.
- [ ] Rover reaches the manually selected goal.
- [ ] Controller boundary and odometry evidence pass.
- [ ] Final YAML/PGM map evidence passes.
- [ ] Two independent GUI runs and clean relaunch pass.

Phase L is the final phase; no later phase is defined.
