# Phase L — Final Mission Demonstration (`launch-l`) — J+K+goal+obstacle+semantic nav

## 1. Scope and gate

Phase L final mission combines J+K+goal+obstacle detection+semantic navigation with real measured values and reproducibility.

Combines:
- Phase J controlled dynamic obstacle injection (inject_dynamic_obstacle.py with timestamps obstacle_introduced/detected/cost_changed/new_plan, injection_distance 2.0 min_motion 0.1 auto_inject true)
- Phase K real metrics (path length Σ sqrt(dx²+dy²) from odometry, success via goal tolerance 0.35, collision count, hazardous exposure ROCK+CRATER %, replanning time, avg/cumulative terrain cost, replan count)
- Goal detection, obstacle detection, semantic navigation (5-class BEDROCK 0 REGOLITH 1 ROCK 2 CRATER 3 SHADOW 4, graded costs BEDROCK=5 REGOLITH=15 SHADOW=45 ROCK=70 CRATER=100, cost_weight 2.5)
- Traditional vs LunaBot experiment consistency
- Reproducibility: 2 independent GUI runs with FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true

Mission observer phase_l_mission.py observes terrain plan, replanning, integrated goal, aggregate evaluation, manual goal, obstacle status, real goal, map, with topics dynamic_obstacle_topic cost_map_topic semantic_map_topic.

Publishes /lunabot/mission/status MISSION_DEMO_PASS.

## 2. Data flow

```
Injector (controlled obstacle) -> Gazebo -> sensors -> E/F/G (5-class graded) -> H (weighted A* cost_weight 2.5) -> I (follower -> /cmd_vel_in -> controller -> /cmd_vel) -> K evaluator (real metrics) -> L mission observer (MISSION_DEMO_PASS)
```

## 3. Interface

| Interface | Type |
|---|---|
| `/lunabot/terrain/plan` | Path |
| `/lunabot/terrain/cost_map` | OccupancyGrid graded 5,15,45,70,100 cost_map_topic |
| `/lunabot/terrain/semantic_map` | OccupancyGrid 5-class semantic_map_topic |
| `/lunabot/dynamic_obstacle/status` | String dynamic_obstacle_topic with timestamps |
| `/lunabot/obstacles/status` | String OBSTACLE_DETECTED |
| `/goal_pose` | PoseStamped goal_topic |
| `/lunabot/evaluation/status` | String EVALUATION_PASS |
| `/lunabot/mission/status` | String MISSION_DEMO_PASS |

Params: goal_topic /goal_pose, cost_map_topic /lunabot/terrain/cost_map, dynamic_obstacle_topic /lunabot/dynamic_obstacle/status, semantic_map_topic /lunabot/terrain/semantic_map, goal_tolerance 0.35, cost_weight 2.5, injection_distance 2.0 min_motion 0.1 auto_inject true

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-l" ~/launch-l
chmod +x launch-l scripts/launch-l.sh scripts/phase_l_mission.py scripts/phase_k_evaluator.py scripts/inject_dynamic_obstacle.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_l.py
```

Checks injector path/PID/shutdown, cost_weight 2.5, injector startup block, evaluator expanded goal_topic cost_map_topic dynamic_obstacle_topic goal_tolerance 0.35, mission observer expanded dynamic_obstacle_topic cost_map_topic semantic_map_topic, no cmd_vel, 23 stages, real metrics.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-l
```

For final manual mission:

```bash
FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true EVIDENCE=1 ~/launch-l
```

And reproducibility second run same command.

Must validate:
- MISSION_DEMO_PASS
- EVALUATION_PASS with real metrics
- DYNAMIC_REPLAN_PASS
- OBSTACLE_DETECTED
- obstacle_introduced/detected/cost_changed/new_plan timestamps in dynamic_obstacle.log
- 5-class graded costs
- FINAL_DEMO mode

## 7. Traditional vs LunaBot experiment

Consistent start/goal/terrain, compare Traditional (D) vs LunaBot (E-H-I) path length, hazardous %, avg cost.

## 8. Evidence

evidence/phase-l-launch-l/: mission.log, mission_status.txt, evaluation_status.txt with real measured values, dynamic_obstacle.log with timestamps, cost_map_sample.txt, semantic_map_sample.txt, map evidence, static_validation.txt

## 9. Reproducibility

2 independent GUI runs with FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true, both must achieve MISSION_DEMO_PASS.

## 10. Acceptance

- [ ] J+K+goal+obstacle detection+semantic nav combined
- [ ] Real measured values: path length Σ sqrt(dx²+dy²), success via goal tolerance 0.35, collisions, hazardous ROCK+CRATER %, replanning time, avg/cumulative terrain cost, replan count
- [ ] Controlled obstacle injection with timestamps obstacle_introduced/detected/cost_changed/new_plan
- [ ] 5-class BEDROCK=0 REGOLITH=1 ROCK=2 CRATER=3 SHADOW=4, graded costs 5,15,45,70,100, cost_weight 2.5
- [ ] Reproducibility 2 independent GUI runs FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true
- [ ] No game interface yet, stops after L
