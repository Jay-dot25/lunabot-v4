# Phase K — Testing and Evaluation (`launch-k`) — Real Metrics + Traditional vs LunaBot

## 1. Scope and gate

Phase K implements quantitative evaluation per master directive and PDF:

Metrics:
 1. Path length Σ sqrt(dx²+dy²) from odometry (total_motion)
 2. Success via goal tolerance 0.35 (distance_to_goal < tolerance or INTEGRATION_GOAL_REACHED)
 3. Collision count (entering CRATER_COST cells)
 4. Hazardous exposure ROCK+CRATER % (hazardous_distance / path_length *100)
 5. Replanning time (obstacle_detected -> new_plan from injector timestamps)
 6. Terrain cost: cumulative and average cost along trajectory
 7. Replan count (plan revisions)

Also supports Traditional (Phase D A* without terrain awareness) vs LunaBot (E-H-I) experiment with consistent start/goal/terrain.

Publishes /lunabot/evaluation/status EVALUATION_PASS with real measured values.

## 2. Data flow

```
Odom -> path length Σ sqrt(dx²+dy²)
Goal + Odom -> success via goal_tolerance 0.35
Cost Map + Odom -> hazardous exposure, collisions, avg/cumulative terrain cost
Dynamic obstacle status -> replanning time
Plan revisions -> replan count
```

## 3. Interface

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/plan` | Path | plan_seen |
| `/lunabot/autonomy/replan_status` | String | DYNAMIC_REPLAN_PASS |
| `/lunabot/autonomy/status` | String | INTEGRATION_GOAL_REACHED |
| `/lunabot/odom` | Odometry | total_motion Σ sqrt |
| `/goal_pose` | PoseStamped | goal_topic |
| `/lunabot/terrain/cost_map` | OccupancyGrid | cost_map_topic |
| `/lunabot/dynamic_obstacle/status` | String | dynamic_obstacle_topic with timestamps |
| `/lunabot/evaluation/status` | String | EVALUATION_PASS with metrics |

Evaluator params: goal_topic /goal_pose, cost_map_topic /lunabot/terrain/cost_map, dynamic_obstacle_topic /lunabot/dynamic_obstacle/status, goal_tolerance 0.35, cost_weight 2.5

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-k" ~/launch-k
chmod +x launch-k scripts/launch-k.sh scripts/phase_k_evaluator.py scripts/inject_dynamic_obstacle.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_k.py
```

Checks evaluator real metrics: path length from odometry sqrt, success via goal tolerance, hazardous exposure ROCK+CRATER, replanning time, terrain cost, goal_topic cost_map_topic dynamic_obstacle_topic, goal_tolerance 0.35, injector injection_distance 2.0 min_motion 0.1 auto_inject true, cost_weight 2.5.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k
```

Validates EVALUATION_PASS with real metrics, dynamic_obstacle.log timestamps, aggregate evaluation.

## 7. Traditional vs LunaBot experiment

Consistent start/goal/terrain:
- Traditional: launch-d A* without terrain costs
- LunaBot: launch-e..i with graded costs 5,15,45,70,100

Compare path length, hazardous %, avg cost, success. LunaBot longer low-cost beats shorter high-cost.

## 8. Evidence

evidence/phase-k-launch-k/: evaluation.log, evaluation_status.txt with path_length, success, collisions, replans, replanning_time, hazard_pct, avg_cost, total_cost, motion, dynamic_obstacle.log, etc.

## 9. Clean relaunch

Launch K environment cleanly closed. Second run.

## 10. Acceptance

- [ ] Real metrics: path length Σ sqrt(dx²+dy²) from odometry
- [ ] Success via goal tolerance 0.35
- [ ] Collision count, hazardous exposure ROCK+CRATER %, replanning time, avg/cumulative terrain cost, replan count
- [ ] Traditional vs LunaBot experiment
- [ ] Injector with timestamps
- [ ] cost_weight 2.5 graded
