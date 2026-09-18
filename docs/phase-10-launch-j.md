# Phase J — Dynamic Replanning with Controlled Obstacle Injection (`launch-j`)

## 1. Scope and gate

Phase J demonstrates compelling dynamic obstacle handling:

 ROVER MOVING
  -> NEW OBSTACLE APPEARS
  -> CURRENT PATH BECOMES UNSAFE
  -> COST MAP CHANGES
  -> NEW PLAN GENERATED
  -> ROVER TAKES NEW ROUTE

Implements controlled injection via scripts/inject_dynamic_obstacle.py:

- Waits for navigation (plan + goal + motion)
- Finds injection point ahead on route (injection_distance 2.0)
- Introduces physical Gazebo obstacle via /world/lunar_world/create
- Sensors observe (LiDAR + RGB-D) -> Phase E/F/G propagate -> Phase H generates changed path

Captures timestamps:
 obstacle_introduced
 obstacle_detected
 cost_changed (cost_map_changed)
 new_plan (new_plan_generated)
 replanning_time

Publishes /lunabot/dynamic_obstacle/status

## 2. Data flow

```
Plan /lunabot/terrain/plan + Goal + Odom + Cost Map -> injector -> Gazebo spawn -> LiDAR/RGB-D -> semantic -> cost -> new plan
DynamicReplanMonitor observes revisions -> DYNAMIC_REPLAN_PASS
```

## 3. Interface

| Interface | Type |
|---|---|
| `/lunabot/terrain/plan` | Path |
| `/lunabot/terrain/cost_map` | OccupancyGrid graded |
| `/lunabot/obstacles/status` | String OBSTACLE_DETECTED |
| `/lunabot/dynamic_obstacle/status` | String DYNAMIC_OBSTACLE_INJECTED + timestamps |
| `/lunabot/autonomy/replan_status` | String DYNAMIC_REPLAN_PASS |

Injector params: injection_distance 2.0, min_motion 0.1, auto_inject true, cost_weight 2.5

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-j" ~/launch-j
chmod +x launch-j scripts/launch-j.sh scripts/dynamic_replan_monitor.py scripts/inject_dynamic_obstacle.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_j.py
```

Checks injector ROS node, controlled injection, timestamps obstacle_introduced/detected/cost_changed/new_plan, obstacle status, no cmd_vel, injector topics plan/goal/odom/cost_map/obstacle_status/planner_status/status, injection_distance 2.0 min_motion 0.1 auto_inject true, cost_weight 2.5, INJECTOR_PATH/PID/shutdown.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j
```

Validates DYNAMIC_REPLAN_PASS, dynamic_obstacle.log with timestamps.

## 7. GUI

EVIDENCE=1 ~/launch-j shows active terrain path and cost map, obstacle injection visible.

## 8. Evidence

evidence/phase-j-launch-j/: replan.log, replan_status.txt, dynamic_obstacle.log with introduced/detected/cost_changed/new_plan timestamps, map evidence.

## 9. Clean relaunch

Launch J environment cleanly closed. Second run.

## 10. Acceptance

- [ ] Controlled dynamic obstacle injection via scripts/inject_dynamic_obstacle.py
- [ ] Timestamps obstacle_introduced/detected/cost_changed/new_plan
- [ ] Real causal evidence, not fake DYNAMIC_REPLAN_PASS
- [ ] cost_weight 2.5 graded
