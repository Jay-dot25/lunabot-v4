# Phase H — Terrain-Aware Weighted A* (`launch-h`) — Graded cost

## 1. Scope and gate

Phase H consumes Phase G graded cost map (BEDROCK 5, REGOLITH 15, SHADOW 45, ROCK 70, CRATER 100) and produces cost-aware path optimizing distance + terrain cost:

 step_cost = distance * (1 + cost_weight * cell_cost/100)
 cost_weight=2.5

Goal: longer low-cost route through BEDROCK/REGOLITH beats shorter high-cost through ROCK/CRATER. Proven via weighted A*.

Publishes:
 /lunabot/terrain/plan (Path)
 /lunabot/terrain/planner/status (TERRAIN_PLAN_PASS with avg_cost, path_length, graded legend)

Does NOT publish velocity.

## 2. Data flow

```
/lunabot/terrain/cost_map (graded) -> terrain_aware_planner (weighted A*) -> /lunabot/terrain/plan
/goal_pose + TF map->chassis + /lunabot/odom
```

## 3. Interface

| Interface | Type |
|---|---|
| `/lunabot/terrain/cost_map` | OccupancyGrid graded |
| `/goal_pose` | PoseStamped |
| `/lunabot/terrain/plan` | Path cost-aware |
| `/lunabot/terrain/planner/status` | String TERRAIN_PLAN_PASS |

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-h" ~/launch-h
chmod +x launch-h scripts/launch-h.sh scripts/terrain_aware_planner.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_h.py
```

Checks weighted A* heapq cost_weight, blocked_cost, eight-connected, avg_cost, graded legend, cost_weight 2.5, no cmd_vel.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h
```

Must validate terrain plan content TERRAIN_PLAN_PASS and cost_weight 2.5.

## 7. GUI

EVIDENCE=1 ~/launch-h shows Terrain-aware Plan.

## 8. Evidence

evidence/phase-h-launch-h/: terrain_planner.log, terrain_plan.txt, terrain_planner_status.txt with avg_cost and graded costs.

## 9. Clean relaunch

Launch H environment cleanly closed. Second run.

## 10. Acceptance

- [ ] Weighted A* minimizing distance+terrain cost
- [ ] Proof longer low-cost beats shorter high-cost via cost_weight 2.5
- [ ] Graded costs preserved

## Habitat and Cost Contract (per master directive)

World habitat: cylindrical base radius 2.2m height 2.3m + dome sphere radius 2.2m + airlock box 1.4x1.2x1.8m, collision+visual, scene ambient 0.12 0.12 0.14 background 0.01 0.01 0.02 shadows false, 6-10 rock props (8 present: presentation_rock_01..08).

Graded cost map per directive: BEDROCK=5 REGOLITH=15 SHADOW=45 ROCK=70 CRATER=100, UNKNOWN=80 conservative, cost_weight 2.5, inflation around ROCK/CRATER.

5-class segmentation: BEDROCK=0 REGOLITH=1 ROCK=2 CRATER=3 SHADOW=4 with OVERLAY_COLORS and lightweight DL model RGB+Depth.
