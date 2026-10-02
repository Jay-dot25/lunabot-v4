# Incremental D*-Lite planning

The packaged planner consumes only `/lunabot/traversability/map` and publishes
`/lunabot/planning/path`, typed `PlannerStatus`, and typed `ReplanEvent`.
Unlike the retained weighted-A* baseline, it preserves `g`, `rhs`, `km`, queue
keys, and changed-cell state. Map callbacks compare revisions, call
`update_vertex` only for changed cells and their neighbors, then invoke
`compute_shortest_path` to repair the existing solution.

```bash
ros2 run lunabot_planning dstar_lite_planner
```

Inputs are the production traversability map, `/lunabot/localization/pose`, and
`/lunabot/planning/goal`. A map geometry change deliberately rebuilds planner
state; ordinary cell changes are incremental. Costs >=100 are impassable.
`scripts/terrain_aware_planner.py` remains the weighted-A* comparison baseline.
