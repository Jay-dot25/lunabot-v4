# Phase L Presentation Mission Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete the final presentation gate.

## Static gate

- [ ] `python3 tools/validate_phase_l.py` passes.
- [ ] `~/launch-l` resolves to this checkout and is executable.
- [ ] Approved Phase A-K baseline remains green.

## Manual GUI mission gate

```bash
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

- [ ] Gazebo shows the lunar habitat, rover, terrain, and physical obstacle.
- [ ] RViz opens with map, LiDAR, obstacle map, cost map, paths, and goal display.
- [ ] RViz `Set Goal` publishes a real `/goal_pose`.
- [ ] `MANUAL_GOAL_SELECTED` is observed.
- [ ] `OBSTACLE_DETECTED` is observed from the LiDAR detector.
- [ ] Obstacle cells appear in `/lunabot/obstacles/map` and the terrain cost map.
- [ ] The terrain path changes around the obstacle.
- [ ] `DYNAMIC_REPLAN_PASS` is observed.
- [ ] The rover reaches the selected goal.
- [ ] `EVALUATION_PASS` and `MISSION_DEMO_PASS` are observed.
- [ ] Nonzero `/cmd_vel_in` and `/cmd_vel` evidence passes.
- [ ] Live map updates and final YAML/PGM map evidence pass.
- [ ] `PHASE L RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch L environment cleanly closed.` is printed.

## Automated regression mode

```bash
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 HEADLESS=1 ~/launch-l
```

- [ ] The regression passes the inherited infrastructure checks.
- [ ] The regression is not counted as manual presentation approval.

## Clean relaunch

- [ ] Repeat the manual GUI mission after clean shutdown.
- [ ] The second run again produces `MISSION_DEMO_PASS`.
- [ ] No stale Gazebo, bridge, obstacle detector, evaluator, mission observer,
      replan monitor, follower, planner, SLAM, or controller process remains.

Phase L is the final phase; no later phase is defined.
