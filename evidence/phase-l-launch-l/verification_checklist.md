# Phase L Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_l.py` passes.
- [ ] `~/launch-l` resolves to this checkout and is executable.
- [ ] Phase K remains statically and runtime approved.

## Final mission gate

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-l
```

- [ ] Phase A-K world, bridge, TF, SLAM, perception, maps, replanning,
      evaluation, and follower checks pass.
- [ ] `/lunabot/mission/status` is a real `std_msgs/msg/String`.
- [ ] A non-empty terrain plan is observed.
- [ ] `DYNAMIC_REPLAN_PASS` is observed.
- [ ] `INTEGRATION_GOAL_REACHED` is observed.
- [ ] Retained `EVALUATION_PASS` is observed.
- [ ] A real `/goal_pose` sample is observed.
- [ ] A non-empty `/map` sample is observed.
- [ ] Status contains `MISSION_DEMO_PASS`.
- [ ] Nonzero `/cmd_vel_in` and `/cmd_vel` evidence passes.
- [ ] Live map updates and final YAML/PGM map evidence pass.
- [ ] `PHASE L RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch L environment cleanly closed.` is printed.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One mission observer starts on the second run.
- [ ] The second run again produces `MISSION_DEMO_PASS`.
- [ ] No stale Gazebo, bridge, evaluator, mission observer, replan monitor,
      follower, A*, planner, cost mapper, semantic mapper, segmentation, SLAM,
      or controller process remains.

Phase L is the final phase; no later phase is defined.
