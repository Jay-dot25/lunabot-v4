# Phase J Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_j.py` passes.
- [ ] `~/launch-j` resolves to this checkout and is executable.
- [ ] Phase I remains statically and runtime approved.

## Headless dynamic-replanning gate

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j
```

- [ ] Phase A-I world, bridge, TF, SLAM, perception, maps, cost map, and follower checks pass.
- [ ] Diagnostic A* remains path/status compatible and isolated from `/cmd_vel_in`.
- [ ] `/lunabot/autonomy/replan_status` is a real `std_msgs/msg/String`.
- [ ] At least two distinct terrain-plan revisions are observed.
- [ ] Revisions are observed after real odometry motion.
- [ ] Status contains `DYNAMIC_REPLAN_PASS`.
- [ ] Terrain follower reaches the goal.
- [ ] Nonzero `/cmd_vel_in` and `/cmd_vel` evidence passes.
- [ ] Live map updates and final YAML/PGM map evidence pass.
- [ ] `PHASE J RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch J environment cleanly closed.` is printed.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One dynamic monitor starts on the second run.
- [ ] The second run again produces `DYNAMIC_REPLAN_PASS`.
- [ ] No stale Gazebo, bridge, monitor, follower, A*, planner, cost mapper,
      semantic mapper, segmentation, SLAM, or controller process remains.

Phase K must not begin until this checklist is explicitly approved.
