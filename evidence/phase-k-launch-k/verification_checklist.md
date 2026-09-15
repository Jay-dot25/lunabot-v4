# Phase K Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_k.py` passes.
- [ ] `~/launch-k` resolves to this checkout and is executable.
- [ ] Phase J remains statically and runtime approved.

## Headless evaluation gate

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k
```

- [ ] Phase A-J world, bridge, TF, SLAM, perception, maps, replanning, and follower checks pass.
- [ ] `/lunabot/evaluation/status` is a real `std_msgs/msg/String`.
- [ ] A non-empty terrain plan is evaluated.
- [ ] `DYNAMIC_REPLAN_PASS` is evaluated.
- [ ] `INTEGRATION_GOAL_REACHED` is evaluated.
- [ ] Controller status is active.
- [ ] Nonzero `/cmd_vel_in` and `/cmd_vel` evidence is evaluated.
- [ ] Real odometry motion exceeds the evaluation threshold.
- [ ] Status contains `EVALUATION_PASS`.
- [ ] Live map updates and final YAML/PGM map evidence pass.
- [ ] `PHASE K RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch K environment cleanly closed.` is printed.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One evaluator starts on the second run.
- [ ] The second run again produces `EVALUATION_PASS`.
- [ ] No stale Gazebo, bridge, evaluator, replan monitor, follower, A*,
      planner, cost mapper, semantic mapper, segmentation, SLAM, or controller
      process remains.

Phase L must not begin until this checklist is explicitly approved.
