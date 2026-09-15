# Phase H Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_h.py` passes.
- [ ] `~/launch-h` resolves to this checkout and is executable.

## Headless terrain-aware planning gate

Command:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h
```

- [ ] Approved Phase A-G world, bridge, TF, controller, odometry, SLAM, perception, semantic map, cost map, and A* checks pass.
- [ ] `/lunabot/terrain/plan` is a real `nav_msgs/msg/Path`.
- [ ] `/lunabot/terrain/planner/status` is a real `std_msgs/msg/String`.
- [ ] Terrain-aware plan is non-empty.
- [ ] Planner status contains `TERRAIN_PLAN_PASS`.
- [ ] Weighted cost and path-cell statistics are present.
- [ ] `A* goal reached: PASS` remains true.
- [ ] Live map updates and final map evidence pass when map saver is installed.
- [ ] `PHASE H RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch H environment cleanly closed.` is printed.

## GUI/RViz gate

Command:

```bash
EVIDENCE=1 ~/launch-h
```

- [ ] Terrain-aware Plan displays `/lunabot/terrain/plan`.
- [ ] Cost map, semantic map, segmentation overlay, camera, LiDAR, map, A* path, odometry, and TF remain error-free.
- [ ] Actual rover motion remains behind the approved A* and Phase B controller path.
- [ ] Ctrl+C closes all Phase H process groups.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One terrain-aware planner starts on the second run.
- [ ] The second run again produces `TERRAIN_PLAN_PASS`.
- [ ] No stale Gazebo, bridge, planner, cost mapper, semantic mapper, segmentation, SLAM, A*, or controller process remains.

Phase I must not begin until this checklist is explicitly approved.
