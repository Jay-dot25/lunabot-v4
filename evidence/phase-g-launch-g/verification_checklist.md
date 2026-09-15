# Phase G Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_g.py` passes.
- [ ] `~/launch-g` resolves to this checkout and is executable.

## Headless cost-map gate

Command:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g
```

- [ ] Approved Phase A-F world, bridge, TF, controller, odometry, SLAM, perception, semantic map, and A* checks pass.
- [ ] `/lunabot/terrain/cost_map` is a real `nav_msgs/msg/OccupancyGrid`.
- [ ] `/lunabot/terrain/cost_map/status` is a real `std_msgs/msg/String`.
- [ ] Cost map contains `info` and `data` fields.
- [ ] Cost status contains `COST_MAP_PASS`.
- [ ] Terrain, unknown, obstacle, and inflated-cell statistics are present.
- [ ] `A* goal reached: PASS` remains true.
- [ ] Live map updates and final map evidence pass when map saver is installed.
- [ ] `PHASE G RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch G environment cleanly closed.` is printed.

## GUI/RViz gate

Command:

```bash
EVIDENCE=1 ~/launch-g
```

- [ ] Terrain Cost Map displays `/lunabot/terrain/cost_map`.
- [ ] Semantic map, segmentation overlay, camera, LiDAR, map, A* path, odometry, and TF remain error-free.
- [ ] Ctrl+C closes all Phase G process groups.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One cost mapper starts on the second run.
- [ ] The second run again produces `COST_MAP_PASS`.
- [ ] No stale Gazebo, bridge, cost mapper, semantic mapper, segmentation, SLAM, planner, or controller process remains.

Phase H must not begin until this checklist is explicitly approved.
