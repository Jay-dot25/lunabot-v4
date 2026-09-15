# Phase F Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_f.py` passes.
- [ ] `~/launch-f` resolves to this checkout and is executable.

## Headless semantic-map gate

Command:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

- [ ] Approved Phase A-E world, bridge, TF, controller, odometry, SLAM, segmentation, and A* checks pass.
- [ ] `/lunabot/terrain/semantic_map` is a real `nav_msgs/msg/OccupancyGrid`.
- [ ] `/lunabot/terrain/semantic_map/status` is a real `std_msgs/msg/String`.
- [ ] Semantic map contains `info` and `data` fields.
- [ ] Semantic status contains `SEMANTIC_MAP_PASS`.
- [ ] Terrain and obstacle cell statistics are present.
- [ ] `A* goal reached: PASS` remains true.
- [ ] Live map updates and final map evidence pass when map saver is installed.
- [ ] `PHASE F RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch F environment cleanly closed.` is printed.

## GUI/RViz gate

Command:

```bash
EVIDENCE=1 ~/launch-f
```

- [ ] Semantic Terrain Map displays `/lunabot/terrain/semantic_map`.
- [ ] Phase E segmentation overlay, camera, LiDAR, map, A* path, odometry, and TF remain error-free.
- [ ] Ctrl+C closes all Phase F process groups.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One semantic mapper starts on the second run.
- [ ] The second run again produces `SEMANTIC_MAP_PASS`.
- [ ] No stale Gazebo, bridge, mapper, segmentation, SLAM, planner, or controller process remains.

Phase G must not begin until this checklist is explicitly approved.
