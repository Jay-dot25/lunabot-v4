# Phase E Runtime Verification Checklist

Run only on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_e.py` passes.
- [ ] `~/launch-e` resolves to this checkout and is executable.

## Headless RGB-D segmentation gate

Command:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

- [ ] Existing Phase A-D world, bridge, TF, controller, odometry, SLAM, and A* checks pass.
- [ ] `/lunabot/depth/image_raw` is a real `sensor_msgs/msg/Image`.
- [ ] `/lunabot/terrain/segmentation` is a real `sensor_msgs/msg/Image`.
- [ ] `/lunabot/terrain/overlay` is a real `sensor_msgs/msg/Image`.
- [ ] `/lunabot/terrain/segmentation/status` is a real `std_msgs/msg/String`.
- [ ] Segmentation status contains `SEGMENTATION_PASS`.
- [ ] Mask and overlay evidence files are non-empty.
- [ ] `A* goal reached: PASS` remains true.
- [ ] Live map updates and final map evidence pass when map saver is installed.
- [ ] `PHASE E RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch E environment cleanly closed.` is printed.

## GUI/RViz gate

Command:

```bash
EVIDENCE=1 ~/launch-e
```

- [ ] Terrain Segmentation Overlay displays `/lunabot/terrain/overlay`.
- [ ] Terrain Segmentation Mask can be enabled on `/lunabot/terrain/segmentation`.
- [ ] Camera, LiDAR, map, A* path, odometry, and TF remain error-free.
- [ ] Ctrl+C closes all Phase E process groups.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One perception node starts on the second run.
- [ ] The second run again produces `SEGMENTATION_PASS`.
- [ ] No stale Gazebo, bridge, perception, SLAM, planner, or controller process remains.

Phase F must not begin until this checklist is explicitly approved.
