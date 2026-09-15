# Phase I Runtime Verification Checklist

Run on the ROS 2 Humble/Gazebo workstation. Static validation does not
complete this checklist.

## Static gate

- [ ] `python3 tools/validate_phase_i.py` passes.
- [ ] `~/launch-i` resolves to this checkout and is executable.

## Headless full-integration gate

Command:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i
```

- [ ] Approved Phase A-H world, bridge, TF, SLAM, perception, semantic map, cost map, and terrain planner checks pass.
- [ ] Diagnostic A* path/status remain available.
- [ ] Diagnostic A* command output is isolated from `/cmd_vel_in`.
- [ ] `/lunabot/autonomy/status` is a real `std_msgs/msg/String`.
- [ ] `/cmd_vel_in` is produced by the terrain follower and reaches the Phase B controller.
- [ ] `cmd_vel_in_motion.txt` contains a nonzero linear or angular command.
- [ ] `cmd_vel_motion.txt` contains nonzero controller output.
- [ ] `controller_boundary_status.txt` shows `ACTIVE input=/cmd_vel_in output=/cmd_vel`.
- [ ] Status contains `INTEGRATION_GOAL_REACHED`.
- [ ] Rover reaches the goal using the terrain-aware path.
- [ ] Live map updates pass and final YAML/PGM map evidence is saved.
- [ ] `PHASE I RUN COMPLETE - overall result: PASS` is printed.
- [ ] `Launch I environment cleanly closed.` is printed.

## GUI/RViz gate

Command:

```bash
EVIDENCE=1 ~/launch-i
```

- [ ] Active Terrain Path displays `/lunabot/terrain/plan`.
- [ ] Diagnostic A* Path remains visible but is not the active command source.
- [ ] The rover moves through `/cmd_vel_in -> Phase B controller -> /cmd_vel`.
- [ ] Cost map, semantic map, segmentation overlay, camera, LiDAR, odometry, and TF remain error-free.
- [ ] Ctrl+C closes all Phase I process groups.

## Clean relaunch

- [ ] Repeat the headless command after shutdown.
- [ ] One terrain follower starts on the second run.
- [ ] The second run again produces `INTEGRATION_GOAL_REACHED`.
- [ ] No stale Gazebo, bridge, follower, A*, terrain planner, cost mapper, semantic mapper, segmentation, SLAM, or controller process remains.

Phase J must not begin until this checklist is explicitly approved.
