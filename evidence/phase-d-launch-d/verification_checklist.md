# Phase D workstation verification checklist

```bash
source /opt/ros/humble/setup.bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d
```

## Automated gate

- [ ] Gazebo explicitly unpauses.
- [ ] `/map` is `nav_msgs/msg/OccupancyGrid`.
- [ ] `map -> odom`, `odom -> chassis`, and scoped LaserScan TF pass.
- [ ] `/goal_pose` receives a real goal.
- [ ] `/plan` receives a non-empty `nav_msgs/Path`.
- [ ] `/lunabot/navigation/status` reports planning/following.
- [ ] `/cmd_vel_in` is published by A* and passes through the controller.
- [ ] The rover reaches the automatic goal.
- [ ] Live map updates during autonomous navigation pass.
- [ ] IMU, Camera, LaserScan, odometry, and control checks pass.
- [ ] Final map YAML/PGM files are saved.
- [ ] `PHASE D RUN COMPLETE - overall result: PASS` is printed.

## Visual gate

Run:

```bash
EVIDENCE=1 ~/launch-d
```

- [ ] RViz fixed frame is `map`.
- [ ] SLAM Map uses `/map`.
- [ ] A* Path uses `/plan` and is visible.
- [ ] LaserScan uses `/lunabot/lidar/scan` with no error.
- [ ] Camera uses `/lunabot/camera/image_raw` with no error.
- [ ] TF shows `map -> odom -> chassis` and the scoped lidar frame.
- [ ] The rover follows the visible path.
- [ ] Ctrl+C shuts down the full process group cleanly.

## Relaunch gate

- [ ] A second `EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d` run passes.
- [ ] No old Gazebo, SLAM, navigation, control, or monitor processes remain.
- [ ] Do not begin Phase E until every required item is confirmed.
