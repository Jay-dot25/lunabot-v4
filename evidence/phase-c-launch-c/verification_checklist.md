# Phase C workstation verification checklist

Run from a sourced ROS 2 Humble terminal:

```bash
source /opt/ros/humble/setup.bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c
```

## Automated gate

- [ ] Gazebo reports `simulation unpaused`.
- [ ] `/lunabot/lidar/scan` receives a LaserScan.
- [ ] `/lunabot/odom` receives real DiffDrive odometry.
- [ ] `/lunabot/imu` receives a real Imu message.
- [ ] `/map` receives a real OccupancyGrid.
- [ ] `map -> odom` lookup passes from slam_toolbox.
- [ ] `odom -> chassis` and sensor-frame lookups pass.
- [ ] `sensor_head -> lunabot_v4/sensor_head/lidar` passes for the Gazebo scan frame.
- [ ] `live map updates during rover motion: PASS`.
- [ ] `demo_drive_result.txt` reports `result : PASS`.
- [ ] `odometry_report.txt` reports quality PASS and real samples.
- [ ] Final output reports `PHASE C RUN COMPLETE - overall result: PASS`.

## Visual gate

Run `EVIDENCE=1 ~/launch-c` after the headless gate and inspect RViz:

- [ ] `SLAM Map` topic is `/map`.
- [ ] LaserScan topic is `/lunabot/lidar/scan`, with Best Effort QoS.
- [ ] Camera topic is `/lunabot/camera/image_raw`, with Best Effort QoS.
- [ ] TF tree shows `map -> odom -> chassis` and sensor frames.
- [ ] Odometry display is `/lunabot/odom`.
- [ ] The rover moves through the controller while the map changes.
- [ ] No Camera or LaserScan topic errors appear.

## Shutdown and relaunch

- [ ] Q or Ctrl+C stops the teleop, controller, monitor, slam_toolbox,
      bridge, static TF, RViz, and Gazebo process group.
- [ ] A second `EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c` run passes.
- [ ] `phase_c_map.yaml` and `.pgm` exist if `nav2_map_server` is installed
      and the map saver succeeds.

Record the date, Ubuntu/ROS/Gazebo versions, terminal output, and screenshot
alongside the generated files before requesting Phase C approval. Do not start
Phase D until every required item above is confirmed.
