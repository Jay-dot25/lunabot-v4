# Phase C runtime evidence

This directory is the output location for `~/launch-c`.

Generate workstation evidence with:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c
```

The launcher writes real ROS/Gazebo samples here, including `/map`,
`map -> odom`, `odom -> chassis`, `/lunabot/odom`, `/lunabot/imu`, the
before/after map-motion samples, `slam.log`, and the control/odometry reports.
Runtime files are intentionally not pre-filled in the repository. Static
validation is recorded separately by `tools/validate_phase_c.py`.

When `ros-humble-nav2-map-server` is installed, a successful final run also
writes `phase_c_map.yaml` and `phase_c_map.pgm`. Inspect `map_saver.log` for
the result. The launcher does not treat the optional map-saver package's
absence as a false runtime pass; it prints the missing-package status and
retains the OccupancyGrid samples.
