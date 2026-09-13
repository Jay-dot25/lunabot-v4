# Phase B visual and functional validation checklist

Run the automated test first, then run the interactive command for visual
checking. Tick only what was observed on the workstation.

## Automated evidence

- [ ] `EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-b` exits 0.
- [ ] Startup runtime checks show PASS for `/clock`, `/lunabot/odom`,
      `/lunabot/control/status`, `/lunabot/odometry/status`, camera, LiDAR,
      IMU, and the two TF checks.
- [ ] `demo_drive_result.txt` says `result : PASS`.
- [ ] `odometry_report.txt` says `quality_result      : PASS`.
- [ ] `odometry_samples.csv` contains real samples over nonzero simulation time.
- [ ] `diag_drive.csv` exists and contains commanded and measured values.

## Interactive Gazebo/RViz check

- [ ] The same grey, dense, uneven cratered lunar environment from Phase A is
      present; no flat replacement world is visible.
- [ ] LunaBot V4 spawns on the terrain and remains stable.
- [ ] RViz fixed frame is `odom`; the Odometry display, TF tree and LiDAR are
      visible.
- [ ] Pressing W moves the rover through `/cmd_vel_in` and the rover moves;
      the controller status reports `ACTIVE`.
- [ ] Releasing the key or waiting longer than 0.5 s causes a controlled stop;
      the status reports `WATCHDOG_STOP` when no input is present.
- [ ] S, A, D and Space behave as documented.
- [ ] Q/Ctrl+C stops Gazebo, bridge, control, monitor, TF and RViz; a second
      `~/launch-b` starts without rebooting.

## User gate

- [ ] Phase B output matches the expected control and odometry behavior.
- [ ] User approves continuation to Phase C.
