# Phase B evidence — Control & Odometry

This directory is populated by a real Phase B run. Do not fabricate or edit
runtime measurements. Run from the repository:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-b
```

## Files

| File | Meaning |
|---|---|
| `last_run.log` | launch-b transcript written by the launcher |
| `gazebo.log` | Gazebo Sim log plus static-TF process output |
| `bridge.log` | ROS 2 ↔ Gazebo bridge output |
| `control.log` | Phase B command smoother/watchdog node output |
| `odometry_monitor.log` | odometry monitor node output |
| `topics.txt` | ROS topic list captured during the run |
| `control_status.txt` | live controller status sample |
| `odometry_status.txt` | live odometry-quality status sample |
| `odom_sample.txt` | one real `/lunabot/odom` message |
| `tf_odom_chassis.txt` | one real dynamic odom-to-chassis TF sample |
| `odometry_samples.csv` | every odometry message received by the monitor |
| `odometry_report.txt` | computed rate, duration, distance, continuity and PASS/FAIL |
| `demo_drive_result.txt` | real controlled drive result from `/cmd_vel_in` |
| `diag_drive.csv` | command/odometry/wheel diagnostics from the teleop demo |

The static validator report is `static_validation.txt`.
