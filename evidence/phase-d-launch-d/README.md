# Phase D runtime evidence

This directory is populated by the independent `~/launch-d` launcher.

Run the automated gate with:

```bash
source /opt/ros/humble/setup.bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d
```

The run records real A* goal, path, planner status, map before/after navigation,
scoped LaserScan TF, odometry, IMU, and final map evidence. Runtime files are
not pre-filled. `tools/validate_phase_d.py` writes the static report separately.

A successful workstation run must report a non-empty `/plan`, `A* goal
reached: PASS`, live map updates during autonomous navigation, saved YAML/PGM
map files when `nav2_map_server` is available, and clean shutdown.
