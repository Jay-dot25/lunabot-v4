# Phase J Evidence — Dynamic Replanning

This directory is written by the independent `~/launch-j` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-j
```

The launcher preserves the approved Phase I stack and records changed
terrain-plan revisions observed after real odometry motion. Static validation
is stored in `static_validation.txt`; it is not a substitute for the
workstation runtime gate.

Expected runtime acceptance includes:

```text
DYNAMIC_REPLAN_PASS
terrain-integrated goal reached: PASS
PHASE J RUN COMPLETE - overall result: PASS
Launch J environment cleanly closed.
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`replan.log`, `terrain_planner.log`, `integration.log`, `navigation.log`, and
`last_run.log`.
