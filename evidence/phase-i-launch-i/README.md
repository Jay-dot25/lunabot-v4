# Phase I Evidence — Full Autonomous Integration

This directory is written by the independent `~/launch-i` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-i
```

The launcher records the active terrain follower, integrated status, terrain
plan, nonzero `/cmd_vel_in` and `/cmd_vel` motion, controller-boundary status,
diagnostic A*, inherited perception/map evidence, and clean-shutdown output. Static validation is stored in
`static_validation.txt`; it is not a substitute for the workstation runtime
gate.

Expected runtime acceptance includes:

```text
INTEGRATION_GOAL_REACHED
live map updates during autonomous navigation: PASS
PHASE I RUN COMPLETE - overall result: PASS
Launch I environment cleanly closed.
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`integration.log`, `terrain_planner.log`, `navigation.log`, controller logs,
bridge/Gazebo logs, and `last_run.log`, correct the runtime issue, and repeat
the run.
