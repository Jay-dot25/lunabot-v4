# Phase H Evidence — Terrain-Aware Path Planning

This directory is written by the independent `~/launch-h` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-h
```

The launcher records real terrain-aware plan/status samples, inherited cost,
semantic, segmentation, and A* evidence, and clean-shutdown output. Static
validation is stored in `static_validation.txt`; it is not a substitute for
the workstation runtime gate.

Expected runtime acceptance includes:

```text
TERRAIN_PLAN_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
PHASE H RUN COMPLETE - overall result: PASS
Launch H environment cleanly closed.
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`terrain_planner.log`, cost/semantic/segmentation logs, bridge/Gazebo logs, and
`last_run.log`, correct the runtime issue, and repeat the run.
