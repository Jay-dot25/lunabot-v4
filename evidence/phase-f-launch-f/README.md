# Phase F Evidence — Semantic Terrain Mapping

This directory is written by the independent `~/launch-f` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

The launcher records real Phase E segmentation inputs, semantic-map samples,
semantic status, inherited Phase D navigation evidence, and clean-shutdown
output. Static validation is stored in `static_validation.txt`; it is not a
substitute for the workstation runtime gate.

Expected runtime acceptance includes:

```text
SEMANTIC_MAP_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
PHASE F RUN COMPLETE - overall result: PASS
Launch F environment cleanly closed.
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`semantic_mapping.log`, `segmentation.log`, bridge/Gazebo logs, and
`last_run.log`, correct the runtime issue, and repeat the run.
