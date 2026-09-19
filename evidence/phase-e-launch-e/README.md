# Phase E Evidence — RGB-D Terrain Perception

This directory is written by the independent `~/launch-e` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-e
```

The launcher records live RGB-D five-class MLP output samples, exact per-class
status, Phase D navigation evidence, and clean-shutdown output. Existing files
from the obsolete three-class heuristic are not acceptance evidence for this
replacement and must be regenerated. Static validation is
stored in `static_validation.txt`; it is not a substitute for the workstation
runtime gate.

Expected runtime acceptance includes:

```text
SEGMENTATION_PASS
A* goal reached: PASS
live map updates during autonomous navigation: PASS
PHASE E RUN COMPLETE - overall result: PASS
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`segmentation.log`, `bridge.log`, `gazebo.log`, and `last_run.log`, correct the
runtime issue, and repeat the run.
