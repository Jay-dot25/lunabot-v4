# Phase L Evidence — Final Mission Demonstration

This directory is written by the independent `~/launch-l` final mission gate.

Run the gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-l
```

The mission observer watches the approved Phase K aggregate result together
with the real terrain plan, dynamic replanning, integrated goal completion,
goal publication, and map output. It publishes `MISSION_DEMO_PASS` only after
all mission criteria are present. It is observation-only and never publishes
velocity or a goal.

Expected runtime acceptance includes:

```text
MISSION_DEMO_PASS
PHASE L RUN COMPLETE - overall result: PASS
Launch L environment cleanly closed.
```

Static validation is stored in `static_validation.txt`; it is not a substitute
for the workstation runtime gate. Do not hand-edit generated runtime evidence.
If a check fails, inspect `mission.log`, `evaluation.log`, `replan.log`,
`integration.log`, and `last_run.log`.
