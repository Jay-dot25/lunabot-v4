# Phase K Evidence — Testing and Evaluation

This directory is written by the independent `~/launch-k` runtime gate.

Run the evidence gate from a sourced ROS 2 Humble terminal:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-k
```

The evaluator observes the approved Phase J plan, dynamic-replanning result,
goal completion, controller boundary, real odometry, and motion commands. It
publishes `EVALUATION_PASS` only after all required metrics are present.
Static validation is stored in `static_validation.txt`; it is not a substitute
for the workstation runtime gate.

Expected runtime acceptance includes:

```text
EVALUATION_PASS
PHASE K RUN COMPLETE - overall result: PASS
Launch K environment cleanly closed.
```

Do not hand-edit generated runtime evidence. If a check fails, inspect
`evaluation.log`, `replan.log`, `integration.log`, `navigation.log`, and
`last_run.log`.
