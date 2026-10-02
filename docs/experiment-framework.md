# Experiment runner and comparative reports

`config/experiments.json` defines a deterministic full-factorial matrix over
planner, held-out world seed, goal, illumination, sensor noise, and dynamic
obstacle scenario, with 30 repetitions. It compares geometric A*, semantic
weighted A*, and semantic D*-Lite.

Preview the resolved matrix without executing ROS:

```bash
python3 tools/run_experiments.py config/experiments.json --output evidence/experiments
```

Execution is deliberately opt-in:

```bash
python3 tools/run_experiments.py config/experiments.json --output evidence/experiments --execute --resume
```

Each trial receives an immutable ID and metadata directory. The runtime launch
must write `trial.json`; failed return codes remain recorded. Phase 17 supplies
the referenced integrated launch, so `--execute` is not claimed runnable before
that phase.

Summarize completed trials with:

```bash
python3 tools/summarize_experiments.py evidence/experiments
```

The tool emits JSON and tidy CSV with sample count, mean, and 95% confidence
interval per planner and metric. Final scientific conclusions require completed
held-out runs; the framework does not fabricate missing trials or results.
