# Final bringup and presentation

The unified entry point is:

```bash
ros2 launch lunabot_bringup lunabot_goal.launch.py mode:=simulation
```

Modes are `simulation`, `bag_replay`, and `evaluation`. Bag replay additionally
requires `bag_path:=...`. A real exported model requires `model_path`,
`model_config`, and `model_checksum`. `scripts/run_final_demo.sh` exposes the
same values through environment variables.

The launch starts inference, semantic fusion, traversability, D*-Lite, replan
verification, regulated following, safety ownership, mission metrics, and RViz.
Simulation also includes Gazebo; replay invokes `ros2 bag play --clock`.
Evaluation changes orchestration only: simulator ground-truth labels and pose
are never inputs to the navigation graph.

RViz shows the semantic class/confidence maps, traversability, D*-Lite path, and
rover model. Camera overlay, live contact bridge, obstacle insertion, complete
TF/localization, and physical goal execution still depend on simulator plugins
and runtime assets. GUI appearance and end-to-end operation require human
inspection and cannot be established by static tests.

Preview the experiment matrix with `scripts/run_experiment.sh`; actual execution
is opt-in via `scripts/run_experiment.sh --execute` and should begin only after a
single launch is demonstrated successfully.
