# Collision and mission metrics

`ros2 run lunabot_evaluation mission_metrics` accumulates planned and executed
length, duration, path efficiency, traversability cost, terrain distance,
replans, minimum clearance, controller cross-track RMSE, and collisions. It
publishes typed `MissionMetrics` and atomically writes one `trial.json` when
`/lunabot/mission/result` is received.

Collision events on `/lunabot/simulation/contact` are JSON objects with `stamp`,
`object`, `active`, and optional `relative_speed` and `impulse`. This topic must
be produced by a Gazebo contact-sensor bridge; planner status or obstacle-map
changes are not collision evidence. Duplicate active messages do not inflate
collision count. Active contacts are closed at mission end.

ATE/RPE, inference timing, clearance, terrain class, and resource utilization
require their respective runtime producers. Unavailable metrics must remain
explicitly unavailable rather than fabricated. Final numerical results require
held-out simulator trials.
