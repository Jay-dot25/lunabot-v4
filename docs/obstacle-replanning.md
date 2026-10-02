# Obstacle-triggered replanning evidence

`replan_verifier` replaces motion-only path-change observation for production.
It correlates sensor obstacle detection, a lethal traversability update, an
obstacle-reason `ReplanEvent`, and new path publication. A valid event requires
the obstacle to intersect the active path and the replacement path to exclude
the blocked footprint. Start-motion replans cannot pass.

```bash
ros2 run lunabot_planning replan_verifier
```

It publishes machine-readable evidence on
`/lunabot/planning/replan_verification`, including detection-to-map, repair, and
total latency. Mission success additionally requires goal completion and zero
collisions. Unit tests execute 20 deterministic correlations, but the final
exit gate still requires 20 physical simulator runs with sensor evidence,
goal completion, and no contact; synthetic unit runs are not that evidence.
