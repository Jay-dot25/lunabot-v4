# Phase L Evidence — Final Presentation Mission

This directory is written by the independent `~/launch-l` final mission gate.

## Manual presentation gate

```bash
cd ~/lunabot-v4
FINAL_DEMO=1 AUTO_GOAL=false EVIDENCE=1 ~/launch-l
```

Use RViz's `Set Goal` tool to choose the rover destination. The mission must
sense the physical Gazebo obstacle with LiDAR, update the obstacle/cost map,
replan, and reach the selected goal.

A repeatable infrastructure regression is also available:

```bash
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  EVIDENCE=1 HEADLESS=1 ~/launch-l
```

The regression does not replace the manual final demonstration.

Expected manual acceptance includes:

```text
manual RViz goal selection: PASS
OBSTACLE_DETECTED
DYNAMIC_REPLAN_PASS
MISSION_DEMO_PASS
PHASE L RUN COMPLETE - overall result: PASS
Launch L environment cleanly closed.
```

Important evidence files include `obstacles.log`, `obstacle_status.txt`,
`obstacle_map.txt`, `goal_selection_wait_status.txt`, `mission.log`,
`mission_status.txt`, `replan_status.txt`, `phase_l_map.yaml`, and
`phase_l_map.pgm`. Do not hand-edit generated runtime evidence.
