# Phase 3 — Structured ROS interfaces and legacy compatibility

## Why this phase exists

The validated A–L baseline communicates state through `std_msgs/String` values
such as `DYNAMIC_REPLAN_PASS`. Strings are useful for terminal evidence but are
unsafe as the primary machine contract: fields are implicit, typos compile,
and timestamps/units are not enforced.

Phase 3 adds typed interfaces while preserving every old string topic.
Existing scripts are deliberately unchanged.

## Interfaces

| Message | Intended producer/consumer |
|---|---|
| `TerrainPrediction` | ML inference metadata, aligned class/confidence images |
| `PlannerStatus` | A*, weighted A*, and D*-Lite state/metrics |
| `ReplanEvent` | Correlated detection, map update, and path-repair event |
| `SafetyStatus` | Safety-supervisor decision and reason |
| `MissionMetrics` | Quantitative mission result and progress |

Definitions are in `src/lunabot_msgs/msg/`. Constants are part of the message
contract so nodes do not exchange undocumented numeric magic values.

## Compatibility bridge

`status_compat_bridge` is an optional observation-only ROS node in
`lunabot_evaluation`. It subscribes to retained legacy strings and republishes:

```text
/lunabot/terrain/planner/status  -> /lunabot/terrain/planner/status_typed
/lunabot/autonomy/replan_status  -> /lunabot/autonomy/replan_event
/lunabot/mission/status          -> /lunabot/mission/metrics
```

The bridge publishes no velocity, goal, map, or localization data. Unknown text
never becomes a success state. Legacy topics remain authoritative for existing
A–L validators until each producer is migrated in its own tested phase.

`TerrainPrediction` and `SafetyStatus` have no honest old-string equivalent;
they will be published natively by the perception and safety phases.

## Static verification

```bash
python3 tools/validate_interfaces.py
python3 -m unittest -v tests.test_status_compat
python3 tools/validate_all.py --no-phase-validators --no-write
```

## ROS build and interface inspection

```bash
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 interface show lunabot_msgs/msg/PlannerStatus
ros2 interface show lunabot_msgs/msg/ReplanEvent
ros2 interface show lunabot_msgs/msg/MissionMetrics
```

Run the optional bridge after starting a baseline phase that publishes the old
topics:

```bash
ros2 run lunabot_evaluation status_compat_bridge
```

Inspect typed topics in another terminal:

```bash
ros2 topic echo /lunabot/autonomy/replan_event
ros2 topic echo /lunabot/mission/metrics
```

## Compatibility rule

Do not remove an old status topic merely because its typed equivalent exists.
A legacy output may be removed only after all launch scripts, validators,
evidence tools, and consumers have migrated and a dedicated regression phase
has passed.
