# Layered traversability mapping

`traversability_node` is the sole production publisher of
`/lunabot/traversability/map`. It synchronizes semantic class, confidence,
elevation, and stale layers from calibrated fusion and applies:

```text
semantic cost + weighted slope + weighted roughness
+ weighted clearance + weighted uncertainty
```

Costs are clamped to 0–99 and 100 is reserved for lethal cells. Configuration
is installed from `src/lunabot_mapping/config/traversability.yaml`. The mapper
enforces longitudinal/cross-slope and step limits, footprint-aware inflation,
unknown and stale policies, and invalid/NaN elevation rejection. The companion
`/lunabot/traversability/reason` grid identifies the dominant hard constraint:
unknown, stale, invalid elevation, semantic lethal, slope, step, or inflation.

```bash
ros2 run lunabot_mapping traversability_node --ros-args \
  --params-file install/lunabot_mapping/share/lunabot_mapping/config/traversability.yaml
```

The historical `scripts/terrain_cost_mapper.py` remains a baseline only and
must not run alongside the production node. Planner bringup must subscribe only
to `/lunabot/traversability/map`.

Unit tests verify every layer and demonstrate that a longer safe route has less
total cost than a shorter route through a crater. Final footprint dimensions,
slope limits, and weights require rover-specific physical validation.
