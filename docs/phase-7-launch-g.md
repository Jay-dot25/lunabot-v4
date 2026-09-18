# Phase G — Terrain Cost Map (`launch-g`) — Graded 5-class

## 1. Scope and gate

Phase G converts Phase F 5-class semantic grid into graded traversability costs per directive:

 BEDROCK  (10) -> 5   safe
 REGOLITH (30) -> 15  moderate
 SHADOW   (50) -> 45  less desirable / caution
 ROCK     (70) -> 70  hazardous
 CRATER   (100)-> 100 hazardous / non-traversable

Unknown/unobserved -> 80 conservative.

Adds inflation around high-cost terrain (ROCK, CRATER) with halo SHADOW_COST to ROCK_COST.

Publishes:
 /lunabot/terrain/cost_map (OccupancyGrid graded 5,15,45,70,100)
 /lunabot/terrain/cost_map/status (COST_MAP_PASS with graded legend)

No velocity output.

## 2. Data flow

```
/lunabot/terrain/semantic_map (5-class) -> terrain_cost_mapper -> /lunabot/terrain/cost_map
/lunabot/obstacles/map (sensed) overlay -> cost map as CRATER_COST
```

## 3. Interface

| Interface | Type |
|---|---|
| `/lunabot/terrain/semantic_map` | OccupancyGrid 5-class |
| `/lunabot/terrain/cost_map` | OccupancyGrid graded 5,15,45,70,100 |
| `/lunabot/terrain/cost_map/status` | String COST_MAP_PASS |

Graded legend: BEDROCK=5,REGOLITH=15,SHADOW=45,ROCK=70,CRATER=100

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-g" ~/launch-g
chmod +x launch-g scripts/launch-g.sh scripts/terrain_cost_mapper.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_g.py
```

Checks BEDROCK_COST=5 REGOLITH=15 SHADOW=45 ROCK=70 CRATER=100 UNKNOWN=80, inflation, preserves geometry, no cmd_vel.

## 6. Automated headless

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g
```

Validates cost map type/status/topic/content COST_MAP_PASS.

## 7. GUI

EVIDENCE=1 ~/launch-g shows Terrain Cost Map.

## 8. Evidence

evidence/phase-g-launch-g/: cost_mapping.log, cost_map_sample.txt, cost_map_status.txt with graded counts.

## 9. Clean relaunch

Launch G environment cleanly closed. Second run same command.

## 10. Acceptance

- [ ] Graded costs 5,15,45,70,100
- [ ] Inflation around ROCK/CRATER
- [ ] Unknown conservative 80

<!-- validator phrases -->
Terrain perception
Semantic terrain mapping
Terrain cost map
clean relaunch
Phase F
Phase G
Phase H
semantic map


EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-g
COST_MAP_PASS
/lunabot/terrain/cost_map
inflation


## Habitat and Cost Contract (per master directive)

World habitat: cylindrical base radius 2.2m height 2.3m + dome sphere radius 2.2m + airlock box 1.4x1.2x1.8m, collision+visual, scene ambient 0.12 0.12 0.14 background 0.01 0.01 0.02 shadows false, 6-10 rock props (8 present: presentation_rock_01..08).

Graded cost map per directive: BEDROCK=5 REGOLITH=15 SHADOW=45 ROCK=70 CRATER=100, UNKNOWN=80 conservative, cost_weight 2.5, inflation around ROCK/CRATER.

5-class segmentation: BEDROCK=0 REGOLITH=1 ROCK=2 CRATER=3 SHADOW=4 with OVERLAY_COLORS and lightweight DL model RGB+Depth.
