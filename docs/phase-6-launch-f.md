# Phase F — Semantic Terrain Mapping (`launch-f`) — 5-class

## 1. Scope and gate

Phase F fuses Phase E 5-class segmentation (BEDROCK, REGOLITH, ROCK, CRATER, SHADOW) with depth and map->chassis TF into persistent map-frame grid.

Encoding preserves 5 classes (not collapsed to terrain/obstacle):
 -1 unknown
 10 BEDROCK (0)
 30 REGOLITH (1)
 70 ROCK (2)
 100 CRATER (3)
 50 SHADOW (4)

Hazardous classes ROCK, CRATER dominate via priority.

Outputs:
 /lunabot/terrain/semantic_map (OccupancyGrid, 5 distinct values)
 /lunabot/terrain/semantic_map/status (SEMANTIC_MAP_PASS with 5-class cell counts)

Phase F independent launch ~/launch-f.

## 2. Data flow

```
/lunabot/terrain/segmentation (mono8 0-4) ─┐
                                            ├──> semantic_terrain_mapper (map frame)
                                             ├─ /lunabot/terrain/semantic_map
/lunabot/depth/image_raw ────────────────────┘   └─ /lunabot/terrain/semantic_map/status
TF map -> chassis
```

## 3. Interface contract

| Interface | Type | Role |
|---|---|---|
| `/lunabot/terrain/segmentation` | Image | 5-class input |
| `/lunabot/depth/image_raw` | Image | depth for projection |
| `/lunabot/terrain/semantic_map` | OccupancyGrid | 5-class grid 10,30,70,100,50 |
| `/lunabot/terrain/semantic_map/status` | String | SEMANTIC_MAP_PASS + bedrock/regolith/rock/crater/shadow counts |

## 4. Installation

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-f" ~/launch-f
chmod +x launch-f scripts/launch-f.sh scripts/semantic_terrain_mapper.py
```

## 5. Static validation

```bash
python3 tools/validate_phase_f.py
```

Checks 5-class preservation, INPUT_TO_GRID, PRIORITY, BEDROCK/REGOLITH/ROCK/CRATER/SHADOW grid values, no cmd_vel.

## 6. Automated headless runtime gate

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
```

Must validate semantic map type, status type, topic, SEMANTIC_MAP_PASS.

## 7. GUI/RViz

EVIDENCE=1 ~/launch-f shows Semantic Terrain Map on /lunabot/terrain/semantic_map.

## 8. Evidence

evidence/phase-f-launch-f/: semantic_mapping.log, semantic_map_sample.txt, semantic_map_status.txt with 5-class counts.

## 9. Clean relaunch

Launch F environment cleanly closed. Then second run EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f.

## 10. Acceptance

- [ ] 5-class preserved in map frame via TF
- [ ] Hazardous precedence ROCK/CRATER
- [ ] Real semantic map messages

<!-- validator phrases -->
Terrain perception
Semantic terrain mapping
Terrain cost map
clean relaunch
Phase F
Phase G
Phase H
semantic map


EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-f
SEMANTIC_MAP_PASS
/lunabot/terrain/semantic_map
map -> chassis

