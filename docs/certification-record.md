# LunaBot Certification Record

> Change a phase status from **LOCKED** to **PASSED** only after its
> acceptance criteria are demonstrated and the evidence is retained.
> Statuses: `LOCKED` (not certified) · `IN PROGRESS` (work underway, gate not
> yet demonstrated) · `PASSED` (gate demonstrated, evidence saved).

| # | Phase | Required deliverable | Gate status | Evidence |
|---|---|---|---|---|
| 1 | Lunar Base Camp | Working simulation and rover sensors | IN PROGRESS | `docs/phase-1-lunar-base-camp.md`, `evidence/phase1/` |
| 2 | Eyes of Luna | Evaluated segmentation model | LOCKED | — |
| 3 | Lost Without GPS | Working localisation and geometric mapping | LOCKED | — |
| 4 | Terrain Intelligence | Semantic map and cost generation | LOCKED | — |
| 5 | Route Architect | Terrain-aware path planner | LOCKED | — |
| 6 | Autonomous Pilot | Route execution and obstacle replanning | LOCKED | — |
| 7 | Lunar Mission Control | Mission interface and status dashboard | LOCKED | — |
| 8 | Lunar Proving Ground | Repeatable tests and evaluation report | LOCKED | — |
| F | Mission LUNA-007 | Fully integrated autonomous mission | LOCKED | — |

Rules:

* The final integration phase remains locked until all eight development
  phases have passed.
* For each phase, retain: code, configuration, test procedure, output, and
  acceptance results — so a failure points at exactly one subsystem.
* If a gate fails: fix that phase, repeat its tests, earn the clearance
  before proceeding.

## Current status notes

### Phase 1 — Lunar Base Camp (IN PROGRESS)
Implementation exists and is ready for acceptance runs:

* Lunar world `src/lunabot_gazebo/worlds/lunar_base_camp.sdf`
* Rover model `src/lunabot_description/models/lunabot_v4/model.sdf`
* Teleop `src/lunabot_teleop/`
* Automated acceptance test `src/lunabot_tests/` → evidence in `evidence/phase1/`

Gate turns to **PASSED** after the acceptance procedure in
`docs/phase-1-lunar-base-camp.md` is executed on a ROS 2 + Gazebo machine
and the reports are committed here.
