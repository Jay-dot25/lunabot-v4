# LunaBot — Project Synopsis (structured requirements)

> Source of truth for the project. Structured from the approved project
> brief "LUNABOT: THE LUNAR AUTONOMY CHALLENGE — Autonomous Navigation of a
> Robot for Lunar Habitats".

## Central idea: earn your mission clearance

LunaBot is being developed for a future lunar settlement. Before it may
explore, it must pass a series of engineering certifications:

1. It can drive.
2. It can see and understand terrain.
3. It can estimate its position, build a map, and choose a safe route.
4. All capabilities work together in a complete autonomous mission.

Each development phase is one certification. **The golden rule: a phase is
complete only when its output works and its pass criteria have been
demonstrated.** Writing code is not sufficient. Training a model is not
sufficient. Creating an attractive simulation is not sufficient.

If a phase fails: fix that phase, repeat its tests, earn the clearance,
then proceed.

## Phase programme

| Phase | Theme | Certification earned | Core deliverable |
|---|---|---|---|
| 1 | Lunar Base Camp | Base Camp | Working simulation, rover, sensors |
| 2 | Eyes of Luna | Perception | Evaluated terrain-segmentation model |
| 3 | Lost Without GPS | GPS-Independent Nav | Localisation + geometric mapping |
| 4 | Terrain Intelligence | Terrain Intelligence | Semantic map + traversability costs |
| 5 | Route Architect | Route Planning | Terrain-aware path planner |
| 6 | Autonomous Pilot | Autonomous Pilot | Route execution + obstacle replanning |
| 7 | Lunar Mission Control | Mission Control | Dashboard + structured mission manager |
| 8 | Lunar Proving Ground | Proving Ground | Repeatable tests + evaluation report |
| Final | Mission LUNA-007 | Full acceptance | End-to-end autonomous mission |

## Phase requirements (summary)

### Phase 1 — Lunar Base Camp
* ROS 2 + Gazebo: lunar ground, mobile rover, rocks/obstacles, camera,
  LiDAR (or range sensor), wheel odometry, manual control.
* Rover moves forward/backward, rotates, stops; camera live images; range
  sensor usable measurements; odometry changes with motion; everything
  survives a repeatable test.
* **Gate:** world launches; rover responds to commands; sensor data on the
  configured interfaces; restartable and repeatable; obstacle detectable via
  the range sensor; **no component needs Gazebo ground-truth pose**.
* Creative demo: three obstacles ahead; live camera + range while driving
  around them manually.

### Phase 2 — Eyes of Luna
* Semantic segmentation (PyTorch or TensorFlow) with one architecture
  (e.g. U-Net, DeepLabV3+, SegFormer — one, not all).
* Classes: regolith, bedrock, rocks, craters, shadows (extendable later).
* Training needs annotated image + pixel-label masks; splits defined.
* **Gate:** consistent inference; mask matches input dimensions; all classes
  represented; held-out evaluation; per-class IoU + mIoU recorded; rocks and
  craters evaluated as safety-relevant; live sim images process cleanly.
* Provisional targets: mIoU ≥ 0.60, hazard-class recall ≥ 0.80 (to be
  reviewed against dataset/baseline before freezing).
* Creative demo: raw image beside predicted mask.

### Phase 3 — Lost Without GPS
* SLAM/localisation from range sensing + odometry (+ other supported
  sensors). Geometric occupancy map is enough at this stage.
* Estimated pose p̂ = (x, y, θ) in a local frame, shared with future
  navigation modules; pose estimates recorded for evaluation.
* **Gate:** estimated pose maintained while driving; map built from sensor
  observations and updates with exploration; **ground truth used only inside
  evaluation**, never as controller input; loop-drive error test.
* Creative demo: two markers on one map — estimated pose vs evaluation-only
  ground truth.

### Phase 4 — Terrain Intelligence
* Semantic mapping + traversability costs: combine Phase 2 masks with Phase 3
  geometry/pose. Geometry + rover capability must inform risk (a crater class
  alone does not reveal depth/traversability).
* Cost policy example (configurable, tested against rover capabilities):
  accessible regolith = low, traversable bedrock = moderate, small rocks =
  elevated, boulder field = high, crater rim = very high, unsafe slope =
  forbidden, unknown = configurable uncertainty penalty.
* **Gate:** masks associated with map regions; costs from semantics+geometry;
  same region gets appropriate cost; hazards cost more than comparable safe
  terrain; **forbidden ≠ merely expensive**; unknown not auto-safe; map grows
  with new observations.
* Creative demo: camera | segmentation | cost map, synchronised.

### Phase 5 — Route Architect
* A* over a grid/graph first; D*-Lite (or similar) later if needed. One
  correctly integrated planner beats several incomplete ones.
* Objective: minimise J(P) = Σ C(c) over path cells (extendable with
  distance, turning, clearance). Must respect obstacles + forbidden regions.
* Output: route in map coordinates, planning cost/stats, clear "no route"
  reporting; optional geometric-shortest-path comparison view.
* **Gate scenario:** Route A (short, near hazardous crater rim) vs Route B
  (longer, safe terrain) — planner picks the option with lower *configured
  cost* (not a hard-coded detour rule). Result must respond to changes in
  goal, obstacle map or costs.
* Creative demo: two coloured routes + the expensive/forbidden crater region.

### Phase 6 — Autonomous Pilot
* Controller converts routes to motion: progress tracking, commands, hazard
  monitoring, map updates, safety stops, replan requests, resume.
* **Gate — two tests:** (A) route execution without manual steering;
  (B) unexpected obstacle inserted mid-mission → detected via the navigation
  pipeline → map updated → rover responds → new route when feasible → no
  collision with the introduced obstacle. Repeat several times.
* Creative demo: mission-control incident narration (hazard detected → route
  invalidated → alternative calculated → movement resumed), every event
  real.

### Phase 7 — Lunar Mission Control
* Dashboard with six views: live world, rover camera, terrain segmentation,
  SLAM + semantic map, cost map + planned route, mission status/performance.
* Structured mission form (mission ID, start, destination, optional return,
  priority, cargo, navigation mode, completion condition).
* Mission state machine: READY → INITIALISING → NAVIGATING → DESTINATION
  REACHED → RETURNING → COMPLETED (+ failure/cancel/attention states).
* **Gate:** mission submit → valid goal; live real data everywhere (no
  hand-drawn routes or hard-coded success); camera/segmentation views
  correspond; state changes driven by real events; failure vs completion
  distinct; mission record saved and reviewable.
* Creative demo: ops-centre status lights (green/amber/red) reflecting real
  subsystem health.

### Phase 8 — Lunar Proving Ground
* Repeatable suite; same evaluation procedure; automatic recording.
* Scenarios: (1) easy delivery Hub→Solar Farm, (2) hazard-aware navigation
  to Science Station past crater rim, (3) unexpected obstacle mid-mission,
  (4) uncertain terrain / unknown region, (5) return journey.
* Metrics (measured only — never invented): mission success rate, path
  length, collision count, replanning events + replanning time, localisation
  error, segmentation performance, forbidden-terrain entry, mission duration.
* **Gate:** repeatable launch + evaluation; metrics from recorded data;
  semantic-planning test shows cost influence; obstacle test records real
  replanning; failures recorded; GT only in evaluation; report separates
  measured results from expected targets.
* Proposed initial reliability target: ≥ 8 successes in 10 end-to-end
  attempts with no hard-forbidden entry (planning target — freeze scenarios
  and rules before final evaluation).
* Creative demo: aerospace-style test report (intended route, actual
  trajectory, obstacles, replans, collisions, status).

### Final — Mission LUNA-007 (Geological Sample Delivery)
* Brief: transport a geological sample container Habitat Hub → Science
  Station → back, GPS/GNSS-independent local navigation, safety over
  distance, map updates as explored, replan when unsafe, record delivery,
  produce a report.
* Operational sequence (events generated from system state, never fixed
  timestamps): mission accepted → nav initialised → terrain observed →
  classified → geometry estimated → map updated → traversability → route
  planned → movement → hazard detected → replanned → destination reached →
  delivery recorded → return initiated → completed → report generated.
* Outputs: (A) working simulation with destinations/terrain/hazards,
  (B) live dashboard, (C) mission debrief with real measured results.
* Acceptance criteria: world+rover, evaluated DL, estimated-pose control,
  sensor-driven mapping, semantic influence on costs, real planning inputs,
  autonomous route following, hazard response, mission event recording, real
  dashboard data, measured evaluation, repeatable suite.

## Development control system

A certification record tracks every phase: LOCKED → PASSED. Change a status
only after the acceptance criteria are met **and the evidence is retained**
(code, configuration, test procedure, output, acceptance results per phase).
Final integration stays locked until all eight phases have passed.

## Global engineering rules

1. **Ground truth policy:** Gazebo's true pose may exist for evaluation and
   demos only. Controllers, mappers, and planners must run on estimated pose
   and sensor data. (Phase 1 gate makes this explicit.)
2. **Evidence policy:** every gate claim needs a reproducible test and saved
   output. Reports must use actual experimental data.
3. **Interface policy:** each phase's output must be a working capability the
   next phase can consume (topics/services/data files, not screenshots).
4. **Scope discipline:** one well-integrated algorithm beats several partial
   ones. No phase advances past a failed gate.
