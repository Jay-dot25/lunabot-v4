# LunaBot V4: Week 1 to Week 4 Report
## Launch A to Launch D, explained like a story for a child

**Project:** LunaBot V4 autonomous lunar rover

**Time plan:**

| Week | Launch | Simple name | Main job |
|---|---|---|---|
| Week 1 | `launch-a` | Build the Moon and the rover | Make a believable lunar playground, rover, sensors, and manual driving |
| Week 2 | `launch-b` | Add the safety driver | Put a safe command gate between people/planners and the wheels; measure motion |
| Week 3 | `launch-c` | Give the rover a memory | Use LiDAR and odometry to build a map and locate the rover in that map |
| Week 4 | `launch-d` | Teach it to choose a route | Use A* search to find a safe path and follow it autonomously |

This report is based on the repository files, phase documents, validators, evidence notes,
world/model definitions, and launch scripts. A **comment line** means a source line whose
purpose is only to explain the code, such as a line beginning with `#` or an XML comment.
The code appendix at the end includes the project source with those comment-only lines
removed. Generated OBJ meshes are not pasted because they are data, not hand-written
program logic; their generator and their measured statistics are included.

---

## 1. The big idea

Imagine a small six-wheeled toy car exploring a huge model of the Moon.

- **Gazebo Sim** is the pretend Moon. It calculates gravity, collisions, wheels, and sensors.
- **ROS 2** is a mailbox system. Programs send messages through named topics.
- **TF** is a set of rulers and coordinate frames. It tells every program where the rover,
  camera, LiDAR, and map are relative to one another.
- **RViz2** is the dashboard. It draws the map, rover frames, camera images, and LiDAR dots.
- **A launch script** is a carefully ordered recipe. It starts the world, checks that pieces
  are really working, saves evidence, and shuts down cleanly.

The phases are deliberately independent. Running `~/launch-d` does not secretly run
`launch-a`, `launch-b`, or `launch-c`; it starts the earlier approved pieces directly.
That is like rebuilding the same Lego model from the instructions instead of assuming
someone left it on the table.

The main safety chain after Week 2 is:

```text
teleop or planner -> /cmd_vel_in -> Phase B controller -> /cmd_vel -> Gazebo DiffDrive -> wheels
```

The important rule is that later code does not jump around the safety controller.

---

# Week 1 — Launch A: make the Moon and the rover

## 2. What Launch A does

Launch A creates the first complete working scene:

1. Checks that the world, rover, terrain meshes, teleoperation program, and RViz file exist.
2. Sources ROS 2 Humble and detects either Gazebo Fortress (`ign`) or newer Gazebo (`gz`).
3. Detects either `ros_ign_bridge` or `ros_gz_bridge`.
4. Removes stale Gazebo processes from an old run.
5. Starts the lunar world.
6. Spawns one LunaBot V4 at the measured terrain height.
7. Bridges Gazebo messages into ROS 2 and ROS 2 commands back to Gazebo.
8. Publishes static sensor transforms.
9. Starts RViz and either WASD teleoperation or an automatic demonstration.
10. Waits for real messages and TF transforms, writes evidence, and shuts down safely.

The short command is:

```bash
ln -sfn "$PWD/launch-a" ~/launch-a
~/launch-a
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-a
```

`HEADLESS=1` removes windows, `DEMO=1` drives forward and turns automatically, and
`EVIDENCE=1` records runtime samples.

## 3. How the lunar terrain was designed

The terrain is made by `tools/generate_lunar_terrain.py`, using a fixed random seed of
`42`. A fixed seed means the same recipe makes the same Moon every time, which is useful
for testing. The terrain is not a photograph; it is a procedural landscape, like a
computer drawing made from mathematical bumps and holes.

The measured terrain design is:

- 400 m by 400 m mission area.
- Visual mesh: 320 by 320 grid, about 1.25 m between points, 102,400 vertices.
- Collision mesh: 100 by 100 grid, about 4 m between points, used to make physics faster.
- Height range: about -9.389 m to +4.970 m.
- 220 small craters, 70 medium craters, and 7 large craters.
- Large craters have lowered centers, raised rims, and some central peaks.
- Rolling multi-scale relief and small roughness make it look less like a flat grey floor.
- A smooth spawn pad near the origin gives the rover a sensible starting place.
- A flat catch plane below the mission area stops the rover falling forever if it drives
  far beyond the edge.

Why two meshes? The child-friendly answer is: the rover needs two pictures of the same
Moon. The detailed picture is for our eyes. The simpler picture is for the physics engine,
so it can calculate wheel contact quickly. The visual mesh has grey vertex colors and the
world adds grey material, producing the monochrome Moon appearance.

The terrain generator also calculates the spawn height. The launcher uses `SPAWN_Z=-2.308`,
which is the terrain height near the origin plus the rover's clearance and a small drop
margin. This is safer than guessing a very high spawn point.

## 4. How the world and habitat were designed

`src/lunabot_gazebo/worlds/lunar_world.sdf` defines:

- Lunar gravity: `0 0 -1.62 m/s^2`, about one sixth of Earth gravity.
- Physics, scene broadcasting, entity commands, sensors, IMU, and contact systems.
- A GUI camera positioned to show the large landscape.
- The terrain visual and terrain collision meshes.
- A static habitat building and an equipment module.
- A colored forward obstacle with both a visible box and collision geometry.
- A side rock landmark with both visible and collision geometry.
- The large horizon catch plane.

The forward obstacle is important. It is not just a painted picture. LiDAR and depth can
hit its collision shape, so later navigation phases can treat it as a real obstacle.

## 5. How the rover was designed

`src/lunabot_gazebo/models/lunabot_v4/model.sdf` describes a rocker-bogie-inspired rover:

- A main chassis, lower armor, upper electronics deck, front armor, solar panels, antenna,
  and sensor mast.
- Six driven wheels: three on the left and three on the right.
- Rocker and bogie joints so the wheel assemblies can respond to uneven ground.
- A mast with pan and tilt joints.
- A front-facing RGB camera, depth camera, 360-degree GPU LiDAR, and IMU.
- A differential-drive plugin connected to all six wheel joints.
- Joint-state publishing and steering/mast controller topics.

The rover's basic drive limits are 0.45 m/s forward or backward and 1.0 rad/s turning.
The six wheel names are used by the DiffDrive plugin and by the drive diagnostics. Under
lunar gravity the wheels have less normal force, so the model uses increased wheel-joint
damping and friction to make commanded motion more dependable in simulation.

The sensor mast is arranged like a child holding a flashlight above the car:

```text
odom -> chassis -> sensor_head -> {rgb_camera, depth_camera, lidar}
                              and the IMU is mounted on the chassis
```

The DiffDrive plugin publishes dynamic `odom -> chassis`. The static TF publishers add
sensor-frame relationships. A moving transform changes as the rover moves; a static one
stays fixed because the sensor is bolted to the rover.

## 6. Where references and data are obtained in Week 1

There are two kinds of references:

1. **Repository references:** the SDF files define the actual rover geometry, sensor poses,
   topic names, wheel names, gravity, and Gazebo plugins. Launch A reads those files and
   uses their values.
2. **Runtime references:** Gazebo produces actual odometry, joint states, LiDAR, camera,
   depth, IMU, and TF messages. The launcher checks those real messages with ROS 2 tools.

The key Week 1 interfaces are:

| Interface | Type | Why it exists |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | drive command to DiffDrive |
| `/lunabot/odom` | `nav_msgs/Odometry` | wheel odometry |
| `/lunabot/camera/image_raw` | `sensor_msgs/Image` | RGB view |
| `/lunabot/depth/image_raw` | depth image | distance picture |
| `/lunabot/lidar/scan` | `sensor_msgs/LaserScan` | surrounding range points |
| `/lunabot/imu` | `sensor_msgs/Imu` | acceleration and turning measurements |
| `/lunabot/joint_states` | `sensor_msgs/JointState` | wheel/joint motion |
| `/clock` | `rosgraph_msgs/Clock` | simulation time |
| `/tf` | `tf2_msgs/TFMessage` | coordinate transforms |

WASD uses `wasd_teleop.py`. In interactive mode W/S drive, A/D turn, Space stops, and Q
quits. In demo mode, the program drives forward for 3 simulated seconds, stops, turns,
and measures movement using `/clock`. Measuring simulation time is fairer than using a
wall clock when the computer renders slowly.

## 7. Week 1 acceptance and evidence

Static validation recorded **80/80 checks passed** in the repository evidence. Those checks
include XML/SDF validity, terrain mesh structure, sensor topics, plugins, bridge mappings,
spawn height, Python/Bash syntax, and RViz configuration.

The intended runtime gate checks that:

- Gazebo really starts and loads the world.
- The rover really spawns on the terrain.
- Camera, depth, LiDAR, IMU, odometry, and TF really publish.
- The rover moves forward and turns.
- RViz shows the TF tree, scan, camera, and depth image.
- Ctrl+C leaves no stale lunar-world process.
- A second launch works.

Evidence is stored under `evidence/phase-a-launch-a/`, including terrain previews, terrain
statistics, validation output, logs, topic samples, TF samples, LiDAR plots, and drive
diagnostics. The development sandbox does not have ROS 2/Gazebo, so runtime evidence must
be collected on the ROS workstation; static evidence must not be mistaken for a real drive.

---

# Week 2 — Launch B: add a safety driver and motion report

## 8. What Launch B adds

Launch B keeps the Week 1 Moon, rover, sensors, TF, and bridge, then adds two ROS programs:

- `scripts/control_odometry.py`: the safety controller.
- `scripts/odometry_monitor.py`: the motion quality reporter.

The new command path is:

```text
/cmd_vel_in -> lunabot_control -> /cmd_vel -> Gazebo DiffDrive
```

A human, a test program, or a future planner writes to `/cmd_vel_in`. The control node:

1. Limits speed to ±0.45 m/s and turning to ±1.0 rad/s.
2. Limits acceleration to 0.4 m/s² and angular acceleration to 0.8 rad/s².
3. Uses a 0.5-second wall-clock watchdog.
4. If commands stop arriving, ramps the output down to zero.
5. Publishes `/lunabot/control/status` with input age, targets, outputs, limits, and counts.

The watchdog is like a toy car that stops when the remote-control signal disappears. It uses
wall time intentionally: even if the simulation pauses, a safety timeout should still work.

## 9. How odometry is measured in Week 2

The monitor subscribes to the actual `/lunabot/odom` stream. It does not invent a position.
For every message it records:

- Simulation timestamp.
- x, y, and yaw.
- Linear and angular velocity.
- Distance between consecutive poses.
- Frame ID and child frame ID.

It writes `odometry_samples.csv` and, on shutdown, `odometry_report.txt`. It checks sample
count, simulation duration, rate, small pose steps, and that the frames are `odom` and
`chassis`. It also publishes `/lunabot/odometry/status`.

This is the difference between saying “the car probably moved” and measuring the car's
actual messages.

## 10. Week 2 references and connections

Launch B reuses these Week 1 references rather than changing them:

- `lunar_world.sdf` for gravity, terrain, habitat, and Gazebo systems.
- `model.sdf` for wheel joints, DiffDrive limits, odom topic, and TF frame names.
- The same bridge mappings for sensors and commands.
- The same static sensor TF geometry.
- The same `wasd_teleop.py`, but with `--topic /cmd_vel_in` in Phase B.

The new code references the real ROS message types `Twist`, `Odometry`, `JointState`, and
`String`. It uses the sensor-data QoS profile because Gazebo bridge sensor publishers use
best-effort delivery.

Launch B also explicitly unpauses `/world/lunar_world/control`. Server-only Gazebo can start
paused, so this is a necessary runtime action, not a cosmetic detail.

## 11. Week 2 acceptance and evidence

The repository records **103/103 static checks passed** for the Phase B gate. The intended
runtime acceptance is:

- `/cmd_vel_in` reaches `/cmd_vel` through the controller.
- Excessive commands are clamped and smoothly limited.
- No command for 0.5 seconds causes a safe stop.
- Real odometry has correct frames, rate, continuity, and nonzero movement.
- Interactive WASD still drives the rover.
- Clean shutdown and clean relaunch work.

The important design result is that all later autonomous code can produce `/cmd_vel_in`
without being trusted to implement emergency stopping itself.

---

# Week 3 — Launch C: give the rover a map and a location

## 12. What Launch C adds

Launch C adds exactly one mapping/localization system: `slam_toolbox`.

SLAM means **Simultaneous Localization and Mapping**. For a child, it is like walking
around a dark room with a ruler: the rover asks “what is near me?” and uses its movement
and measurements to draw a map while also estimating where it is on that map.

The data flow is:

```text
/lunabot/lidar/scan + odom -> chassis + /lunabot/odom
                              |
                              v
                         slam_toolbox
                              |
                      /map + map -> odom
```

The map is a `nav_msgs/OccupancyGrid`: a grid of little squares. A square can be free,
occupied, or unknown. `slam_toolbox` receives the LiDAR scan and the rover's odometry,
then publishes the map and the transform from `map` to `odom`.

The full movement path remains:

```text
/cmd_vel_in -> Phase B controller -> /cmd_vel -> DiffDrive -> /lunabot/odom
```

No second SLAM, AMCL, Cartographer, or extra odometry system is added.

## 13. Why TF matters in Week 3

The LiDAR message from Gazebo uses a scoped frame name:
`lunabot_v4/sensor_head/lidar`. The launch script publishes the exact scoped transform,
while preserving the easier sensor-frame relationships. This makes the map system able to
match “this scan came from this exact place on the rover.”

The reference chain becomes:

```text
map -> odom -> chassis -> sensor_head -> lidar
```

- `map -> odom` comes from `slam_toolbox`.
- `odom -> chassis` comes from the DiffDrive odometry.
- Sensor transforms come from the rover geometry and static TF publishers.

`config/slam_toolbox_phase_c.yaml` sets simulated time, `map`, `odom`, `chassis`, and the
scan topic. That file is the single configuration reference for the SLAM system.

## 14. Week 3 evidence and checks

The repository records **133/133 static checks passed** for the Phase C gate. The documented
workstation run also reports real map, TF, IMU, sensor, motion, map-saving, shutdown, and
relaunch checks as approved. In this sandbox, ROS 2/Gazebo cannot be rerun, so the report
carefully distinguishes the repository's recorded workstation result from a fresh local
runtime test.

The runtime evidence includes `map_sample.txt`, before/after map samples, `tf_map_odom.txt`,
`tf_odom_chassis.txt`, scoped LiDAR TF, odometry and IMU samples, saved map YAML/PGM, and
SLAM logs. A map update during rover motion is required: a node merely staying alive is
not enough.

RViz changes from the Week 2 dashboard to include `SLAM Map` on `/map`, fixed-frame TF,
LiDAR, camera, and odometry. Best-effort QoS is explicitly used for sensor displays.

---

# Week 4 — Launch D: plan a safe path and drive it

## 15. What Launch D adds

Launch D adds one custom program: `scripts/astar_navigation.py`.

A* is a treasure-map search. The map is divided into squares. The rover asks:

1. Which square am I in?
2. Which square contains the goal?
3. Which neighboring squares are safe?
4. Which safe route is shortest or cheapest?

The planner uses a heap-based open set, stores the best known travel cost (`g_score`),
uses a distance estimate to the goal, and searches eight directions, including diagonals.
It consumes the live SLAM `OccupancyGrid` and produces:

- `/goal_pose` (`geometry_msgs/PoseStamped`) as the requested destination.
- `/plan` (`nav_msgs/Path`) as the route drawn in RViz.
- `/lunabot/navigation/status` (`std_msgs/String`) for readable state messages.
- `/cmd_vel_in` (`geometry_msgs/Twist`) for motion commands into the Phase B safety gate.

The planner never publishes directly to `/cmd_vel` and has no Gazebo dependency.

## 16. How the planner knows what is safe

The planner treats cells with occupancy value 65 or higher as blocked. Unknown cells are
also blocked by default. It inflates obstacles by 0.25 m, meaning it marks a safety ring
around a rock instead of letting the rover's center pass right beside it.

The grid conversion uses the map resolution and the map origin's rotation. This is important:
a map may not start facing the same direction as the classroom floor. The planner has both
`_world_to_grid` and `_grid_to_world` functions so it can convert physical coordinates to
squares and back correctly.

If the rover or goal lands inside a blocked cell, `_nearest_free` searches nearby for a safe
cell. If there is no route, status becomes `NO_PATH`. If planning succeeds, status includes
`PLANNING_PASS`; while driving it reports `FOLLOWING`; on arrival it reports `GOAL_REACHED`.

The map subscription uses reliable, transient-local QoS. In simple words, if the planner
starts after the map was published, it can still receive the latest map rather than waiting
for the next one. The path and status outputs use the same late-join-friendly idea.

## 17. How the rover follows the path

The follower finds the closest path point and looks a few points ahead. It turns toward that
lookahead point. If the angle is too large, it turns without driving forward. Otherwise it
moves slowly, limited to 0.25 m/s and 0.7 rad/s. When it is within 0.35 m of the goal it
publishes zero velocity and reports success.

The automatic goal is only for regression mode (`DEMO=1`): 1.5 m ahead of the current map
pose. In normal GUI mode, RViz provides the operator's `Set Goal` tool on `/goal_pose`. This
separation prevents an accidental automatic drive when a human expects to choose a goal.

## 18. Week 4 references and evidence

Launch D takes its references from:

- `/map` and `map -> odom` from the one approved `slam_toolbox` system.
- `/lunabot/odom` and `odom -> chassis` from the inherited DiffDrive.
- The map origin, resolution, width, height, occupancy values, and orientation from the
  actual `nav_msgs/OccupancyGrid` message.
- The goal from RViz `/goal_pose` or the deterministic demo goal.
- The Phase B controller's `/cmd_vel_in` boundary.

The documented gate reports **152/152 static checks passed** for Phase D. The runtime gate
requires a non-empty path, real goal/path/status messages, a reached goal, live map changes
during navigation, saved map evidence, all inherited sensor/TF checks, and clean relaunch.
The runtime evidence directory includes navigation logs, goal, path, status, before/after
maps, map/odom TF, LiDAR TF, odometry, and final YAML/PGM map files.

RViz Week 4 uses `map` as its fixed frame and adds the `A* Path`, `Set Goal`, and selected
goal displays. The operator can see the same green route that the rover is following.

---

# 19. Complete interface map

```text
Gazebo lunar_world.sdf
  |-- terrain meshes, habitat, obstacle, gravity, physics, sensors
  |-- lunabot_v4/model.sdf
  |     |-- six wheels + DiffDrive
  |     |-- camera, depth camera, LiDAR, IMU
  |     `-- /lunabot/odom and odom -> chassis
  |
  `-- bridge
        |-- /clock
        |-- /lunabot/camera/image_raw
        |-- /lunabot/depth/image_raw
        |-- /lunabot/lidar/scan
        |-- /lunabot/imu
        |-- /lunabot/joint_states
        `-- /cmd_vel

WASD or A* -> /cmd_vel_in
                   |
             control_odometry.py
          clamp + acceleration + watchdog
                   |
                /cmd_vel
                   |
                DiffDrive
                   |
                six wheels

/lunabot/odom + TF + LiDAR -> slam_toolbox -> /map + map -> odom
/map + odom pose + /goal_pose -> astar_navigation.py
                                      |-- /plan
                                      |-- navigation status
                                      `-- /cmd_vel_in
```

# 20. File-by-file code map

The complete comment-free code is included in the appendix. These are the roles of every
hand-written source used by Weeks 1–4:

| File | Role |
|---|---|
| `launch-a` through `launch-d` | small symlink-safe entry points |
| `scripts/launch-a.sh` | Week 1 startup, bridge, TF, runtime checks, evidence, shutdown |
| `scripts/launch-b.sh` | Week 2 startup plus controller and monitor |
| `scripts/launch-c.sh` | Week 3 startup plus SLAM, map checks, map saving |
| `scripts/launch-d.sh` | Week 4 startup plus A*, planner checks, map saving |
| `scripts/wasd_teleop.py` | manual keyboard input and measured demo drive |
| `scripts/control_odometry.py` | command clamp, acceleration limiting, watchdog |
| `scripts/odometry_monitor.py` | odometry measurement and quality evidence |
| `scripts/astar_navigation.py` | occupancy-grid A* and conservative follower |
| `tools/generate_lunar_terrain.py` | deterministic crater/relief mesh generator |
| `tools/plot_lidar_scan.py` | turns a LiDAR sample into an evidence image |
| `tools/validate_phase_a.py` | Week 1 static gate |
| `tools/validate_phase_b.py` | Week 2 static gate |
| `tools/validate_phase_c.py` | Week 3 static gate |
| `tools/validate_phase_d.py` | Week 4 static gate |
| `src/.../lunar_world.sdf` | world, terrain references, habitat, obstacles, gravity/plugins |
| `src/.../model.sdf` | rover geometry, joints, sensors, drive, odometry |
| `config/slam_toolbox_phase_c.yaml` | sole SLAM configuration |
| `rviz/phase_a.rviz` through `phase_d.rviz` | dashboard layouts for each week |

The two OBJ files are generated terrain data. They are used by the SDF but are intentionally
not expanded into this report. The generator, stats, mesh dimensions, and validation rules
show exactly how they are made and checked.

# 21. Reproducible commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash

python3 tools/validate_phase_a.py
python3 tools/validate_phase_b.py
python3 tools/validate_phase_c.py
python3 tools/validate_phase_d.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-a
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-b
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-c
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-d
```

For visual runs, omit `HEADLESS=1` and use `~/launch-a`, `~/launch-b`, `~/launch-c`, or
`~/launch-d`. For Launch D GUI mode, choose the destination using RViz `Set Goal`.

# 22. What “done” means after Week 4

A child-friendly final test is:

1. The Moon looks like a Moon and the rover is on it.
2. The rover can be driven safely by a person.
3. The rover can draw where it has been and where it is.
4. The rover can receive a destination.
5. It finds a route that avoids blocked squares and stays away from obstacles.
6. It sends commands through the safety driver.
7. It stops at the destination.
8. The map, path, messages, logs, and shutdown all prove what happened.

That is the foundation for later terrain perception and terrain-aware planning phases. Weeks
1–4 do not yet claim semantic terrain understanding; they establish a trustworthy world,
control boundary, map, and basic autonomous route.

---

# Appendix A — complete source code, comment-only lines removed

The following blocks are generated from the repository at report creation time. Blank lines,
program logic, strings, data, and executable configuration remain. Full-line comments and
XML comment blocks are omitted as requested.


## `launch-a`

```bash
SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
exec "$(cd "$(dirname "$SCRIPT")" && pwd)/scripts/launch-a.sh" "$@"
```

## `launch-b`

```bash
SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
exec "$(cd "$(dirname "$SCRIPT")" && pwd)/scripts/launch-b.sh" "$@"
```

## `launch-c`

```bash
SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
exec "$(cd "$(dirname "$SCRIPT")" && pwd)/scripts/launch-c.sh" "$@"
```

## `launch-d`

```bash
SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
exec "$(cd "$(dirname "$SCRIPT")" && pwd)/scripts/launch-d.sh" "$@"
```

## `scripts/launch-a.sh`

```bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_a.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-a-launch-a"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"

HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
TELEOP_PID=""
TF_PIDS=()
BRIDGE_PKG=""

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

shutdown() {
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase A safely..."
  say "------------------------------------------------------------"
  [ -n "$TELEOP_PID" ] && kill "$TELEOP_PID" 2>/dev/null
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do kill "$p" 2>/dev/null; done
  [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null
    fi
    wait "$GAZEBO_PID" 2>/dev/null
    say "      Gazebo stopped."
  fi
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null); do
    kill -9 "$p" 2>/dev/null
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch A environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit 0
}
trap shutdown INT TERM

echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE A"
echo "                 SIMULATION & ROVER FOUNDATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

echo "[1/10] Checking project files...................."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$RVIZ_CONFIG" \
         "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required files missing (see above)."
  echo "      Re-generate terrain:  python3 tools/generate_lunar_terrain.py"
  exit 1
fi
echo "      world, rover, lunar habitat, physical obstacle, terrain meshes, teleop, rviz: OK"
log "[1/10] project files OK"

echo "[2/10] Checking ROS 2 / Gazebo environment......."
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  log "ERROR: ros2 CLI missing"
  exit 2
fi

IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz' in PATH)."
  echo "      Install ros-humble-gazebo-ros-pkgs + ros-humble-ros-ign"
  log "ERROR: Gazebo Sim not found"
  exit 2
fi

if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  echo "      Install: sudo apt install ros-humble-ros-ign"
  log "ERROR: bridge package not found"
  exit 2
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
log "[2/10] environment OK (ROS 2 humble, $IGN/$MSGNS, $BRIDGE_PKG)"

echo "[3/10] Checking for stale processes................"
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null
  sleep 2
fi
echo "      clean state: OK"
log "[3/10] clean state OK"

echo "[4/10] Starting Gazebo (lunar world)..............."
export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

: > "$EVIDENCE_DIR/gazebo.log"
if [ "$HEADLESS" = "1" ]; then
  setsid "$IGN" gazebo -s -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
else
  setsid "$IGN" gazebo -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
fi
GAZEBO_PID=$!

ready=0
for _ in $(seq 1 90); do
  if timeout 3 "$IGN" service -s "/world/$WORLD_NAME/info" \
       --reqtype "$MSGNS.Empty" --reptype "$MSGNS.World" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    echo "      ERROR: Gazebo exited during startup - see $EVIDENCE_DIR/gazebo.log"
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    exit 1
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "      ERROR: Gazebo did not become ready within 90 s."
  kill -KILL -"$GAZEBO_PID" 2>/dev/null
  exit 1
fi
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/10] gazebo OK (pid $GAZEBO_PID)"

echo "[5/10] Spawning LunaBot V4........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *)
    echo "      ERROR: spawn failed: $spawn_resp"
    kill -KILL -"$GAZEBO_PID" 2>/dev/null
    exit 1
    ;;
esac
log "[5/10] spawn OK (z=$SPAWN_Z)"

echo "[6/10] Starting ROS 2 <-> Gazebo bridge............"
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
  "/clock@rosgraph_msgs/msg/Clock[$MSGNS.Clock" \
  "/lunabot/camera/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/depth/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[$MSGNS.LaserScan" \
  "/lunabot/imu@sensor_msgs/msg/Imu[$MSGNS.IMU" \
  "/lunabot/odom@nav_msgs/msg/Odometry[$MSGNS.Odometry" \
  "/lunabot/joint_states@sensor_msgs/JointState[$MSGNS.Model" \
  "/tf@tf2_msgs/msg/TFMessage[$MSGNS.Pose_V" \
  "/lunabot/steer/front_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/front_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/pan@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/tilt@std_msgs/msg/Float64]$MSGNS.Double" \
  >> "$EVIDENCE_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "      ERROR: bridge exited at startup - see $EVIDENCE_DIR/bridge.log"
  tail -20 "$EVIDENCE_DIR/bridge.log" | sed 's/^/      | /'
  kill -KILL -"$GAZEBO_PID" 2>/dev/null
  exit 1
fi
echo "      bridge running (PID $BRIDGE_PID), 16 topic mappings"
log "[6/10] bridge OK (pid $BRIDGE_PID)"

echo "[7/10] Starting static TF (sensor frames)..........."
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      4 static transforms published (chassis -> sensor frames)"
log "[7/10] static TF OK"

if [ "$HEADLESS" = "1" ]; then
  echo "[8/10] RViz2 (skipped - HEADLESS)................... OK"
  log "[8/10] rviz skipped (headless)"
else
  echo "[8/10] Starting RViz2..............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f odom >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: odom"
    log "[8/10] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[8/10] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

echo "[9/10] Runtime validation..........................."
topic_ok() {  # <name> <topic>
  if timeout 30 ros2 topic echo "$2" --once 2>/dev/null | head -n 3 >/dev/null; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
  fi
}
tf_ok() {  # <name> <parent> <child>
  if timeout 30 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
    echo "      $1: PASS"
    log "validation PASS: TF $2->$3"
  else
    echo "      $1: FAIL"
    log "validation FAIL: TF $2->$3"
    OVERALL="FAIL"
  fi
}
if kill -0 "$GAZEBO_PID" 2>/dev/null; then
  echo "      Gazebo process alive: PASS"
  log "validation PASS: gazebo process"
else
  echo "      Gazebo process alive: FAIL"
  log "validation FAIL: gazebo process"
  OVERALL="FAIL"
fi
topic_ok "topic /lunabot/odom"              "/lunabot/odom"
topic_ok "topic /lunabot/camera/image_raw"  "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/depth/image_raw"   "/lunabot/depth/image_raw"
topic_ok "topic /lunabot/lidar/scan"        "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"               "/lunabot/imu"
tf_ok    "TF odom -> chassis"               "odom" "chassis"
tf_ok    "TF chassis -> sensor_head"        "chassis" "sensor_head"

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null     > "$EVIDENCE_DIR/topics.txt"
  {
    timeout 20 ros2 topic list 2>/dev/null | while read -r t; do
      timeout 10 ros2 topic info "$t" --verbose 2>/dev/null | head -8
      echo "---- $t"
    done
  } > "$EVIDENCE_DIR/topic_info.txt"
  timeout 25 ros2 run tf2_ros tf2_echo "odom" "chassis" 2>/dev/null \
    | head -12 > "$EVIDENCE_DIR/tf_odom_chassis.txt"
  timeout 25 ros2 topic echo /lunabot/odom --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 30 ros2 topic echo /lunabot/lidar/scan --once 2>/dev/null \
    > "$EVIDENCE_DIR/lidar_scan_sample.yaml"
  python3 "$REPO_DIR/tools/plot_lidar_scan.py" \
    "$EVIDENCE_DIR/lidar_scan_sample.yaml" \
    "$EVIDENCE_DIR/lidar_scan_preview.png" 2>/dev/null \
    || echo "      (LiDAR plot skipped - needs matplotlib/pyyaml)"
  echo "      evidence written (topics, tf, odom, lidar scan + plot)"
  log "[9/10] evidence recorded"
fi

echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 4 static)"
echo "Phase Component   : RUNNING"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "Topics (ROS 2):"
echo "  /cmd_vel                      geometry_msgs/Twist        (INPUT - drive)"
echo "  /clock                        rosgraph_msgs/Clock        (sim time)"
echo "  /lunabot/odom                 nav_msgs/Odometry"
echo "  /lunabot/camera/image_raw     sensor_msgs/Image          (RGB 640x480 @20Hz)"
echo "  /lunabot/depth/image_raw      sensor_msgs/Image          (depth @15Hz)"
echo "  Gazebo lunar_habitat_main    static habitat presentation model"
echo "  Gazebo presentation_obstacle_forward  physical LiDAR obstacle"
echo "  /lunabot/lidar/scan           sensor_msgs/LaserScan      (720 beams @10Hz)"
echo "  /lunabot/imu                  sensor_msgs/Imu            (@100Hz)"
echo "  /lunabot/joint_states         sensor_msgs/JointState"
echo "  /lunabot/steer/{front,rear}_{left,right}  std_msgs/Float64"
echo "  /lunabot/mast/{pan,tilt}      std_msgs/Float64"
echo "TF frames: odom -> chassis -> sensor_head -> {rgb_camera, depth_camera, lidar}"
echo "Gazebo GUI camera: 0 -300 200 (lunar terrain overview)"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - all project files present"
echo "  - Gazebo + lunar_world started (terrain + habitat + physical obstacles)"
echo "  - LunaBot V4 spawned on terrain (spawn z=$SPAWN_Z)"
echo "  - RGB-D camera and LiDAR are available for later obstacle sensing"
echo "  - bridge + TF + sensor topics verified at runtime"
echo "  - overall: $OVERALL"
echo ""
echo "============================================================"
log "[10/10] status printed (overall $OVERALL)"

if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED DRIVE TEST (DEMO mode)"
  echo "============================================================"
  python3 "$WASD_PATH" --demo "$EVIDENCE_DIR"
  demo_rc=$?
  if [ "$demo_rc" -ne 0 ]; then
    OVERALL="FAIL"
  fi
  echo ""
  echo "============================================================"
  printf "AUTO RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown
else
  echo ""
  echo "============================================================"
  echo " Launch A environment is running."
  echo ""
  echo " Controls:"
  echo "   W = Forward      S = Reverse"
  echo "   A = Turn Left    D = Turn Right"
  echo "   Space = Stop     Q = Quit"
  echo ""
  echo " RViz2:"
  echo "   Camera  -> /lunabot/camera/image_raw"
  echo "   LiDAR   -> /lunabot/lidar/scan"
  echo "   TF      -> odom/chassis/sensor frames"
  echo ""
  echo " Press Ctrl+C to stop LunaBot safely."
  echo "============================================================"
  python3 "$WASD_PATH"
  shutdown
fi
```

## `scripts/launch-b.sh`

```bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
CONTROL_PATH="$REPO_DIR/scripts/control_odometry.py"
ODOM_PATH="$REPO_DIR/scripts/odometry_monitor.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_b.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-b-launch-b"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
TF_PIDS=()
BRIDGE_PKG=""
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

shutdown() {
  local rc="${1:-$EXIT_CODE}"
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase B safely..."
  say "------------------------------------------------------------"
  if [ -n "$CONTROL_PID" ]; then
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
  if [ -n "$ODOM_PID" ]; then
    kill -TERM "$ODOM_PID" 2>/dev/null || true
    wait "$ODOM_PID" 2>/dev/null || true
  fi
  [ -n "$BRIDGE_PID" ] && kill -TERM "$BRIDGE_PID" 2>/dev/null || true
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do kill "$p" 2>/dev/null || true; done
  [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null || true
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null || true
    fi
    wait "$GAZEBO_PID" 2>/dev/null || true
    say "      Gazebo stopped."
  fi
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null || true); do
    kill -9 "$p" 2>/dev/null || true
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch B environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit "$rc"
}
abort() {
  say "ERROR: $*"
  OVERALL="FAIL"
  shutdown 1
}
trap 'shutdown 130' INT TERM

echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE B"
echo "                 CONTROL & ODOMETRY"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

echo "[1/12] Checking Phase A baseline + Phase B files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" \
         "$RVIZ_CONFIG" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/B files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, rviz: OK"
log "[1/12] files OK"

echo "[2/12] Checking ROS 2 / Gazebo environment........"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  exit 2
fi
IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz')."
  exit 2
fi
if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  exit 2
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
log "[2/12] environment OK ($IGN/$MSGNS, $BRIDGE_PKG)"

echo "[3/12] Checking for stale Phase A/B processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/12] clean state OK"

echo "[4/12] Starting Gazebo lunar world.................."
export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
: > "$EVIDENCE_DIR/gazebo.log"
if [ "$HEADLESS" = "1" ]; then
  setsid "$IGN" gazebo -s -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
else
  setsid "$IGN" gazebo -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
fi
GAZEBO_PID=$!
ready=0
for _ in $(seq 1 90); do
  if timeout 3 "$IGN" service -s "/world/$WORLD_NAME/info" \
       --reqtype "$MSGNS.Empty" --reptype "$MSGNS.World" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    abort "Gazebo exited during startup; see $EVIDENCE_DIR/gazebo.log"
  fi
  sleep 1
done
[ "$ready" -eq 1 ] || abort "Gazebo did not become ready within 90 s"
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/12] gazebo OK and unpaused (pid $GAZEBO_PID)"

echo "[5/12] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/12] spawn OK (z=$SPAWN_Z)"

echo "[6/12] Starting ROS 2 <-> Gazebo bridge............."
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
  "/clock@rosgraph_msgs/msg/Clock[$MSGNS.Clock" \
  "/lunabot/camera/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/depth/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[$MSGNS.LaserScan" \
  "/lunabot/imu@sensor_msgs/msg/Imu[$MSGNS.IMU" \
  "/lunabot/odom@nav_msgs/msg/Odometry[$MSGNS.Odometry" \
  "/lunabot/joint_states@sensor_msgs/JointState[$MSGNS.Model" \
  "/tf@tf2_msgs/msg/TFMessage[$MSGNS.Pose_V" \
  "/lunabot/steer/front_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/front_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/pan@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/tilt@std_msgs/msg/Float64]$MSGNS.Double" \
  >> "$EVIDENCE_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3
kill -0 "$BRIDGE_PID" 2>/dev/null || abort "bridge exited; see $EVIDENCE_DIR/bridge.log"
echo "      bridge running (PID $BRIDGE_PID), 16 mappings"
log "[6/12] bridge OK (pid $BRIDGE_PID)"

echo "[7/12] Starting static TF (sensor frames)............"
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      4 static transforms published (chassis -> sensor frames)"
log "[7/12] static TF OK"

echo "[8/12] Starting Phase B control layer................"
: > "$EVIDENCE_DIR/control.log"
setsid python3 "$CONTROL_PATH" \
  --input-topic /cmd_vel_in --output-topic /cmd_vel \
  --max-linear 0.45 --max-angular 1.0 \
  --max-linear-accel 0.4 --max-angular-accel 0.8 \
  --watchdog-sec 0.5 --rate 30 \
  >> "$EVIDENCE_DIR/control.log" 2>&1 &
CONTROL_PID=$!
sleep 2
kill -0 "$CONTROL_PID" 2>/dev/null || abort "control node exited; see $EVIDENCE_DIR/control.log"
echo "      control running (PID $CONTROL_PID): /cmd_vel_in -> /cmd_vel"
log "[8/12] control OK (pid $CONTROL_PID)"

echo "[9/12] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[9/12] odometry monitor OK (pid $ODOM_PID)"

if [ "$HEADLESS" = "1" ]; then
  echo "[10/12] RViz2 (skipped - HEADLESS).................. OK"
  log "[10/12] rviz skipped (headless)"
else
  echo "[10/12] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f odom >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: odom"
    log "[10/12] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[10/12] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

echo "[11/12] Runtime validation.........................."
topic_ok() {
  local sample=""
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability best_effort --once 2>/dev/null)" && [ -n "$sample" ]; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
    if [ "$2" = "/lunabot/imu" ]; then
      timeout 10 "$IGN" topic -l 2>/dev/null \
        | grep -iE 'imu|inertial' > "$EVIDENCE_DIR/imu_gazebo_topics.txt" || true
      timeout 8 "$IGN" topic -e -t "/lunabot/imu" 2>/dev/null \
        > "$EVIDENCE_DIR/imu_gazebo_sample.txt" || true
      timeout 10 ros2 topic info /lunabot/imu --verbose 2>/dev/null \
        > "$EVIDENCE_DIR/imu_topic_info.txt" || true
      {
        echo "--- bridge IMU diagnostics ---"
        grep -iE 'imu|error|fail|warn' "$EVIDENCE_DIR/bridge.log" 2>/dev/null || true
        echo "--- Gazebo IMU diagnostics ---"
        grep -iE 'imu|sensor|error|fail|warn' "$EVIDENCE_DIR/gazebo.log" 2>/dev/null | tail -80 || true
      } > "$EVIDENCE_DIR/imu_diagnostics.txt"
      echo "      IMU diagnostics written to $EVIDENCE_DIR/imu_*"
    fi
  fi
}
tf_ok() {
  for _ in 1 2 3; do
    if timeout 10 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
      echo "      $1: PASS"
      log "validation PASS: TF $2->$3"
      return 0
    fi
    sleep 1
  done
  echo "      $1: FAIL"
  log "validation FAIL: TF $2->$3"
  OVERALL="FAIL"
  return 1
}
kill -0 "$GAZEBO_PID" 2>/dev/null && echo "      Gazebo process alive: PASS" || { echo "      Gazebo process alive: FAIL"; OVERALL="FAIL"; }
topic_ok "topic /clock"                       "/clock"
topic_ok "topic /lunabot/odom"                "/lunabot/odom"
topic_ok "topic /lunabot/control/status"      "/lunabot/control/status"
topic_ok "topic /lunabot/odometry/status"     "/lunabot/odometry/status"
topic_ok "topic /lunabot/camera/image_raw"   "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase B runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    | head -12 > "$EVIDENCE_DIR/tf_odom_chassis.txt"
  echo "      evidence recording started"
  log "[11/12] evidence recorded"
fi

echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 4 static)"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "  /cmd_vel_in                  geometry_msgs/Twist (control input)"
echo "  /cmd_vel                     geometry_msgs/Twist (smoothed output)"
echo "  /lunabot/control/status      std_msgs/String (watchdog + limits)"
echo "  /lunabot/odom                nav_msgs/Odometry (DiffDrive source)"
echo "  /lunabot/odometry/status     std_msgs/String (quality monitor)"
echo "  evidence/phase-b-launch-b/odometry_samples.csv"
echo "  evidence/phase-b-launch-b/odometry_report.txt"
echo "  evidence/phase-b-launch-b/demo_drive_result.txt"
echo "  evidence/phase-b-launch-b/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors and TF preserved"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[12/12] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED CONTROL + ODOMETRY TEST (DEMO mode)"
  echo "============================================================"
  python3 "$WASD_PATH" --demo --topic /cmd_vel_in "$EVIDENCE_DIR"
  demo_rc=$?
  if [ "$demo_rc" -ne 0 ]; then
    OVERALL="FAIL"
    EXIT_CODE=1
  fi
  sleep 1
  echo ""
  echo "============================================================"
  printf "PHASE B RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase B environment is running."
  echo ""
  echo " Controls: W forward | S reverse | A left | D right"
  echo "           Space stop | Q quit"
  echo ""
  echo " Control path: keyboard -> /cmd_vel_in -> controller"
  echo "               -> /cmd_vel -> Gazebo DiffDrive"
  echo ""
  echo " Press Ctrl+C to stop Phase B safely."
  echo "============================================================"
  python3 "$WASD_PATH" --topic /cmd_vel_in
  teleop_rc=$?
  [ "$teleop_rc" -eq 0 ] || EXIT_CODE=1
  shutdown "$EXIT_CODE"
fi
```

## `scripts/launch-c.sh`

```bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
CONTROL_PATH="$REPO_DIR/scripts/control_odometry.py"
ODOM_PATH="$REPO_DIR/scripts/odometry_monitor.py"
SLAM_CONFIG="$REPO_DIR/config/slam_toolbox_phase_c.yaml"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_c.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-c-launch-c"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
SLAM_PID=""
TF_PIDS=()
BRIDGE_PKG=""
MAP_SAVER_AVAILABLE=0
MAP_SAVE_DONE=0
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

stop_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 0
  kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.25
  done
  kill -KILL -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

save_final_map() {
  [ "$MAP_SAVE_DONE" = "1" ] && return 0
  [ "$EVIDENCE" = "1" ] || return 0
  [ "$MAP_SAVER_AVAILABLE" = "1" ] || return 0
  [ -n "$SLAM_PID" ] || return 0
  MAP_SAVE_DONE=1
  say "      saving final map evidence..."
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$EVIDENCE_DIR/phase_c_map" \
    --ros-args -p save_map_timeout:=10.0 2>/dev/null \
    >> "$EVIDENCE_DIR/map_saver.log" 2>&1 || true
  if [ -s "$EVIDENCE_DIR/phase_c_map.yaml" ] && [ -s "$EVIDENCE_DIR/phase_c_map.pgm" ]; then
    say "      saved map evidence (YAML + PGM): PASS"
    log "validation PASS: final map files saved"
    return 0
  fi
  say "      saved map evidence (YAML + PGM): FAIL"
  log "validation FAIL: map saver package was available but map files are missing"
  OVERALL="FAIL"
  return 1
}

shutdown() {
  local rc="${1:-$EXIT_CODE}"
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase C safely..."
  say "------------------------------------------------------------"
  if ! save_final_map; then
    rc=1
  fi
  if [ -n "$SLAM_PID" ]; then
    stop_group "$SLAM_PID"
    wait "$SLAM_PID" 2>/dev/null || true
  fi
  if [ -n "$CONTROL_PID" ]; then
    stop_group "$CONTROL_PID"
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
  if [ -n "$ODOM_PID" ]; then
    stop_group "$ODOM_PID"
    wait "$ODOM_PID" 2>/dev/null || true
  fi
  [ -n "$BRIDGE_PID" ] && stop_group "$BRIDGE_PID"
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do stop_group "$p"; done
  [ -n "$RVIZ_PID" ] && stop_group "$RVIZ_PID"
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null || true
    fi
    wait "$GAZEBO_PID" 2>/dev/null || true
    say "      Gazebo stopped."
  fi
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null || true); do
    kill -9 "$p" 2>/dev/null || true
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch C environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit "$rc"
}
abort() {
  say "ERROR: $*"
  OVERALL="FAIL"
  shutdown 1
}
trap 'shutdown 130' INT TERM

echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE C"
echo "                 SLAM & LOCALIZATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

echo "[1/13] Checking Phase A baseline + Phase C files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" "$SLAM_CONFIG" \
         "$RVIZ_CONFIG" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/C files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, rviz: OK"
log "[1/13] files OK"

echo "[2/13] Checking ROS 2 / Gazebo environment........"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  exit 2
fi
IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz')."
  exit 2
fi
if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  exit 2
fi
if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "      ERROR: slam_toolbox not found."
  echo "      Install: sudo apt install ros-humble-slam-toolbox"
  exit 2
fi
if ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  MAP_SAVER_AVAILABLE=1
  MAP_SAVER_STATUS="available"
else
  MAP_SAVER_STATUS="not installed (map saving will be skipped)"
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
echo "      SLAM system: slam_toolbox | map saver: $MAP_SAVER_STATUS"
log "[2/13] environment OK ($IGN/$MSGNS, $BRIDGE_PKG, slam_toolbox)"

echo "[3/13] Checking for stale Phase A/C processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py" "slam_toolbox.*online_async"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/13] clean state OK"

echo "[4/13] Starting Gazebo lunar world.................."
export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
: > "$EVIDENCE_DIR/gazebo.log"
if [ "$HEADLESS" = "1" ]; then
  setsid "$IGN" gazebo -s -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
else
  setsid "$IGN" gazebo -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
fi
GAZEBO_PID=$!
ready=0
for _ in $(seq 1 90); do
  if timeout 3 "$IGN" service -s "/world/$WORLD_NAME/info" \
       --reqtype "$MSGNS.Empty" --reptype "$MSGNS.World" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    abort "Gazebo exited during startup; see $EVIDENCE_DIR/gazebo.log"
  fi
  sleep 1
done
[ "$ready" -eq 1 ] || abort "Gazebo did not become ready within 90 s"
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/13] gazebo OK and unpaused (pid $GAZEBO_PID)"

echo "[5/13] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/13] spawn OK (z=$SPAWN_Z)"

echo "[6/13] Starting ROS 2 <-> Gazebo bridge............."
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
  "/clock@rosgraph_msgs/msg/Clock[$MSGNS.Clock" \
  "/lunabot/camera/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/depth/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[$MSGNS.LaserScan" \
  "/lunabot/imu@sensor_msgs/msg/Imu[$MSGNS.IMU" \
  "/lunabot/odom@nav_msgs/msg/Odometry[$MSGNS.Odometry" \
  "/lunabot/joint_states@sensor_msgs/JointState[$MSGNS.Model" \
  "/tf@tf2_msgs/msg/TFMessage[$MSGNS.Pose_V" \
  "/lunabot/steer/front_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/front_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/pan@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/tilt@std_msgs/msg/Float64]$MSGNS.Double" \
  >> "$EVIDENCE_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3
kill -0 "$BRIDGE_PID" 2>/dev/null || abort "bridge exited; see $EVIDENCE_DIR/bridge.log"
echo "      bridge running (PID $BRIDGE_PID), 16 mappings"
log "[6/13] bridge OK (pid $BRIDGE_PID)"

echo "[7/13] Starting static TF (sensor frames)............"
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
  "sensor_head|lunabot_v4/sensor_head/lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      5 static transforms published (including scoped Gazebo LaserScan frame)"
log "[7/13] static TF OK"

echo "[8/13] Starting Phase C control layer................"
: > "$EVIDENCE_DIR/control.log"
setsid python3 "$CONTROL_PATH" \
  --input-topic /cmd_vel_in --output-topic /cmd_vel \
  --max-linear 0.45 --max-angular 1.0 \
  --max-linear-accel 0.4 --max-angular-accel 0.8 \
  --watchdog-sec 0.5 --rate 30 \
  >> "$EVIDENCE_DIR/control.log" 2>&1 &
CONTROL_PID=$!
sleep 2
kill -0 "$CONTROL_PID" 2>/dev/null || abort "control node exited; see $EVIDENCE_DIR/control.log"
echo "      control running (PID $CONTROL_PID): /cmd_vel_in -> /cmd_vel"
log "[8/13] control OK (pid $CONTROL_PID)"

echo "[9/13] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[9/13] odometry monitor OK (pid $ODOM_PID)"

echo "[10/13] Starting slam_toolbox mapping................"
: > "$EVIDENCE_DIR/slam.log"
setsid ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$SLAM_CONFIG" use_sim_time:=true \
  >> "$EVIDENCE_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 4
kill -0 "$SLAM_PID" 2>/dev/null || abort "slam_toolbox exited; see $EVIDENCE_DIR/slam.log"
echo "      slam_toolbox running (PID $SLAM_PID), mapping /lunabot/lidar/scan"
log "[10/13] slam_toolbox OK (pid $SLAM_PID)"

if [ "$HEADLESS" = "1" ]; then
  echo "[11/13] RViz2 (skipped - HEADLESS).................. OK"
  log "[11/13] rviz skipped (headless)"
else
  echo "[11/13] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f odom >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: odom"
    log "[11/13] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[11/13] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

echo "[12/13] Runtime validation.........................."
topic_ok() {
  local sample=""
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability best_effort --once 2>/dev/null)" && [ -n "$sample" ]; then
    if [ "$2" = "/map" ] && { [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]]; }; then
      sample=""
    fi
  fi
  if [ -n "$sample" ]; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
    if [ "$2" = "/lunabot/imu" ]; then
      timeout 10 "$IGN" topic -l 2>/dev/null \
        | grep -iE 'imu|inertial' > "$EVIDENCE_DIR/imu_gazebo_topics.txt" || true
      timeout 8 "$IGN" topic -e -t "/lunabot/imu" 2>/dev/null \
        > "$EVIDENCE_DIR/imu_gazebo_sample.txt" || true
      timeout 10 ros2 topic info /lunabot/imu --verbose 2>/dev/null \
        > "$EVIDENCE_DIR/imu_topic_info.txt" || true
      {
        echo "--- bridge IMU diagnostics ---"
        grep -iE 'imu|error|fail|warn' "$EVIDENCE_DIR/bridge.log" 2>/dev/null || true
        echo "--- Gazebo IMU diagnostics ---"
        grep -iE 'imu|sensor|error|fail|warn' "$EVIDENCE_DIR/gazebo.log" 2>/dev/null | tail -80 || true
      } > "$EVIDENCE_DIR/imu_diagnostics.txt"
      echo "      IMU diagnostics written to $EVIDENCE_DIR/imu_*"
    fi
  fi
}
tf_ok() {
  for _ in 1 2 3; do
    if timeout 10 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
      echo "      $1: PASS"
      log "validation PASS: TF $2->$3"
      return 0
    fi
    sleep 1
  done
  echo "      $1: FAIL"
  log "validation FAIL: TF $2->$3"
  OVERALL="FAIL"
  return 1
}
kill -0 "$GAZEBO_PID" 2>/dev/null && echo "      Gazebo process alive: PASS" || { echo "      Gazebo process alive: FAIL"; OVERALL="FAIL"; }
map_type=""
for _ in 1 2 3; do
  map_type="$(timeout 10 ros2 topic type /map 2>/dev/null || true)"
  if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
    break
  fi
  sleep 1
done
if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
  echo "      /map type nav_msgs/msg/OccupancyGrid: PASS"
  log "validation PASS: /map type nav_msgs/msg/OccupancyGrid"
else
  echo "      /map type nav_msgs/msg/OccupancyGrid: FAIL"
  log "validation FAIL: /map type was [$map_type]"
  OVERALL="FAIL"
fi
topic_ok "topic /map"                         "/map"
tf_ok "TF map -> odom" "map" "odom"
topic_ok "topic /clock"                       "/clock"
topic_ok "topic /lunabot/odom"                "/lunabot/odom"
topic_ok "topic /lunabot/control/status"      "/lunabot/control/status"
topic_ok "topic /lunabot/odometry/status"     "/lunabot/odometry/status"
topic_ok "topic /lunabot/camera/image_raw"   "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"
tf_ok "TF sensor_head -> scoped LaserScan frame" "sensor_head" "lunabot_v4/sensor_head/lidar"

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase C runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    > "$EVIDENCE_DIR/tf_odom_chassis.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo map odom 2>/dev/null \
    > "$EVIDENCE_DIR/tf_map_odom.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo sensor_head lunabot_v4/sensor_head/lidar 2>/dev/null \
    > "$EVIDENCE_DIR/tf_lidar_scoped.txt" || true
  timeout 20 ros2 topic echo /map --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_sample.txt"
  if [ "$MAP_SAVER_AVAILABLE" = "1" ]; then
    echo "      final map saver will run during clean shutdown"
    echo "map saver scheduled for clean shutdown" > "$EVIDENCE_DIR/map_saver.log"
  else
    echo "      map saver unavailable; map_sample.txt is retained"
    echo "map saver skipped: install ros-humble-nav2-map-server to write PGM/YAML" \
      > "$EVIDENCE_DIR/map_saver.log"
  fi
  echo "      evidence written (map, TF map->odom, odom, SLAM log)"
  log "[12/13] evidence recorded"
fi

echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "SLAM              : RUNNING (slam_toolbox mapping)"
echo "Map               : RUNNING (/map + map->odom TF)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 5 static)"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "  /cmd_vel_in                  geometry_msgs/Twist (control input)"
echo "  /cmd_vel                     geometry_msgs/Twist (smoothed output)"
echo "  /lunabot/control/status      std_msgs/String (watchdog + limits)"
echo "  /lunabot/odom                nav_msgs/Odometry (DiffDrive source)"
echo "  /lunabot/odometry/status     std_msgs/String (quality monitor)"
echo "  /map                         nav_msgs/OccupancyGrid (SLAM map)"
echo "  map -> odom                  TF (SLAM localization)"
echo "  evidence/phase-c-launch-c/map_sample.txt"
echo "  evidence/phase-c-launch-c/phase_c_map.yaml/.pgm"
echo "  evidence/phase-c-launch-c/odometry_samples.csv"
echo "  evidence/phase-c-launch-c/odometry_report.txt"
echo "  evidence/phase-c-launch-c/demo_drive_result.txt"
echo "  evidence/phase-c-launch-c/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors, control and odometry preserved"
echo "  - one SLAM system: slam_toolbox"
echo "  - /map and map->odom localization outputs verified"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[13/13] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED SLAM + CONTROL + ODOMETRY TEST (DEMO mode)"
  echo "============================================================"
  if [ "$EVIDENCE" = "1" ]; then
    timeout 20 ros2 topic echo /map --once 2>/dev/null \
      > "$EVIDENCE_DIR/map_before_motion.txt" || true
  fi
  python3 "$WASD_PATH" --demo --topic /cmd_vel_in "$EVIDENCE_DIR"
  demo_rc=$?
  if [ "$demo_rc" -ne 0 ]; then
    OVERALL="FAIL"
    EXIT_CODE=1
  fi
  if [ "$EVIDENCE" = "1" ]; then
    sleep 2
    timeout 20 ros2 topic echo /map --once 2>/dev/null \
      > "$EVIDENCE_DIR/map_after_motion.txt" || true
    if [ -s "$EVIDENCE_DIR/map_before_motion.txt" ] && \
       [ -s "$EVIDENCE_DIR/map_after_motion.txt" ] && \
       ! cmp -s "$EVIDENCE_DIR/map_before_motion.txt" "$EVIDENCE_DIR/map_after_motion.txt"; then
      echo "      live map updates during rover motion: PASS"
      log "validation PASS: map changed during motion"
    else
      echo "      live map updates during rover motion: FAIL"
      log "validation FAIL: map did not produce distinct before/after samples"
      OVERALL="FAIL"
      EXIT_CODE=1
    fi
  fi
  if ! save_final_map; then
    EXIT_CODE=1
  fi
  sleep 1
  echo ""
  echo "============================================================"
  printf "PHASE C RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase C environment is running."
  echo ""
  echo " Controls: W forward | S reverse | A left | D right"
  echo "           Space stop | Q quit"
  echo ""
  echo " Control path: keyboard -> /cmd_vel_in -> controller"
  echo "               -> /cmd_vel -> Gazebo DiffDrive"
  echo ""
  echo " Press Ctrl+C to stop Phase C safely."
  echo "============================================================"
  python3 "$WASD_PATH" --topic /cmd_vel_in
  teleop_rc=$?
  [ "$teleop_rc" -eq 0 ] || EXIT_CODE=1
  shutdown "$EXIT_CODE"
fi
```

## `scripts/launch-d.sh`

```bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD_DIR="$REPO_DIR/src/lunabot_gazebo/worlds"
WORLD_PATH="$WORLD_DIR/lunar_world.sdf"
MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"
CONTROL_PATH="$REPO_DIR/scripts/control_odometry.py"
ODOM_PATH="$REPO_DIR/scripts/odometry_monitor.py"
SLAM_CONFIG="$REPO_DIR/config/slam_toolbox_phase_c.yaml"
NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"
RVIZ_CONFIG="$REPO_DIR/rviz/phase_d.rviz"
EVIDENCE_DIR="$REPO_DIR/evidence/phase-d-launch-d"
LOG_FILE="$EVIDENCE_DIR/last_run.log"

SPAWN_Z="-2.308"
WORLD_NAME="lunar_world"
HEADLESS="${HEADLESS:-0}"
DEMO="${DEMO:-0}"
EVIDENCE="${EVIDENCE:-0}"
AUTO_GOAL="${AUTO_GOAL:-false}"

OVERALL="PASS"
GAZEBO_PID=""
BRIDGE_PID=""
RVIZ_PID=""
CONTROL_PID=""
ODOM_PID=""
SLAM_PID=""
NAV_PID=""
GOAL_WAIT_PID=""
TF_PIDS=()
BRIDGE_PKG=""
MAP_SAVER_AVAILABLE=0
MAP_SAVE_DONE=0
EXIT_CODE=0

mkdir -p "$EVIDENCE_DIR"
: > "$LOG_FILE"
say() { echo "$@"; echo "$@" >> "$LOG_FILE"; }
log() { echo "$@" >> "$LOG_FILE"; }

stop_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 0
  kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.25
  done
  kill -KILL -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

save_final_map() {
  [ "$MAP_SAVE_DONE" = "1" ] && return 0
  [ "$EVIDENCE" = "1" ] || return 0
  [ "$MAP_SAVER_AVAILABLE" = "1" ] || return 0
  [ -n "$SLAM_PID" ] || return 0
  MAP_SAVE_DONE=1
  say "      saving final map evidence..."
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$EVIDENCE_DIR/phase_d_map" \
    --ros-args -p save_map_timeout:=10.0 2>/dev/null \
    >> "$EVIDENCE_DIR/map_saver.log" 2>&1 || true
  if [ -s "$EVIDENCE_DIR/phase_d_map.yaml" ] && [ -s "$EVIDENCE_DIR/phase_d_map.pgm" ]; then
    say "      saved map evidence (YAML + PGM): PASS"
    log "validation PASS: final map files saved"
    return 0
  fi
  say "      saved map evidence (YAML + PGM): FAIL"
  log "validation FAIL: map saver package was available but map files are missing"
  OVERALL="FAIL"
  return 1
}

shutdown() {
  local rc="${1:-$EXIT_CODE}"
  trap - INT TERM
  say ""
  say "------------------------------------------------------------"
  say " Shutting down LunaBot Phase D safely..."
  say "------------------------------------------------------------"
  if ! save_final_map; then
    rc=1
  fi
  if [ -n "$GOAL_WAIT_PID" ]; then
    stop_group "$GOAL_WAIT_PID"
    wait "$GOAL_WAIT_PID" 2>/dev/null || true
    GOAL_WAIT_PID=""
  fi
  if [ -n "$NAV_PID" ]; then
    stop_group "$NAV_PID"
    wait "$NAV_PID" 2>/dev/null || true
  fi
  if [ -n "$SLAM_PID" ]; then
    stop_group "$SLAM_PID"
    wait "$SLAM_PID" 2>/dev/null || true
  fi
  if [ -n "$CONTROL_PID" ]; then
    stop_group "$CONTROL_PID"
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
  if [ -n "$ODOM_PID" ]; then
    stop_group "$ODOM_PID"
    wait "$ODOM_PID" 2>/dev/null || true
  fi
  [ -n "$BRIDGE_PID" ] && stop_group "$BRIDGE_PID"
  for p in ${TF_PIDS[@]+"${TF_PIDS[@]}"}; do stop_group "$p"; done
  [ -n "$RVIZ_PID" ] && stop_group "$RVIZ_PID"
  if [ -n "$GAZEBO_PID" ]; then
    kill -TERM -"$GAZEBO_PID" 2>/dev/null || kill -TERM "$GAZEBO_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$GAZEBO_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$GAZEBO_PID" 2>/dev/null; then
      kill -KILL -"$GAZEBO_PID" 2>/dev/null || kill -KILL "$GAZEBO_PID" 2>/dev/null || true
    fi
    wait "$GAZEBO_PID" 2>/dev/null || true
    say "      Gazebo stopped."
  fi
  for p in $(pgrep -f "lunar_world.sdf" 2>/dev/null || true); do
    kill -9 "$p" 2>/dev/null || true
  done
  echo "" >> "$LOG_FILE"
  echo "clean shutdown at $(date -u +%FT%TZ)" >> "$LOG_FILE"
  say "Launch D environment cleanly closed."
  say "Log: $LOG_FILE"
  say "============================================================"
  exit "$rc"
}
abort() {
  say "ERROR: $*"
  OVERALL="FAIL"
  shutdown 1
}
trap 'shutdown 130' INT TERM

echo "============================================================"
echo "                 LUNABOT V4"
echo "                 PHASE D"
echo "                 AUTONOMOUS NAVIGATION"
echo "============================================================"
echo ""
say "Launch mode: $([ "$HEADLESS" = 1 ] && echo HEADLESS || echo GUI) | demo=$([ "$DEMO" = 1 ] && echo yes || echo no) | evidence=$([ "$EVIDENCE" = 1 ] && echo yes || echo no)"

echo "[1/14] Checking Phase A baseline + Phase D files....."
missing=0
for f in "$WORLD_PATH" "$MODEL_PATH" "$WASD_PATH" "$CONTROL_PATH" "$ODOM_PATH" "$SLAM_CONFIG" "$NAV_PATH" \
         "$RVIZ_CONFIG" "$WORLD_DIR/meshes/lunar_terrain.obj" \
         "$WORLD_DIR/meshes/lunar_terrain_collision.obj"; do
  if [ ! -f "$f" ]; then
    say "      MISSING: $f"
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "      ERROR: required Phase A/D files are missing."
  exit 1
fi
echo "      world, rover, terrain, teleop, controller, monitor, planner, rviz: OK"
log "[1/14] files OK"

echo "[2/14] Checking ROS 2 / Gazebo environment........"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "      ERROR: ROS 2 Humble not found at /opt/ros/humble"
  echo "      Install: https://docs.ros.org/en/humble/Installation.html"
  log "ERROR: ROS 2 Humble not found"
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo "      ERROR: ros2 CLI not found after sourcing."
  exit 2
fi
IGN=""
MSGNS=""
if command -v ign >/dev/null 2>&1; then
  IGN="ign"; MSGNS="ignition.msgs"
elif command -v gz >/dev/null 2>&1; then
  IGN="gz"; MSGNS="gz.msgs"
else
  echo "      ERROR: no Gazebo Sim found (neither 'ign' nor 'gz')."
  exit 2
fi
if ros2 pkg prefix ros_ign_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_ign_bridge"
elif ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  BRIDGE_PKG="ros_gz_bridge"
fi
if [ -z "$BRIDGE_PKG" ]; then
  echo "      ERROR: no bridge package found (ros_ign_bridge / ros_gz_bridge)."
  exit 2
fi
if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "      ERROR: slam_toolbox not found."
  echo "      Install: sudo apt install ros-humble-slam-toolbox"
  exit 2
fi
if ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  MAP_SAVER_AVAILABLE=1
  MAP_SAVER_STATUS="available"
else
  MAP_SAVER_STATUS="not installed (map saving will be skipped)"
fi
echo "      ROS 2 Humble: OK | Gazebo Sim: $IGN ($MSGNS) | bridge: $BRIDGE_PKG"
echo "      SLAM system: slam_toolbox | map saver: $MAP_SAVER_STATUS"
log "[2/14] environment OK ($IGN/$MSGNS, $BRIDGE_PKG, slam_toolbox)"

echo "[3/14] Checking for stale Phase A/D processes......."
stale="$(pgrep -f "lunar_world.sdf" 2>/dev/null || true)"
if [ -n "$stale" ]; then
  echo "      Killing stale Gazebo for lunar_world (PIDs: $stale)"
  kill -9 $stale 2>/dev/null || true
  sleep 2
fi
for pattern in "ros_ign_bridge.*parameter_bridge" "ros_gz_bridge.*parameter_bridge" \
               "scripts/control_odometry.py" "scripts/odometry_monitor.py" "scripts/astar_navigation.py" "slam_toolbox.*online_async"; do
  for p in $(pgrep -f "$pattern" 2>/dev/null || true); do
    kill -TERM "$p" 2>/dev/null || true
  done
done
sleep 1
echo "      clean state: OK"
log "[3/14] clean state OK"

echo "[4/14] Starting Gazebo lunar world.................."
export GZ_SIM_RESOURCE_PATH="$WORLD_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
: > "$EVIDENCE_DIR/gazebo.log"
if [ "$HEADLESS" = "1" ]; then
  setsid "$IGN" gazebo -s -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
else
  setsid "$IGN" gazebo -v 3 "$WORLD_PATH" >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
fi
GAZEBO_PID=$!
ready=0
for _ in $(seq 1 90); do
  if timeout 3 "$IGN" service -s "/world/$WORLD_NAME/info" \
       --reqtype "$MSGNS.Empty" --reptype "$MSGNS.World" >/dev/null 2>&1; then
    ready=1
    break
  fi
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    tail -20 "$EVIDENCE_DIR/gazebo.log" | sed 's/^/      | /'
    abort "Gazebo exited during startup; see $EVIDENCE_DIR/gazebo.log"
  fi
  sleep 1
done
[ "$ready" -eq 1 ] || abort "Gazebo did not become ready within 90 s"
control_resp="$(timeout 10 "$IGN" service -s "/world/$WORLD_NAME/control" \
  --reqtype "$MSGNS.WorldControl" --reptype "$MSGNS.Boolean" \
  --timeout 5000 --req 'pause: false' 2>>"$EVIDENCE_DIR/gazebo.log")"
case "$control_resp" in
  *true*) echo "      Gazebo simulation unpaused";;
  *) abort "could not unpause Gazebo simulation: $control_resp";;
esac
echo "      Gazebo running (PID $GAZEBO_PID), lunar_world loaded"
log "[4/14] gazebo OK and unpaused (pid $GAZEBO_PID)"

echo "[5/14] Spawning LunaBot V4.........................."
spawn_resp="$(timeout 20 "$IGN" service -s "/world/$WORLD_NAME/create" \
  --reqtype "$MSGNS.EntityFactory" --reptype "$MSGNS.Boolean" \
  --timeout 15000 \
  --req "sdf_filename: \"$MODEL_PATH\", name: \"lunabot_v4\", pose: {position: {x: 0.0, y: 0.0, z: $SPAWN_Z}}" \
  2>>"$EVIDENCE_DIR/gazebo.log")"
case "$spawn_resp" in
  *true*) echo "      LunaBot V4 spawned at (0.0, 0.0, $SPAWN_Z)";;
  *) abort "spawn failed: $spawn_resp";;
esac
log "[5/14] spawn OK (z=$SPAWN_Z)"

echo "[6/14] Starting ROS 2 <-> Gazebo bridge............."
: > "$EVIDENCE_DIR/bridge.log"
setsid ros2 run "$BRIDGE_PKG" parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist]$MSGNS.Twist" \
  "/clock@rosgraph_msgs/msg/Clock[$MSGNS.Clock" \
  "/lunabot/camera/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/depth/image_raw@sensor_msgs/msg/Image[$MSGNS.Image" \
  "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[$MSGNS.LaserScan" \
  "/lunabot/imu@sensor_msgs/msg/Imu[$MSGNS.IMU" \
  "/lunabot/odom@nav_msgs/msg/Odometry[$MSGNS.Odometry" \
  "/lunabot/joint_states@sensor_msgs/JointState[$MSGNS.Model" \
  "/tf@tf2_msgs/msg/TFMessage[$MSGNS.Pose_V" \
  "/lunabot/steer/front_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/front_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_left@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/steer/rear_right@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/pan@std_msgs/msg/Float64]$MSGNS.Double" \
  "/lunabot/mast/tilt@std_msgs/msg/Float64]$MSGNS.Double" \
  >> "$EVIDENCE_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3
kill -0 "$BRIDGE_PID" 2>/dev/null || abort "bridge exited; see $EVIDENCE_DIR/bridge.log"
echo "      bridge running (PID $BRIDGE_PID), 16 mappings"
log "[6/14] bridge OK (pid $BRIDGE_PID)"

echo "[7/14] Starting static TF (sensor frames)............"
TF_SPECS=(
  "chassis|sensor_head|0.18 0 0.85 0 0 0"
  "sensor_head|rgb_camera|0.14 0 0 0 0 0"
  "sensor_head|depth_camera|0.14 0 -0.04 0 0 0"
  "sensor_head|lidar|0.02 0 0.09 0 0.5 0"
  "sensor_head|lunabot_v4/sensor_head/lidar|0.02 0 0.09 0 0.5 0"
)
for spec in "${TF_SPECS[@]}"; do
  IFS='|' read -r parent child xyzrpy <<< "$spec"
  setsid ros2 run tf2_ros static_transform_publisher $xyzrpy "$parent" "$child" \
    >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  TF_PIDS+=("$!")
done
echo "      5 static transforms published (including scoped Gazebo LaserScan frame)"
log "[7/14] static TF OK"

echo "[8/14] Starting Phase D control layer................"
: > "$EVIDENCE_DIR/control.log"
setsid python3 "$CONTROL_PATH" \
  --input-topic /cmd_vel_in --output-topic /cmd_vel \
  --max-linear 0.45 --max-angular 1.0 \
  --max-linear-accel 0.4 --max-angular-accel 0.8 \
  --watchdog-sec 0.5 --rate 30 \
  >> "$EVIDENCE_DIR/control.log" 2>&1 &
CONTROL_PID=$!
sleep 2
kill -0 "$CONTROL_PID" 2>/dev/null || abort "control node exited; see $EVIDENCE_DIR/control.log"
echo "      control running (PID $CONTROL_PID): /cmd_vel_in -> /cmd_vel"
log "[8/14] control OK (pid $CONTROL_PID)"

echo "[9/14] Starting odometry monitor...................."
: > "$EVIDENCE_DIR/odometry_monitor.log"
setsid python3 "$ODOM_PATH" --evidence-dir "$EVIDENCE_DIR" \
  >> "$EVIDENCE_DIR/odometry_monitor.log" 2>&1 &
ODOM_PID=$!
sleep 2
kill -0 "$ODOM_PID" 2>/dev/null || abort "odometry monitor exited; see $EVIDENCE_DIR/odometry_monitor.log"
echo "      monitor running (PID $ODOM_PID), recording /lunabot/odom"
log "[9/14] odometry monitor OK (pid $ODOM_PID)"

echo "[10/14] Starting slam_toolbox mapping................"
: > "$EVIDENCE_DIR/slam.log"
setsid ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$SLAM_CONFIG" use_sim_time:=true \
  >> "$EVIDENCE_DIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 4
kill -0 "$SLAM_PID" 2>/dev/null || abort "slam_toolbox exited; see $EVIDENCE_DIR/slam.log"
echo "      slam_toolbox running (PID $SLAM_PID), mapping /lunabot/lidar/scan"
log "[10/14] slam_toolbox OK (pid $SLAM_PID)"

echo "[11/14] Starting A* autonomous navigation..........."
: > "$EVIDENCE_DIR/navigation.log"
if [ "$DEMO" = "1" ]; then
  AUTO_GOAL=true
fi
if [ "$EVIDENCE" = "1" ] && [ "$DEMO" = "1" ]; then
  timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_before_navigation.txt" || true
fi
setsid python3 "$NAV_PATH" --ros-args \
  -p auto_goal:="$AUTO_GOAL" \
  -p map_topic:=/map -p goal_topic:=/goal_pose \
  -p path_topic:=/plan -p cmd_topic:=/cmd_vel_in \
  -p odom_topic:=/lunabot/odom \
  -p status_topic:=/lunabot/navigation/status \
  -p map_frame:=map -p unknown_is_obstacle:=true \
  -p inflation_radius:=0.25 -p auto_goal_distance:=1.5 \
  >> "$EVIDENCE_DIR/navigation.log" 2>&1 &
NAV_PID=$!
sleep 3
kill -0 "$NAV_PID" 2>/dev/null || abort "A* navigation exited; see $EVIDENCE_DIR/navigation.log"
echo "      A* navigation running (PID $NAV_PID), /plan -> /cmd_vel_in"
log "[11/14] A* navigation OK (pid $NAV_PID, auto_goal=$AUTO_GOAL)"

if [ "$HEADLESS" = "1" ]; then
  echo "[12/14] RViz2 (skipped - HEADLESS).................. OK"
  log "[12/14] rviz skipped (headless)"
else
  echo "[12/14] Starting RViz2.............................."
  setsid rviz2 -d "$RVIZ_CONFIG" -f map >> "$EVIDENCE_DIR/gazebo.log" 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "      RViz2 running (PID $RVIZ_PID), fixed frame: map"
    log "[12/14] rviz OK (pid $RVIZ_PID)"
  else
    echo "      WARNING: RViz2 exited at startup (continuing without it)."
    log "[12/14] rviz FAILED to start"
    OVERALL="FAIL"
  fi
fi

echo "[13/14] Runtime validation.........................."
wait_for_goal() {
  local status_file="$EVIDENCE_DIR/goal_wait_status.txt"
  rm -f "$status_file"
  echo "      waiting for A* GOAL_REACHED..."
  setsid timeout 180 ros2 topic echo /lunabot/navigation/status \
    --qos-reliability best_effort --qos-durability transient_local \
    > "$status_file" 2>/dev/null &
  GOAL_WAIT_PID=$!
  for _ in $(seq 1 180); do
    if grep -q "GOAL_REACHED" "$status_file" 2>/dev/null; then
      stop_group "$GOAL_WAIT_PID"
      wait "$GOAL_WAIT_PID" 2>/dev/null || true
      GOAL_WAIT_PID=""
      echo "      A* goal reached: PASS"
      log "validation PASS: A* GOAL_REACHED"
      return 0
    fi
    if ! kill -0 "$GOAL_WAIT_PID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  stop_group "$GOAL_WAIT_PID"
  wait "$GOAL_WAIT_PID" 2>/dev/null || true
  GOAL_WAIT_PID=""
  echo "      A* goal reached: FAIL"
  log "validation FAIL: A* did not reach goal within 180 s"
  OVERALL="FAIL"
  return 1
}

type_ok() {
  local label="$1"
  local topic="$2"
  local expected="$3"
  local value=""
  for _ in 1 2 3; do
    value="$(timeout 10 ros2 topic type "$topic" 2>/dev/null || true)"
    if printf '%s\n' "$value" | grep -qx "$expected"; then
      echo "      $label: PASS"
      log "validation PASS: type $topic=$expected"
      return 0
    fi
    sleep 1
  done
  echo "      $label: FAIL"
  log "validation FAIL: type $topic was [$value]"
  OVERALL="FAIL"
  return 1
}

topic_ok() {
  local sample=""
  local durability="volatile"
  case "$2" in
    /map|/plan|/lunabot/navigation/status) durability="transient_local" ;;
  esac
  if sample="$(timeout 30 ros2 topic echo "$2" --qos-reliability best_effort \
      --qos-durability "$durability" --once 2>/dev/null)" && [ -n "$sample" ]; then
    if [ "$2" = "/map" ] && { [[ "$sample" != *"info:"* ]] || [[ "$sample" != *"data:"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/plan" ] && { [[ "$sample" != *"poses:"* ]] || [[ "$sample" == *"poses: []"* ]]; }; then
      sample=""
    fi
    if [ "$2" = "/goal_pose" ] && [[ "$sample" != *"pose:"* ]]; then
      sample=""
    fi
    if [ "$2" = "/lunabot/navigation/status" ] && [[ "$sample" != *"data:"* ]]; then
      sample=""
    fi
  fi
  if [ -n "$sample" ]; then
    echo "      $1: PASS"
    log "validation PASS: $2"
  else
    echo "      $1: FAIL"
    log "validation FAIL: $2"
    OVERALL="FAIL"
    if [ "$2" = "/lunabot/imu" ]; then
      timeout 10 "$IGN" topic -l 2>/dev/null \
        | grep -iE 'imu|inertial' > "$EVIDENCE_DIR/imu_gazebo_topics.txt" || true
      timeout 8 "$IGN" topic -e -t "/lunabot/imu" 2>/dev/null \
        > "$EVIDENCE_DIR/imu_gazebo_sample.txt" || true
      timeout 10 ros2 topic info /lunabot/imu --verbose 2>/dev/null \
        > "$EVIDENCE_DIR/imu_topic_info.txt" || true
      {
        echo "--- bridge IMU diagnostics ---"
        grep -iE 'imu|error|fail|warn' "$EVIDENCE_DIR/bridge.log" 2>/dev/null || true
        echo "--- Gazebo IMU diagnostics ---"
        grep -iE 'imu|sensor|error|fail|warn' "$EVIDENCE_DIR/gazebo.log" 2>/dev/null | tail -80 || true
      } > "$EVIDENCE_DIR/imu_diagnostics.txt"
      echo "      IMU diagnostics written to $EVIDENCE_DIR/imu_*"
    fi
  fi
}
tf_ok() {
  for _ in 1 2 3; do
    if timeout 10 ros2 run tf2_ros tf2_echo "$2" "$3" 2>/dev/null | grep -qm1 "Translation:"; then
      echo "      $1: PASS"
      log "validation PASS: TF $2->$3"
      return 0
    fi
    sleep 1
  done
  echo "      $1: FAIL"
  log "validation FAIL: TF $2->$3"
  OVERALL="FAIL"
  return 1
}
kill -0 "$GAZEBO_PID" 2>/dev/null && echo "      Gazebo process alive: PASS" || { echo "      Gazebo process alive: FAIL"; OVERALL="FAIL"; }
map_type=""
for _ in 1 2 3; do
  map_type="$(timeout 10 ros2 topic type /map 2>/dev/null || true)"
  if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
    break
  fi
  sleep 1
done
if printf '%s\n' "$map_type" | grep -qx 'nav_msgs/msg/OccupancyGrid'; then
  echo "      /map type nav_msgs/msg/OccupancyGrid: PASS"
  log "validation PASS: /map type nav_msgs/msg/OccupancyGrid"
else
  echo "      /map type nav_msgs/msg/OccupancyGrid: FAIL"
  log "validation FAIL: /map type was [$map_type]"
  OVERALL="FAIL"
fi
topic_ok "topic /map"                         "/map"
tf_ok "TF map -> odom" "map" "odom"
topic_ok "topic /clock"                       "/clock"
topic_ok "topic /lunabot/odom"                "/lunabot/odom"
topic_ok "topic /lunabot/control/status"      "/lunabot/control/status"
topic_ok "topic /lunabot/odometry/status"     "/lunabot/odometry/status"
topic_ok "topic /lunabot/camera/image_raw"   "/lunabot/camera/image_raw"
topic_ok "topic /lunabot/lidar/scan"          "/lunabot/lidar/scan"
topic_ok "topic /lunabot/imu"                 "/lunabot/imu"
tf_ok "TF odom -> chassis" "odom" "chassis"
tf_ok "TF chassis -> sensor_head" "chassis" "sensor_head"
tf_ok "TF sensor_head -> scoped LaserScan frame" "sensor_head" "lunabot_v4/sensor_head/lidar"
type_ok "type /goal_pose geometry_msgs/PoseStamped" "/goal_pose" "geometry_msgs/msg/PoseStamped"
type_ok "type /plan nav_msgs/Path" "/plan" "nav_msgs/msg/Path"
type_ok "type navigation status std_msgs/String" "/lunabot/navigation/status" "std_msgs/msg/String"
topic_ok "topic /goal_pose"                    "/goal_pose"
topic_ok "topic /plan"                         "/plan"
topic_ok "topic /lunabot/navigation/status"    "/lunabot/navigation/status"
topic_ok "topic /cmd_vel_in"                  "/cmd_vel_in"
if [ "$DEMO" = "1" ]; then
  wait_for_goal || true
fi

if [ "$EVIDENCE" = "1" ]; then
  echo ""
  echo "Recording Phase D runtime evidence -> $EVIDENCE_DIR"
  timeout 20 ros2 topic list 2>/dev/null > "$EVIDENCE_DIR/topics.txt"
  timeout 15 ros2 topic echo /lunabot/control/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/control_status.txt"
  timeout 15 ros2 topic echo /lunabot/odometry/status --once 2>/dev/null \
    > "$EVIDENCE_DIR/odometry_status.txt"
  timeout 15 ros2 topic echo /lunabot/navigation/status --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/navigation_status.txt"
  timeout 15 ros2 topic echo /goal_pose --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/goal_pose.txt"
  timeout 15 ros2 topic echo /plan --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/plan.txt"
  timeout 25 ros2 topic echo /lunabot/odom --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/odom_sample.txt"
  timeout 25 ros2 topic echo /lunabot/imu --qos-reliability best_effort --once 2>/dev/null \
    > "$EVIDENCE_DIR/imu_sample.txt"
  timeout 25 ros2 run tf2_ros tf2_echo odom chassis 2>/dev/null \
    > "$EVIDENCE_DIR/tf_odom_chassis.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo map odom 2>/dev/null \
    > "$EVIDENCE_DIR/tf_map_odom.txt" || true
  timeout 25 ros2 run tf2_ros tf2_echo sensor_head lunabot_v4/sensor_head/lidar 2>/dev/null \
    > "$EVIDENCE_DIR/tf_lidar_scoped.txt" || true
  timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
    > "$EVIDENCE_DIR/map_sample.txt"
  if [ "$DEMO" = "1" ]; then
    timeout 20 ros2 topic echo /map --qos-reliability best_effort --qos-durability transient_local --once 2>/dev/null \
      > "$EVIDENCE_DIR/map_after_navigation.txt" || true
    if [ -s "$EVIDENCE_DIR/map_before_navigation.txt" ] && \
       [ -s "$EVIDENCE_DIR/map_after_navigation.txt" ] && \
       ! cmp -s "$EVIDENCE_DIR/map_before_navigation.txt" "$EVIDENCE_DIR/map_after_navigation.txt"; then
      echo "      live map updates during autonomous navigation: PASS"
      log "validation PASS: map changed during A* navigation"
    else
      echo "      live map updates during autonomous navigation: FAIL"
      log "validation FAIL: map did not produce distinct A* before/after samples"
      OVERALL="FAIL"
    fi
  fi
  if [ "$MAP_SAVER_AVAILABLE" = "1" ]; then
    echo "      final map saver will run during clean shutdown"
    echo "map saver scheduled for clean shutdown" > "$EVIDENCE_DIR/map_saver.log"
  else
    echo "      map saver unavailable; map_sample.txt is retained"
    echo "map saver skipped: install ros-humble-nav2-map-server to write PGM/YAML" \
      > "$EVIDENCE_DIR/map_saver.log"
  fi
  echo "      evidence written (map, TF map->odom, odom, SLAM log)"
  log "[13/14] evidence recorded"
fi

echo ""
echo "------------------------------------------------------------"
echo "PHASE STATUS"
echo "------------------------------------------------------------"
printf "Environment       : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "RUNNING (headless)" || echo "RUNNING" )"
echo "LunaBot           : RUNNING (Phase A baseline)"
echo "Control           : RUNNING (/cmd_vel_in -> /cmd_vel)"
echo "SLAM              : RUNNING (slam_toolbox mapping)"
echo "Navigation        : RUNNING (A* planner + follower)"
echo "Map               : RUNNING (/map + map->odom TF)"
echo "Watchdog          : RUNNING (0.5 s timeout)"
echo "Odometry          : RUNNING (/lunabot/odom monitor)"
echo "Sensors           : RUNNING (camera, depth, lidar, imu)"
echo "Bridge            : RUNNING (16 mappings)"
echo "TF                : RUNNING (odom->chassis + 5 static)"
printf "RViz2             : %s\n" "$( [ "$HEADLESS" = 1 ] && echo "skipped (headless)" || echo "RUNNING" )"
echo ""
echo "------------------------------------------------------------"
echo "AVAILABLE OUTPUTS"
echo "------------------------------------------------------------"
echo "  /cmd_vel_in                  geometry_msgs/Twist (control input)"
echo "  /cmd_vel                     geometry_msgs/Twist (smoothed output)"
echo "  /lunabot/control/status      std_msgs/String (watchdog + limits)"
echo "  /lunabot/odom                nav_msgs/Odometry (DiffDrive source)"
echo "  /lunabot/odometry/status     std_msgs/String (quality monitor)"
echo "  /map                         nav_msgs/OccupancyGrid (SLAM map)"
echo "  /goal_pose                  geometry_msgs/PoseStamped (navigation goal)"
echo "  /plan                       nav_msgs/Path (A* path)"
echo "  /lunabot/navigation/status std_msgs/String (planner state)"
echo "  map -> odom                  TF (SLAM localization)"
echo "  evidence/phase-d-launch-d/map_sample.txt"
echo "  evidence/phase-d-launch-d/phase_d_map.yaml/.pgm"
echo "  evidence/phase-d-launch-d/odometry_samples.csv"
echo "  evidence/phase-d-launch-d/odometry_report.txt"
echo "  evidence/phase-d-launch-d/demo_drive_result.txt"
echo "  evidence/phase-d-launch-d/diag_drive.csv"
echo ""
echo "------------------------------------------------------------"
echo "VALIDATION"
echo "------------------------------------------------------------"
echo "  - Phase A lunar world, rover, sensors, control and odometry preserved"
echo "  - one SLAM system: slam_toolbox"
echo "  - A* planner consumes /map and publishes /plan"
echo "  - navigation commands remain behind the Phase B controller"
echo "  - /map and map->odom localization outputs verified"
echo "  - command safety layer clamps and ramps /cmd_vel"
echo "  - watchdog stops output after 0.5 s without input"
echo "  - live odometry monitor checks frames, rate and continuity"
echo "  - overall startup validation: $OVERALL"
echo ""
echo "============================================================"
log "[14/14] status printed (overall $OVERALL)"
if [ "$OVERALL" != "PASS" ]; then
  EXIT_CODE=1
fi

if [ "$DEMO" = "1" ]; then
  echo ""
  echo "============================================================"
  echo " AUTOMATED A* NAVIGATION TEST (DEMO mode)"
  echo "============================================================"
  sleep 1
  if [ "$OVERALL" != "PASS" ]; then
    EXIT_CODE=1
  fi
  if ! save_final_map; then
    EXIT_CODE=1
  fi
  echo ""
  echo "============================================================"
  printf "PHASE D RUN COMPLETE - overall result: %s\n" "$OVERALL"
  echo "============================================================"
  shutdown "$EXIT_CODE"
else
  echo ""
  echo "============================================================"
  echo " Phase D autonomous navigation is running."
  echo ""
  echo " A* auto goal: $AUTO_GOAL"
  echo " RViz can publish a replacement goal on /goal_pose."
  echo " Press Ctrl+C to stop Phase D safely."
  echo "============================================================"
  while true; do sleep 1; done
fi
```

## `scripts/wasd_teleop.py`

```python
"""
LunaBot V4 - Phase A - WASD teleoperation + automated demo drive test
=======================================================================

Interactive mode (default):
    W forward | S reverse | A turn left | D turn right | Space stop | Q quit
    Publishes geometry_msgs/msg/Twist on /cmd_vel.

Demo mode (headless / automated validation):
    python3 wasd_teleop.py --demo [evidence_dir]
    Sequence (SIMULATION time, via /clock - robust to slow rendering):
        forward 3 s -> stop 0.5 s -> turn left 3 s -> stop 0.5 s
    Reports odometry deltas and writes to <evidence_dir>:
        demo_drive_result.txt   summary + PASS/FAIL
        diag_drive.csv          per-sample diagnostics:
                                sim time, commanded vs measured velocity,
                                rover pose, actual wheel joint velocities
                                (used to separate wheel slip / traction
                                issues from simulation-time lag)
    Exit code 0 = movement verified.

Speeds respect the DiffDrive plugin limits in model.sdf
(max_linear_velocity 0.45 m/s, max_angular_velocity 1.0 rad/s).
"""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

MAX_LINEAR = 0.45   # m/s  (DiffDrive plugin limit in model.sdf)
MAX_ANGULAR = 1.0   # rad/s
WHEEL_R = 0.17      # m (model.sdf)
WHEEL_JOINTS = (
    'left_front_wheel_joint', 'left_middle_wheel_joint', 'left_rear_wheel_joint',
    'right_front_wheel_joint', 'right_middle_wheel_joint', 'right_rear_wheel_joint',
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TeleopNode(Node):
    def __init__(self, topic='/cmd_vel'):
        super().__init__('wasd_teleop')
        self.topic = topic
        self.pub = self.create_publisher(Twist, topic, 10)
        self.odom = None
        self.joint_vel = {}
        self.sim_t = None
        self.cmd = (0.0, 0.0)          # last commanded (v, wz)
        self.create_subscription(Odometry, '/lunabot/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/lunabot/joint_states',
                                 self._js_cb, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._clock_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, msg):
        self.odom = msg

    def _js_cb(self, msg):
        for name, vel in zip(msg.name, msg.velocity):
            if name in WHEEL_JOINTS:
                self.joint_vel[name] = vel

    def _clock_cb(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def cmd_of(self, v, wz):
        self.cmd = (v, wz)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = wz
        self.pub.publish(msg)

    def snapshot(self):
        """(x, y, yaw, odom_v, odom_wz, mean left wheel omega, mean right)"""
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        t = self.odom.twist.twist
        lv = [self.joint_vel[j] for j in WHEEL_JOINTS[:3] if j in self.joint_vel]
        rv = [self.joint_vel[j] for j in WHEEL_JOINTS[3:] if j in self.joint_vel]
        return (p.x, p.y, yaw_from_quaternion(q),
                t.linear.x, t.angular.z,
                sum(lv) / len(lv) if lv else 0.0,
                sum(rv) / len(rv) if rv else 0.0)


def run_demo(node, evidence_dir):
    """Run the controlled drive test while spinning rclpy in this thread.

    rclpy's global executor is not safe to lazily construct from a background
    thread on ROS 2 Humble. Synchronous spin_once keeps command publication and
    subscription callbacks in one executor context.
    """
    phase_label = 'Phase B' if node.topic == '/cmd_vel_in' else 'Phase A'

    def write_failure(reason):
        lines = [
            f"LunaBot V4 - {phase_label} - automated demo drive result",
            "====================================================",
            f"time                 : {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
            f"failure_reason       : {reason}",
            "result                : FAIL",
        ]
        for line in lines:
            print(line, flush=True)
        if evidence_dir:
            os.makedirs(evidence_dir, exist_ok=True)
            with open(os.path.join(evidence_dir, "demo_drive_result.txt"), "w") as fh:
                fh.write("\n".join(lines) + "\n")
        return 1

    print("Waiting for /lunabot/odom and /clock ...", flush=True)
    deadline = time.monotonic() + 15.0
    while (node.odom is None or node.sim_t is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.odom is None or node.sim_t is None:
        return write_failure("odom/clock not received within 15 s")
    print("Odometry + sim time received. Starting drive test (sim-time based).",
          flush=True)

    csv_path = os.path.join(evidence_dir, "diag_drive.csv") if evidence_dir else None
    csv = open(csv_path, "w") if csv_path else None
    if csv:
        csv.write("t_wall,t_sim,cmd_v,cmd_wz,odom_v,odom_wz,"
                  "x,y,yaw,wheel_omega_left,wheel_omega_right\n")

    def sample():
        s = node.snapshot()
        if s and csv:
            csv.write(f"{time.time():.2f},{node.sim_t:.3f},"
                      f"{node.cmd[0]:.3f},{node.cmd[1]:.3f},"
                      f"{s[3]:.3f},{s[4]:.3f},{s[0]:.3f},{s[1]:.3f},{s[2]:.3f},"
                      f"{s[5]:.3f},{s[6]:.3f}\n")
            csv.flush()

    def drive(v, wz, sim_dur):
        """Drive until SIMULATION time advances by sim_dur seconds."""
        start = node.sim_t
        deadline = time.monotonic() + 120.0
        while (node.sim_t - start < sim_dur and
               time.monotonic() < deadline):
            node.cmd_of(v, wz)
            rclpy.spin_once(node, timeout_sec=0.02)
            sample()
        node.cmd_of(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        return node.sim_t - start >= sim_dur

    s0 = node.snapshot()
    if s0 is None:
        if csv:
            csv.close()
        return write_failure("odometry snapshot unavailable after startup")
    if not drive(MAX_LINEAR, 0.0, 3.0):
        if csv:
            csv.close()
        return write_failure("simulation time stopped during forward phase")
    s1 = node.snapshot()
    drive(0.0, 0.0, 0.5)
    if not drive(0.0, 0.6, 3.0):
        if csv:
            csv.close()
        return write_failure("simulation time stopped during turn phase")
    s2 = node.snapshot()
    drive(0.0, 0.0, 0.5)
    if csv:
        csv.close()

    dist = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
    dyaw = math.atan2(math.sin(s2[2] - s1[2]), math.cos(s2[2] - s1[2]))
    eff_lin = dist / 1.10
    eff_ang = abs(dyaw) / 1.54
    ok = dist > 0.6 and abs(dyaw) > 0.8

    lines = [
        f"LunaBot V4 - {phase_label} - automated demo drive result",
        "====================================================",
        f"time                 : {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
        f"forward 3 s(sim) @ {MAX_LINEAR} m/s : distance = {dist:.3f} m (expect > 0.6 m)",
        f"turn 3 s(sim) @ 0.6 rad/s          : yaw delta  = {dyaw:+.3f} rad (expect > 0.8 rad)",
        f"forward efficiency : {eff_lin * 100:.0f}% of ideal",
        f"turn efficiency    : {eff_ang * 100:.0f}% of ideal",
        f"start pose (x,y,yaw)  : {s0[0]:.3f}, {s0[1]:.3f}, {s0[2]:.3f}",
        f"after forward         : {s1[0]:.3f}, {s1[1]:.3f}, {s1[2]:.3f}",
        f"after turn            : {s2[0]:.3f}, {s2[1]:.3f}, {s2[2]:.3f}",
        f"diag csv             : {csv_path or '(none)'}",
        f"result                : {'PASS' if ok else 'FAIL'}",
    ]
    for line in lines:
        print(line, flush=True)
    if evidence_dir:
        os.makedirs(evidence_dir, exist_ok=True)
        with open(os.path.join(evidence_dir, "demo_drive_result.txt"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
    return 0 if ok else 1

def run_interactive(node):
    import select
    import termios
    import tty

    if not sys.stdin.isatty():
        print("Interactive teleop needs a terminal. Use --demo for headless runs.")
        return 1

    settings = termios.tcgetattr(sys.stdin)
    speed = MAX_LINEAR
    turn = 0.6

    def get_key():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    print("WASD to drive, Space to stop, Q to quit", flush=True)
    try:
        while True:
            key = get_key()
            msg = Twist()
            if key == 'w':
                msg.linear.x = speed
                print("Moving forward", flush=True)
            elif key == 's':
                msg.linear.x = -speed
                print("Moving backward", flush=True)
            elif key == 'a':
                msg.angular.z = turn
                print("Turning left", flush=True)
            elif key == 'd':
                msg.angular.z = -turn
                print("Turning right", flush=True)
            elif key == ' ':
                print("Stopping", flush=True)
            elif key in ('q', 'Q', '\x03'):
                break
            node.pub.publish(msg)
    except Exception as e:
        print(e, flush=True)
    finally:
        node.pub.publish(Twist())     # always stop on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return 0


def main():
    ap = argparse.ArgumentParser(description="LunaBot V4 WASD teleop / demo drive")
    ap.add_argument("--demo", action="store_true",
                    help="run the automated drive test instead of interactive WASD")
    ap.add_argument("--topic", default="/cmd_vel",
                    help="Twist command topic (Phase A: /cmd_vel; Phase B: /cmd_vel_in)")
    ap.add_argument("evidence_dir", nargs="?", default="",
                    help="optional evidence dir for --demo results")
    args = ap.parse_args()

    rclpy.init()
    node = TeleopNode(args.topic)
    rc = 1
    try:
        if args.demo:
            rc = run_demo(node, args.evidence_dir)
        else:
            rc = run_interactive(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
```

## `scripts/control_odometry.py`

```python
"""
LunaBot V4 Phase B control layer.

INPUT
  /cmd_vel_in  geometry_msgs/msg/Twist from teleoperation or a future planner

PROCESS
  Clamp commands to the rover's configured limits, apply acceleration limits,
  and stop automatically when the input watchdog expires.

OUTPUT
  /cmd_vel             geometry_msgs/msg/Twist for the Gazebo DiffDrive plugin
  /lunabot/control/status  std_msgs/msg/String health and watchdog status

This node deliberately does not own the Gazebo bridge or the wheel model. It
is the reusable ROS-side control boundary introduced by Phase B.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class ControlNode(Node):
    def __init__(self, input_topic, output_topic, max_linear, max_angular,
                 max_linear_accel, max_angular_accel, watchdog_sec, rate):
        super().__init__('lunabot_control')
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.max_linear = abs(max_linear)
        self.max_angular = abs(max_angular)
        self.max_linear_accel = abs(max_linear_accel)
        self.max_angular_accel = abs(max_angular_accel)
        self.watchdog_sec = max(0.05, watchdog_sec)
        self.target_v = 0.0
        self.target_w = 0.0
        self.output_v = 0.0
        self.output_w = 0.0
        self.last_input = 0.0
        self.last_tick = time.monotonic()
        self.last_status = 0.0
        self.input_count = 0
        self.output_count = 0

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/lunabot/control/status', 10)
        self.create_subscription(Twist, input_topic, self._input_cb, 10)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'control active: {input_topic} -> {output_topic}; '
            f'watchdog={self.watchdog_sec:.2f}s, '
            f'limits={self.max_linear:.2f}m/s {self.max_angular:.2f}rad/s')

    @staticmethod
    def _clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    @staticmethod
    def _approach(current, target, rate, dt):
        step = max(0.0, rate) * max(0.0, min(dt, 0.25))
        if target > current:
            return min(target, current + step)
        return max(target, current - step)

    def _input_cb(self, msg):
        self.target_v = self._clamp(msg.linear.x, self.max_linear)
        self.target_w = self._clamp(msg.angular.z, self.max_angular)
        self.last_input = time.monotonic()
        self.input_count += 1

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        age = now - self.last_input if self.last_input else float('inf')
        watchdog = age > self.watchdog_sec
        target_v = 0.0 if watchdog else self.target_v
        target_w = 0.0 if watchdog else self.target_w
        self.output_v = self._approach(
            self.output_v, target_v, self.max_linear_accel, dt)
        self.output_w = self._approach(
            self.output_w, target_w, self.max_angular_accel, dt)

        out = Twist()
        out.linear.x = self.output_v
        out.angular.z = self.output_w
        self.cmd_pub.publish(out)
        self.output_count += 1

        if now - self.last_status >= 1.0:
            status = String()
            state = 'WATCHDOG_STOP' if watchdog else 'ACTIVE'
            status.data = (
                f'{state} input={self.input_topic} '
                f'age={age:.3f}s target=({target_v:.3f},{target_w:.3f}) '
                f'output=({self.output_v:.3f},{self.output_w:.3f}) '
                f'clamps=({self.max_linear:.3f},{self.max_angular:.3f}) '
                f'counts=({self.input_count},{self.output_count})')
            self.status_pub.publish(status)
            self.last_status = now


def main():
    parser = argparse.ArgumentParser(description='LunaBot Phase B control node')
    parser.add_argument('--input-topic', default='/cmd_vel_in')
    parser.add_argument('--output-topic', default='/cmd_vel')
    parser.add_argument('--max-linear', type=float, default=0.45)
    parser.add_argument('--max-angular', type=float, default=1.0)
    parser.add_argument('--max-linear-accel', type=float, default=0.4)
    parser.add_argument('--max-angular-accel', type=float, default=0.8)
    parser.add_argument('--watchdog-sec', type=float, default=0.5)
    parser.add_argument('--rate', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ControlNode(
        args.input_topic, args.output_topic, args.max_linear, args.max_angular,
        args.max_linear_accel, args.max_angular_accel, args.watchdog_sec,
        args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## `scripts/odometry_monitor.py`

```python
"""
LunaBot V4 Phase B odometry monitor.

Subscribes to the DiffDrive odometry stream and records an auditable report.
It does not replace the simulator's odometry source; it measures that source.

OUTPUT FILES (when --evidence-dir is supplied)
  odometry_samples.csv  every received odometry sample
  odometry_report.txt   rates, duration, travelled distance, yaw, continuity,
                        frame IDs, and PASS/FAIL quality result
  /lunabot/odometry/status  std_msgs/msg/String live quality status
"""

import argparse
import csv
import math
import os
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class OdometryMonitor(Node):
    def __init__(self, evidence_dir):
        super().__init__('lunabot_odometry_monitor')
        self.evidence_dir = evidence_dir
        if evidence_dir:
            os.makedirs(evidence_dir, exist_ok=True)
        self.status_pub = self.create_publisher(
            String, '/lunabot/odometry/status', 10)
        self.create_subscription(Odometry, '/lunabot/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.samples = 0
        self.first_wall = None
        self.last_wall = None
        self.first_sim = None
        self.last_sim = None
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.total_distance = 0.0
        self.total_yaw = 0.0
        self.max_step = 0.0
        self.max_speed = 0.0
        self.sum_speed = 0.0
        self.frame_ids = set()
        self.child_frame_ids = set()
        self._closed = False
        self.csv_file = None
        self.csv_writer = None
        if evidence_dir:
            self.csv_file = open(
                os.path.join(evidence_dir, 'odometry_samples.csv'),
                'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'wall_time', 'sim_time', 'x', 'y', 'yaw',
                'linear_x', 'angular_z', 'step_distance', 'frame_id',
                'child_frame_id'])
            self.csv_file.flush()
        self.create_timer(1.0, self._publish_status)

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _unwrap(delta):
        return math.atan2(math.sin(delta), math.cos(delta))

    def _odom_cb(self, msg):
        now = time.time()
        sim_time = self._stamp_seconds(msg.header.stamp)
        p = msg.pose.pose.position
        yaw = self._yaw(msg.pose.pose.orientation)
        step = 0.0
        yaw_step = 0.0
        if self.last_x is not None:
            step = math.hypot(p.x - self.last_x, p.y - self.last_y)
            yaw_step = self._unwrap(yaw - self.last_yaw)
            self.total_distance += step
            self.total_yaw += yaw_step
        self.max_step = max(self.max_step, step)
        speed = abs(msg.twist.twist.linear.x)
        self.max_speed = max(self.max_speed, speed)
        self.sum_speed += speed
        self.samples += 1
        self.first_wall = now if self.first_wall is None else self.first_wall
        self.last_wall = now
        self.first_sim = sim_time if self.first_sim is None else self.first_sim
        self.last_sim = sim_time
        self.last_x, self.last_y, self.last_yaw = p.x, p.y, yaw
        if msg.header.frame_id:
            self.frame_ids.add(msg.header.frame_id)
        if msg.child_frame_id:
            self.child_frame_ids.add(msg.child_frame_id)
        if self.csv_writer:
            self.csv_writer.writerow([
                f'{now:.3f}', f'{sim_time:.6f}', f'{p.x:.6f}', f'{p.y:.6f}',
                f'{yaw:.6f}', f'{msg.twist.twist.linear.x:.6f}',
                f'{msg.twist.twist.angular.z:.6f}', f'{step:.6f}',
                msg.header.frame_id, msg.child_frame_id])
            self.csv_file.flush()

    def _quality(self):
        duration = ((self.last_sim - self.first_sim)
                    if self.first_sim is not None and self.last_sim is not None
                    else 0.0)
        rate = (self.samples / duration) if duration > 0 else 0.0
        frames_ok = ('odom' in self.frame_ids and 'chassis' in self.child_frame_ids)
        passed = (self.samples >= 10 and duration >= 1.0 and
                  self.max_step < 0.5 and frames_ok)
        return passed, duration, rate, frames_ok

    def _publish_status(self):
        passed, duration, rate, frames_ok = self._quality()
        msg = String()
        msg.data = (f"{'PASS' if passed else 'WARMING'} samples={self.samples} "
                    f"sim_duration={duration:.2f}s rate={rate:.1f}Hz "
                    f"frames={'OK' if frames_ok else 'CHECK'} "
                    f"max_step={self.max_step:.3f}m")
        self.status_pub.publish(msg)

    def write_report(self):
        if self._closed:
            return
        self._closed = True
        passed, duration, rate, frames_ok = self._quality()
        avg_speed = self.sum_speed / self.samples if self.samples else 0.0
        lines = [
            'LunaBot V4 - Phase B - odometry monitor report',
            '================================================',
            f'generated_utc       : {time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}',
            f'samples             : {self.samples}',
            f'simulation_duration : {duration:.3f} s',
            f'odometry_rate       : {rate:.3f} Hz',
            f'total_distance      : {self.total_distance:.6f} m',
            f'total_yaw_change    : {self.total_yaw:.6f} rad',
            f'max_pose_step       : {self.max_step:.6f} m',
            f'max_abs_linear_speed: {self.max_speed:.6f} m/s',
            f'avg_abs_linear_speed: {avg_speed:.6f} m/s',
            f'frame_ids           : {sorted(self.frame_ids)}',
            f'child_frame_ids     : {sorted(self.child_frame_ids)}',
            f'continuity_frames   : {"PASS" if frames_ok else "FAIL"}',
            'quality_result      : ' + ('PASS' if passed else 'FAIL'),
        ]
        for line in lines:
            self.get_logger().info(line)
        if self.evidence_dir:
            with open(os.path.join(self.evidence_dir, 'odometry_report.txt'), 'w') as fh:
                fh.write('\n'.join(lines) + '\n')
            if self.csv_file:
                self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description='LunaBot Phase B odometry monitor')
    parser.add_argument('--evidence-dir', default='')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = OdometryMonitor(args.evidence_dir)

    def stop(_signum, _frame):
        node.write_report()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## `scripts/astar_navigation.py`

```python
"""LunaBot V4 Phase D: grid A* planner and conservative path follower.

The node intentionally uses the Phase C interfaces instead of Nav2: slam_toolbox
publishes /map and map->odom, DiffDrive publishes /lunabot/odom and odom->chassis,
and this node publishes safe planner input on /cmd_vel_in for the inherited
Phase B controller. Unknown/occupied cells are not traversed by default.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import tf2_ros


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def angle_error(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AStarNavigation(Node):
    """A* planner plus waypoint follower with an auditable ROS contract."""

    def __init__(self) -> None:
        super().__init__('lunabot_astar_navigation')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/lunabot/odom')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_topic', '/cmd_vel_in')
        self.declare_parameter('status_topic', '/lunabot/navigation/status')
        self.declare_parameter('auto_goal', False)
        self.declare_parameter('auto_goal_distance', 1.5)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('inflation_radius', 0.25)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 0.7)
        self.declare_parameter('map_frame', 'map')

        p = self.get_parameter
        self.map_topic = p('map_topic').value
        self.goal_topic = p('goal_topic').value
        self.odom_topic = p('odom_topic').value
        self.path_topic = p('path_topic').value
        self.cmd_topic = p('cmd_topic').value
        self.status_topic = p('status_topic').value
        self.auto_goal = bool(p('auto_goal').value)
        self.auto_goal_distance = float(p('auto_goal_distance').value)
        self.goal_tolerance = float(p('goal_tolerance').value)
        self.occupied_threshold = int(p('occupied_threshold').value)
        self.inflation_radius = float(p('inflation_radius').value)
        self.unknown_is_obstacle = bool(p('unknown_is_obstacle').value)
        self.replan_period = float(p('replan_period').value)
        self.max_linear = float(p('max_linear').value)
        self.max_angular = float(p('max_angular').value)
        self.map_frame = str(p('map_frame').value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback,
            qos_profile_sensor_data)
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, self.path_topic, latched_qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_msg: Optional[OccupancyGrid] = None
        self.odom_msg: Optional[Odometry] = None
        self.goal_msg: Optional[PoseStamped] = None
        self.goal_sent = False
        self.auto_goal_sent = False
        self.manual_goal_seen = False
        self.path_points: list[tuple[float, float]] = []
        self.path_stamp = self.get_clock().now()
        self.last_plan = self.get_clock().now() - Duration(seconds=10.0)
        self.last_status = ''
        self.reached = False
        self.last_waypoint = -1
        self.timer = self.create_timer(1.0 / max(1.0, p('control_rate').value),
                                      self._tick)
        self.publish_status('PLANNER_WAITING_FOR_MAP')

    def publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            self.publish_status('PLANNER_INVALID_MAP')
            return
        self.map_msg = msg
        self.path_stamp = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_msg = msg

    @staticmethod
    def _same_goal(first: PoseStamped, second: PoseStamped) -> bool:
        if (first.header.frame_id or '') != (second.header.frame_id or ''):
            return False
        a, b = first.pose, second.pose
        return (
            abs(a.position.x - b.position.x) < 1e-6 and
            abs(a.position.y - b.position.y) < 1e-6 and
            abs(a.position.z - b.position.z) < 1e-6 and
            abs(a.orientation.x - b.orientation.x) < 1e-6 and
            abs(a.orientation.y - b.orientation.y) < 1e-6 and
            abs(a.orientation.z - b.orientation.z) < 1e-6 and
            abs(a.orientation.w - b.orientation.w) < 1e-6
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        if self.goal_msg is not None and self._same_goal(msg, self.goal_msg):
            return
        self.goal_msg = msg
        self.goal_sent = True
        self.manual_goal_seen = True
        self.reached = False
        self.path_points = []
        self.publish_status(
            f'MANUAL_GOAL_SELECTED frame={msg.header.frame_id or self.map_frame} '
            f'x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}')

    def _transform_pose(self, x: float, y: float, yaw: float,
                        source_frame: str, target_frame: str):
        source_frame = source_frame or target_frame
        if source_frame == target_frame:
            return x, y, yaw
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, Time())
        t = transform.transform.translation
        tf_yaw = yaw_from_quaternion(transform.transform.rotation)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y, angle_error(yaw + tf_yaw)

    def _current_map_pose(self):
        if self.odom_msg is None:
            raise RuntimeError('odom not received')
        pose = self.odom_msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        return self._transform_pose(
            pose.position.x, pose.position.y, yaw,
            self.odom_msg.header.frame_id or 'odom', self.map_frame)

    def _send_auto_goal(self, current) -> None:
        if not self.auto_goal or self.goal_sent:
            return
        x, y, yaw = current
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x + self.auto_goal_distance * math.cos(yaw)
        goal.pose.position.y = y + self.auto_goal_distance * math.sin(yaw)
        q = quaternion_from_yaw(yaw)
        goal.pose.orientation.x, goal.pose.orientation.y = q[0], q[1]
        goal.pose.orientation.z, goal.pose.orientation.w = q[2], q[3]
        self.goal_msg = goal
        self.goal_sent = True
        self.auto_goal_sent = True
        self.goal_pub.publish(goal)
        self.publish_status(
            f'AUTO_GOAL_SENT frame={self.map_frame} '
            f'x={goal.pose.position.x:.2f} y={goal.pose.position.y:.2f}')

    def _world_to_grid(self, x: float, y: float):
        info = self.map_msg.info
        origin = info.origin
        oyaw = yaw_from_quaternion(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        c, s = math.cos(oyaw), math.sin(oyaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(mx / info.resolution)), int(math.floor(my / info.resolution))

    def _grid_to_world(self, cell):
        info = self.map_msg.info
        origin = info.origin
        oyaw = yaw_from_quaternion(origin.orientation)
        mx = (cell[0] + 0.5) * info.resolution
        my = (cell[1] + 0.5) * info.resolution
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (origin.position.x + c * mx - s * my,
                origin.position.y + s * mx + c * my)

    def _cell_is_free(self, cell) -> bool:
        info = self.map_msg.info
        x, y = cell
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            return False
        value = self.map_msg.data[y * info.width + x]
        if value < 0:
            return not self.unknown_is_obstacle
        return value < self.occupied_threshold

    def _safe_cell(self, cell):
        info = self.map_msg.info
        radius = max(0, int(math.ceil(self.inflation_radius / info.resolution)))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius and not self._cell_is_free((cell[0] + dx, cell[1] + dy)):
                    return False
        return True

    def _nearest_free(self, cell):
        if self._safe_cell(cell):
            return cell
        for radius in range(1, 15):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self._safe_cell(candidate):
                        return candidate
        return None

    def _astar(self, start, goal):
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                     (1, 1), (1, -1), (-1, 1), (-1, -1))
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._safe_cell(nxt):
                    continue
                step = math.sqrt(2.0) if dx and dy else 1.0
                tentative = g_score[current] + step
                if tentative >= g_score.get(nxt, float('inf')):
                    continue
                came_from[nxt] = current
                g_score[nxt] = tentative
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_set, (tentative + heuristic, nxt))
        return []

    def _plan(self, current):
        if self.map_msg is None or self.goal_msg is None:
            return False
        try:
            goal_pose = self.goal_msg.pose
            goal_yaw = yaw_from_quaternion(goal_pose.orientation)
            goal = self._transform_pose(
                goal_pose.position.x, goal_pose.position.y, goal_yaw,
                self.goal_msg.header.frame_id or self.map_frame, self.map_frame)
            start = current
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f'PLANNER_WAITING_FOR_TF {exc}')
            return False
        start_cell = self._nearest_free(self._world_to_grid(start[0], start[1]))
        goal_cell = self._nearest_free(self._world_to_grid(goal[0], goal[1]))
        if start_cell is None or goal_cell is None:
            self.publish_status('NO_SAFE_START_OR_GOAL_CELL')
            return False
        cells = self._astar(start_cell, goal_cell)
        if not cells:
            self.path_points = []
            self.publish_status('NO_PATH')
            return False
        self.path_points = [self._grid_to_world(cell) for cell in cells]
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(self.path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point
            if index + 1 < len(self.path_points):
                heading = math.atan2(self.path_points[index + 1][1] - point[1],
                                     self.path_points[index + 1][0] - point[0])
            else:
                heading = goal[2]
            q = quaternion_from_yaw(heading)
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.last_plan = self.get_clock().now()
        self.last_waypoint = -1
        self.publish_status(f'PLANNING_PASS cells={len(cells)}')
        return True

    def _follow(self, current):
        if not self.path_points or self.goal_msg is None:
            self._publish_stop()
            return
        goal_pose = self.goal_msg.pose
        goal_x, goal_y, _ = self._transform_pose(
            goal_pose.position.x, goal_pose.position.y,
            yaw_from_quaternion(goal_pose.orientation),
            self.goal_msg.header.frame_id or self.map_frame, self.map_frame)
        goal_distance = math.hypot(goal_x - current[0], goal_y - current[1])
        if goal_distance <= self.goal_tolerance:
            self._publish_stop()
            if not self.reached:
                self.reached = True
                self.publish_status(f'GOAL_REACHED distance={goal_distance:.2f}')
            return
        nearest = min(range(len(self.path_points)),
                      key=lambda i: math.hypot(self.path_points[i][0] - current[0],
                                               self.path_points[i][1] - current[1]))
        lookahead = min(len(self.path_points) - 1, nearest + 3)
        target = self.path_points[lookahead]
        heading = math.atan2(target[1] - current[1], target[0] - current[0])
        error = angle_error(heading - current[2])
        cmd = Twist()
        cmd.angular.z = clamp(2.2 * error, -self.max_angular, self.max_angular)
        if abs(error) > 0.9:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.max_linear, 0.5 * goal_distance)
            cmd.linear.x *= max(0.15, math.cos(error))
        self.cmd_pub.publish(cmd)
        if lookahead != self.last_waypoint:
            self.last_waypoint = lookahead
            self.publish_status(f'FOLLOWING waypoint={lookahead}/{len(self.path_points)}')

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if self.map_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_MAP')
            self._publish_stop()
            return
        if self.odom_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_ODOM')
            self._publish_stop()
            return
        try:
            current = self._current_map_pose()
            self._send_auto_goal(current)
        except (RuntimeError, tf2_ros.TransformException) as exc:
            self.publish_status(f'PLANNER_WAITING_FOR_TF {exc}')
            self._publish_stop()
            return
        if self.goal_msg is None:
            self.publish_status('PLANNER_WAITING_FOR_GOAL')
            self._publish_stop()
            return
        if not self.reached:
            self.goal_pub.publish(self.goal_msg)
        now = self.get_clock().now()
        if (not self.path_points or
                (now - self.last_plan).nanoseconds / 1e9 >= self.replan_period):
            self._plan(current)
        self._follow(current)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## `tools/generate_lunar_terrain.py`

```python
"""
LunaBot V4 - Phase A - Lunar terrain generator (deterministic)
===============================================================

Generates the lunar terrain mesh used by
  src/lunabot_gazebo/worlds/lunar_world.sdf

Outputs (relative to this repo):
  src/lunabot_gazebo/worlds/meshes/lunar_terrain.obj           (visual, high-res, per-vertex color)
  src/lunabot_gazebo/worlds/meshes/lunar_terrain_collision.obj (collision, low-res)
  evidence/phase-a-launch-a/terrain_stats.txt
  evidence/phase-a-launch-a/terrain_preview_topdown.png
  evidence/phase-a-launch-a/terrain_preview_perspective.png

Terrain model (matches the Phase A visual reference spec):
  * 400 m x 400 m area, centered on world origin, 1.25 m grid (visual)
  * large-scale rolling relief (multi-octave value noise)
  * fine rocky roughness (short-wavelength noise)
  * dense crater field:
      - small  : ~220 craters, 3-10 m diameter
      - medium :  ~70 craters, 12-30 m diameter
      - large  :   7 craters, 44-90 m diameter (with central peaks)
  * smooth spawn pad at the origin (rover spawn zone)
  * flat boundary shelf so the terrain blends into the horizon plane
  * grey/monochrome per-vertex albedo with crater shadow/rim modulation

Usage:
  python3 tools/generate_lunar_terrain.py             # all outputs
  python3 tools/generate_lunar_terrain.py --no-previews
  python3 tools/generate_lunar_terrain.py --seed 7    # different field, still deterministic

Requires: numpy (pillow + matplotlib only needed for previews)
"""

import argparse
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_MESH_DIR = os.path.join(REPO, "src", "lunabot_gazebo", "worlds", "meshes")
EVIDENCE_DIR = os.path.join(REPO, "evidence", "phase-a-launch-a")

EXTENT = 400.0          # m, terrain spans [-200, 200] in x and y
N_FINE = 320            # visual grid resolution (cells)
N_COLL = 100            # collision grid resolution (cells)
SEED = 42

SPAWN_RADIUS = 14.0     # m, flat-ish pad around origin (fully blended inside)
SPAWN_FADE = 24.0       # m, blend completes here
BND_IN = 170.0          # m, boundary shelf starts
BND_OUT = 200.0         # m, boundary shelf is fully flat


def value_noise(rng, n, extent, wavelength):
    """2-D value noise: random lattice resampled with smoothstep bilinear."""
    m = max(2, int(round(extent / wavelength)))
    g = rng.random((m + 1, m + 1))
    u = np.linspace(0.0, m, n)
    U, V = np.meshgrid(u, u, indexing="xy")
    ix = np.clip(np.floor(U).astype(int), 0, m - 1)
    jx = np.clip(np.floor(V).astype(int), 0, m - 1)
    fx = U - ix
    fx = fx * fx * (3.0 - 2.0 * fx)
    fy = V - jx
    fy = fy * fy * (3.0 - 2.0 * fy)
    a = g[jx, ix]
    b = g[jx, ix + 1]
    c = g[jx + 1, ix]
    d = g[jx + 1, ix + 1]
    ab = a + (b - a) * fx
    cd = c + (d - c) * fx
    return (ab + (cd - ab) * fy) * 2.0 - 1.0   # -> [-1, 1]


def base_relief(x, y, rng):
    z = (
        3.2 * value_noise(rng, N_FINE, EXTENT, 260.0)
        + 1.8 * value_noise(rng, N_FINE, EXTENT, 130.0)
        + 0.9 * value_noise(rng, N_FINE, EXTENT, 65.0)
        + 0.45 * value_noise(rng, N_FINE, EXTENT, 32.0)
        + 0.35 * value_noise(rng, N_FINE, EXTENT, 10.0)
        + 0.18 * value_noise(rng, N_FINE, EXTENT, 5.0)
        + 0.002 * (x - y)                       # gentle overall slope
    )
    return z


def apply_crater(z, dep, rim, x, y, cx, cy, R, D):
    """Parabolic bowl + gaussian rim (+ central peak for large craters)."""
    d = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    bowl = D * (1.0 - np.clip(d / R, 0.0, 1.0) ** 2)
    bowl = np.where(d <= R, bowl, 0.0)
    rimh = 0.35 * D * np.exp(-(((d - R) / (0.30 * R)) ** 2))
    rimh = np.where(d < 1.6 * R, rimh, 0.0)
    peak = np.zeros_like(d)
    if R >= 22.0:
        pk = 0.30 * D * np.exp(-((d / (0.18 * R)) ** 2))
        peak = np.where(d < 0.4 * R, pk, 0.0)
    z = z - bowl + rimh + peak
    dep = np.maximum(dep, bowl / D)
    rim = np.maximum(rim, rimh / (0.35 * D))
    return z, dep, rim


def make_craters(rng):
    """Returns (cx, cy, R, D, class) list, sorted large -> small."""
    cr = []
    placed = []
    attempts = 0
    while len(placed) < 7 and attempts < 4000:
        attempts += 1
        R = rng.uniform(22.0, 45.0)
        cx, cy = rng.uniform(-180.0, 180.0), rng.uniform(-180.0, 180.0)
        if math.hypot(cx, cy) < 40.0:
            continue
        if any(math.hypot(cx - px, cy - py) < (R + pR) * 1.15 for px, py, pR in placed):
            continue
        placed.append((cx, cy, R))
        cr.append((cx, cy, R, R * rng.uniform(0.14, 0.22), "large"))
    for _ in range(70):
        R = rng.uniform(6.0, 15.0)
        while True:
            cx, cy = rng.uniform(-190.0, 190.0), rng.uniform(-190.0, 190.0)
            if math.hypot(cx, cy) > 20.0:
                break
        cr.append((cx, cy, R, R * rng.uniform(0.12, 0.20), "medium"))
    for _ in range(220):
        R = rng.uniform(1.5, 5.0)
        cx, cy = rng.uniform(-195.0, 195.0), rng.uniform(-195.0, 195.0)
        if math.hypot(cx, cy) < 12.0:
            cx, cy = cx * 2.0, cy * 2.0
        cr.append((cx, cy, R, R * rng.uniform(0.08, 0.16), "small"))
    cr.sort(key=lambda t: -t[2])   # apply largest first
    return cr


def write_obj(path, x, y, z, normals, colors=None, grid_shape=None):
    """Write OBJ with per-vertex normals (and optional per-vertex colors).

    Vertices are stored row-major on a regular grid (grid_shape = (rows, cols)).
    """
    if grid_shape is None:
        rows = cols = int(math.sqrt(x.shape[0]))
        if rows * cols != x.shape[0]:
            raise ValueError("cannot infer grid shape from vertex count")
    else:
        rows, cols = grid_shape
    n = x.shape[0]
    if n != rows * cols:
        raise ValueError(f"vertex count {n} != grid {rows}x{cols}")
    tmp = path + ".tmp"
    path_final = path
    path = tmp
    with open(path, "w") as fh:
        fh.write("# LunaBot V4 - generated lunar terrain (deterministic)\n")
        buf = []
        for i in range(n):
            if colors is not None:
                cr, cg, cb = colors[i]
                buf.append(f"v {x[i]:.4f} {y[i]:.4f} {z[i]:.4f}\n"
                           f"vn {normals[i, 0]:.5f} {normals[i, 1]:.5f} {normals[i, 2]:.5f}\n"
                           f"c {cr:.4f} {cg:.4f} {cb:.4f}\n")
            else:
                buf.append(f"v {x[i]:.4f} {y[i]:.4f} {z[i]:.4f}\n"
                           f"vn {normals[i, 0]:.5f} {normals[i, 1]:.5f} {normals[i, 2]:.5f}\n")
            if len(buf) >= 8192:
                fh.write("".join(buf))
                buf.clear()
        if buf:
            fh.write("".join(buf))
        buf = []
        for r in range(rows - 1):
            for c in range(cols - 1):
                a = r * cols + c + 1        # (r, c)
                b = r * cols + c + 2        # (r, c+1)
                d = (r + 1) * cols + c + 1  # (r+1, c)
                e = (r + 1) * cols + c + 2  # (r+1, c+1)
                buf.append(f"f {a}//{a} {b}//{b} {e}//{e}\n"
                           f"f {a}//{a} {e}//{e} {d}//{d}\n")
                if len(buf) >= 8192:
                    fh.write("".join(buf))
                    buf.clear()
        if buf:
            fh.write("".join(buf))
        size = os.path.getsize(path)
        if size > 200e6:
            os.remove(path)
            raise RuntimeError(f"safety valve: {path_final} grew to {size/1e6:.0f} MB, aborting")
    os.replace(tmp, path_final)   # atomic: never leaves a corrupt mesh behind


def bilinear_sample(z, n_fine, n_coarse):
    """Sample fine height field (n_fine x n_fine) onto a coarse grid (n_c, n_c)."""
    idx = np.linspace(0, n_fine - 1, n_coarse)
    i0 = np.floor(idx).astype(int)
    i1 = np.clip(i0 + 1, 0, n_fine - 1)
    f = (idx - i0)[:, None]                 # (n_c, 1)
    zi = z[i0] * (1.0 - f) + z[i1] * f      # (n_c, n_fine)  -> axis 0
    out = zi[:, i0] * (1.0 - f) + zi[:, i1] * f   # (n_c, n_c)  -> axis 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--no-previews", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    os.makedirs(WORLD_MESH_DIR, exist_ok=True)
    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    cell = EXTENT / N_FINE
    xs = -EXTENT / 2 + (np.arange(N_FINE) + 0.5) * cell
    X, Y = np.meshgrid(xs, xs, indexing="xy")     # rows = y, cols = x
    Z = base_relief(X, Y, rng)

    dep = np.zeros((N_FINE, N_FINE))
    rim = np.zeros((N_FINE, N_FINE))
    craters = make_craters(rng)
    for cx, cy, R, D, _cls in craters:
        Z, dep, rim = apply_crater(Z, dep, rim, X, Y, cx, cy, R, D)

    d0 = np.sqrt(X ** 2 + Y ** 2)
    h0 = float(Z[d0 < 10.0].mean())
    fade = np.clip((d0 - SPAWN_RADIUS) / (SPAWN_FADE - SPAWN_RADIUS), 0.0, 1.0)
    w = 0.5 - 0.5 * np.cos(np.pi * fade)
    Z = Z * w + h0 * (1.0 - w)

    zmean = float(Z.mean())
    fb = np.clip((d0 - BND_IN) / (BND_OUT - BND_IN), 0.0, 1.0)
    fb = fb * fb * (3.0 - 2.0 * fb)
    Z = Z * (1.0 - fb) + zmean * fb

    dzdx, dzdy = np.gradient(Z, cell)
    normals = np.stack([-dzdx, -dzdy, np.ones_like(Z)], axis=-1)
    normals /= (np.linalg.norm(normals, axis=-1, keepdims=True) + 1e-9)

    alb = 0.50 + 0.06 * value_noise(rng, N_FINE, EXTENT, 120.0) + 0.02 * value_noise(rng, N_FINE, EXTENT, 20.0)
    col = np.clip(alb - 0.16 * np.clip(dep, 0, 1.2) + 0.07 * np.clip(rim, 0, 1.0), 0.30, 0.72)
    fb2 = np.clip((d0 - BND_IN) / (BND_OUT - BND_IN), 0.0, 1.0)
    col = col * (1 - fb2) + (0.46 + 0.015 * value_noise(rng, N_FINE, EXTENT, 40.0)) * fb2
    col = np.clip(col, 0.30, 0.72)
    colors = np.repeat(col[..., None], 3, axis=-1)

    vis_path = os.path.join(WORLD_MESH_DIR, "lunar_terrain.obj")
    write_obj(vis_path, X.ravel(), Y.ravel(), Z.ravel(),
              normals.reshape(-1, 3), colors.reshape(-1, 3), grid_shape=(N_FINE, N_FINE))

    Zc = bilinear_sample(Z, N_FINE, N_COLL)
    dzdx_c, dzdy_c = np.gradient(Zc, EXTENT / N_COLL)
    nc = np.stack([-dzdx_c, -dzdy_c, np.ones_like(Zc)], axis=-1)
    nc /= (np.linalg.norm(nc, axis=-1, keepdims=True) + 1e-9)
    xs_c = -EXTENT / 2 + (np.arange(N_COLL) + 0.5) * (EXTENT / N_COLL)
    Xc, Yc = np.meshgrid(xs_c, xs_c, indexing="xy")
    coll_path = os.path.join(WORLD_MESH_DIR, "lunar_terrain_collision.obj")
    write_obj(coll_path, Xc.ravel(), Yc.ravel(), Zc.ravel(),
              nc.reshape(-1, 3), None, grid_shape=(N_COLL, N_COLL))

    n_large = sum(1 for t in craters if t[4] == "large")
    n_med = sum(1 for t in craters if t[4] == "medium")
    n_small = sum(1 for t in craters if t[4] == "small")
    stats = {
        "seed": args.seed,
        "extent_m": EXTENT,
        "visual_grid": f"{N_FINE}x{N_FINE} ({cell:.2f} m)",
        "collision_grid": f"{N_COLL}x{N_COLL} ({EXTENT / N_COLL:.1f} m)",
        "visual_vertices": N_FINE * N_FINE,
        "visual_triangles": 2 * (N_FINE - 1) * (N_FINE - 1),
        "z_min_m": round(float(Z.min()), 3),
        "z_max_m": round(float(Z.max()), 3),
        "z_mean_m": round(float(Z.mean()), 3),
        "spawn_height_m": round(h0, 3),
        "craters": {"large": n_large, "medium": n_med, "small": n_small},
        "large_craters": [
            {"x": round(c[0], 1), "y": round(c[1], 1), "R_m": round(c[2], 1), "depth_m": round(c[3], 2)}
            for c in craters if c[4] == "large"
        ],
        "visual_obj_mb": round(os.path.getsize(vis_path) / 1e6, 1),
        "collision_obj_mb": round(os.path.getsize(coll_path) / 1e6, 1),
    }
    stats_path = os.path.join(EVIDENCE_DIR, "terrain_stats.txt")
    with open(stats_path, "w") as fh:
        fh.write("LunaBot V4 - Phase A - lunar terrain statistics\n")
        fh.write("=" * 50 + "\n")
        for k, v in stats.items():
            fh.write(f"{k:20s}: {json.dumps(v)}\n")
        fh.write("\nSuggested rover spawn (model root z):\n")
        fh.write(f"  z = {h0 + 0.48:.3f} m   # wheel bottom = model_root_z - 0.43, +5 cm drop margin\n")
    print(json.dumps(stats, indent=2))

    if not args.no_previews:
        try:
            make_previews(X, Y, Z, col, craters)
            print("previews written to", EVIDENCE_DIR)
        except ImportError as e:
            print(f"WARNING: previews skipped ({e})")
    return 0


def make_previews(X, Y, Z, col, craters):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.tri import Triangulation

    L = np.array([0.5, -0.5, 0.6])
    L /= np.linalg.norm(L)
    dzdx, dzdy = np.gradient(Z, EXTENT / N_FINE)
    Nn = np.stack([-dzdx, -dzdy, np.ones_like(Z)], axis=-1)
    Nn /= (np.linalg.norm(Nn, axis=-1, keepdims=True) + 1e-9)
    shade = np.clip(Nn[..., 0] * L[0] + Nn[..., 1] * L[1] + Nn[..., 2] * L[2], 0, 1)
    surf = 0.16 + 0.62 * shade * (col / 0.5)

    fig, ax = plt.subplots(figsize=(10, 10), dpi=140)
    ax.imshow(surf, extent=[-EXTENT / 2, EXTENT / 2, EXTENT / 2, -EXTENT / 2],
              cmap="gray", vmin=0, vmax=1, origin="lower")
    ax.contour(Z, levels=24, colors="w", alpha=0.15, linewidths=0.4)
    for cx, cy, R, D, cls in craters:
        if cls == "large":
            ax.add_patch(Circle((cx, cy), R, fill=False, ec="orange", lw=1.0, alpha=0.85))
            ax.text(cx, cy + R + 5, f"R={R:.0f}m", color="orange", fontsize=8, ha="center")
    ax.add_patch(Circle((0, 0), 20, fill=False, ec="cyan", lw=1.2, ls="--"))
    ax.text(0, 27, "spawn pad", color="cyan", fontsize=9, ha="center")
    ax.set_xlim(-EXTENT / 2, EXTENT / 2)
    ax.set_ylim(-EXTENT / 2, EXTENT / 2)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("LunaBot V4 - lunar terrain, top-down (400 m x 400 m)")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(EVIDENCE_DIR, "terrain_preview_topdown.png"))
    plt.close(fig)

    step = 2
    Xs2 = X[::step, ::step]
    Ys2 = Y[::step, ::step]
    Zs2 = Z[::step, ::step]
    Cs2 = 0.16 + 0.62 * shade[::step, ::step] * (col[::step, ::step] / 0.5)
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    tri2d = Triangulation(Xs2.ravel(), Ys2.ravel())
    t = tri2d.triangles
    Xs, Ys, Zs = Xs2.ravel(), Ys2.ravel(), Zs2.ravel()
    per_tri = Cs2.ravel()[t].mean(axis=1)
    verts = np.stack((Xs[t], Ys[t], Zs[t]), axis=-1)          # (M, 3, 3)
    cmap = plt.get_cmap("gray")
    fc = cmap(np.clip(per_tri, 0, 1)[:, None])               # (M, 4) rgba
    fig = plt.figure(figsize=(11, 7.5), dpi=140)
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(verts, facecolors=fc, edgecolors="none", linewidths=0))
    ax.set_xlim(-EXTENT / 2, EXTENT / 2)
    ax.set_ylim(-EXTENT / 2, EXTENT / 2)
    ax.set_zlim(float(Z.min()) - 2, float(Z.max()) + 2)
    ax.scatter([0], [0], [Z[N_FINE // 2, N_FINE // 2] + 0.8], c="red", s=30, depthshade=False)
    ax.set_box_aspect((1, 1, 0.16))
    ax.view_init(elev=27, azim=-90)
    ax.dist = 27
    ax.set_title("LunaBot V4 - lunar terrain, perspective (view ~ Gazebo GUI camera)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    fig.tight_layout()
    fig.savefig(os.path.join(EVIDENCE_DIR, "terrain_preview_perspective.png"))
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
```

## `tools/plot_lidar_scan.py`

```python
"""
LunaBot V4 - Phase A - plot a single LiDAR scan (sensor_msgs/LaserScan)
saved as YAML by `ros2 topic echo /lunabot/lidar/scan --once`.

Usage:
    python3 tools/plot_lidar_scan.py lidar_scan_sample.yaml lidar_scan_preview.png

Writes a top-down polar view of the scan (the crater field around the rover).
"""
import math
import os
import sys


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    yaml_path, out_path = sys.argv[1], sys.argv[2]

    import yaml
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(yaml_path) as fh:
        scan = yaml.safe_load(fh)

    ranges = [float(r) for r in scan.get("ranges", [])]
    angles = [float(a) for a in scan.get("angles", [])]
    if not angles:
        n = len(ranges)
        a0 = float(scan.get("angle_min", -math.pi))
        a1 = float(scan.get("angle_max", math.pi))
        angles = [a0 + (a1 - a0) * i / max(1, n - 1) for i in range(n)]

    a_min = float(scan.get("angle_min", -math.pi))
    a_max = float(scan.get("angle_max", math.pi))
    r_max = min(60.0, float(scan.get("range_max", 60.0)))
    r_min = float(scan.get("range_min", 0.1))
    frame = scan.get("header", {}).get("frame_id", "lidar")
    stamp = scan.get("header", {}).get("stamp", {})
    t = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9

    pts = []
    for r, a in zip(ranges, angles):
        if r_min < r < r_max and r > 0:
            pts.append((math.cos(a) * r, math.sin(a) * r))

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    lim = min(60.0, max([abs(v) for v in xs + ys] + [20.0]))
    fig, ax = plt.subplots(figsize=(9, 9), dpi=140)
    ax.scatter(xs, ys, s=1.2, c="0.25", alpha=0.8)
    for ring in (10, 20, 30, 40, 50):
        if ring <= lim:
            ax.add_patch(plt.Circle((0, 0), ring, fill=False, ec="0.6", lw=0.6, ls="--"))
    ax.plot([0], [0], marker="^", ms=9, color="red")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (rover frame)")
    ax.set_ylabel("y [m] (rover frame)")
    ax.set_title(f"LiDAR scan - {frame} - t={t:.1f} s - {len(pts)}/{len(ranges)} valid ranges")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"wrote {out_path} ({len(pts)} points)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

## `tools/validate_phase_a.py`

```python
"""
LunaBot V4 - Phase A - static validation
=========================================
Validates every Phase A artifact that can be checked WITHOUT running Gazebo:

  1. world + model SDF: XML well-formedness + semantic checks
     (plugins, joints, links, sensors, mesh files on disk, gravity)
  2. topic contract: every sensor topic in model.sdf is bridged in launch-a.sh
  3. spawn height: launch-a.sh SPAWN_Z matches terrain_stats.txt (deterministic)
  4. OBJ meshes: vertex/face counts, index bounds, bounding box
  5. scripts: bash -n syntax, python py_compile
  6. rviz config: required display topic types present
  7. documentation + entry points present

Writes evidence/phase-a-launch-a/static_validation.txt
Exit code: 0 = all PASS, 1 = at least one FAIL.
"""
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DIR = os.path.join(REPO, "src", "lunabot_gazebo", "worlds")
EVIDENCE = os.path.join(REPO, "evidence", "phase-a-launch-a")

WORLD = os.path.join(WORLD_DIR, "lunar_world.sdf")
MODEL = os.path.join(REPO, "src", "lunabot_gazebo", "models", "lunabot_v4", "model.sdf")
LAUNCH = os.path.join(REPO, "scripts", "launch-a.sh")
TELEOP = os.path.join(REPO, "scripts", "wasd_teleop.py")
RVIZ = os.path.join(REPO, "rviz", "phase_a.rviz")
ENTRYP = os.path.join(REPO, "launch-a")
DOCS = os.path.join(REPO, "docs", "phase-a-launch-a.md")
STATS = os.path.join(EVIDENCE, "terrain_stats.txt")
VIS_OBJ = os.path.join(WORLD_DIR, "meshes", "lunar_terrain.obj")
COL_OBJ = os.path.join(WORLD_DIR, "meshes", "lunar_terrain_collision.obj")

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    mark = "PASS" if ok else "FAIL"
    print(f"[{mark}] {name}" + (f" - {detail}" if detail and not ok else ""))
    return bool(ok)


def load_sdf(path):
    """Return the <world> element (world files)."""
    return ET.parse(path).getroot().find("world")


def load_model_sdf(path):
    """Return the <model> element (model files)."""
    return ET.parse(path).getroot().find("model")


def validate_sdfs():
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    check("world SDF XML well-formed", root.tag == "sdf" and world is not None)
    check("world name == lunar_world",
          world.get("name") == "lunar_world", f"got {world.get('name')}")

    g = world.find("gravity").text.split()
    check("lunar gravity (0 0 -1.62)",
          g == ["0", "0", "-1.62"], f"got {g}")

    plugins = {p.get("name") for p in world.findall("plugin")}
    for req in ("ignition::gazebo::systems::Physics",
                "ignition::gazebo::systems::SceneBroadcaster",
                "ignition::gazebo::systems::UserCommands",
                "ignition::gazebo::systems::Sensors",
                "ignition::gazebo::systems::Contact"):
        check(f"world plugin {req.split('::')[-1]}", req in plugins)

    terrain = world.find("./model[@name='lunar_terrain']")
    check("terrain model present", terrain is not None)
    if terrain is not None:
        check("terrain static", terrain.find("static").text == "true")
        for kind in ("collision", "visual"):
            uri = terrain.find(f"link/{kind}/geometry/mesh/uri").text
            local = uri.replace("file://", "")
            path = os.path.normpath(os.path.join(WORLD_DIR, local))
            check(f"terrain {kind} mesh exists ({local})", os.path.isfile(path), path)

    horizon = world.find("./model[@name='lunar_horizon']")
    check("horizon catch-plane present", horizon is not None)
    check("GUI camera configured", world.find("gui/camera") is not None)
    habitat = world.find("./model[@name='lunar_habitat_main']")
    equipment = world.find("./model[@name='lunar_habitat_equipment']")
    obstacle = world.find("./model[@name='presentation_obstacle_forward']")
    check("lunar habitat presentation model present", habitat is not None)
    check("habitat equipment presentation model present", equipment is not None)
    check("physical forward obstacle present", obstacle is not None and
          obstacle.find("link/collision") is not None)

    mroot = ET.parse(MODEL).getroot()
    m = mroot.find("model")
    check("model SDF XML well-formed", mroot.tag == "sdf" and m is not None)
    check("model name == lunabot_v4", m.get("name") == "lunabot_v4")
    links = {l.get("name") for l in m.findall("link")}
    joints = {j.get("name"): j for j in m.findall("joint")}
    check("6 wheel joints",
          len([n for n in joints if "wheel_joint" in n]) == 6,
          str(sorted(n for n in joints if "wheel_joint" in n)))

    bad_parents = [n for n, j in joints.items()
                   if j.find("parent").text not in links or j.find("child").text not in links]
    check("all joint parent/child resolve to links", not bad_parents, str(bad_parents))

    expected_links = {"chassis", "left_rocker", "right_rocker", "left_bogie",
                      "right_bogie", "left_front_wheel", "right_front_wheel",
                      "left_middle_wheel", "right_middle_wheel",
                      "left_rear_wheel", "right_rear_wheel", "sensor_head",
                      "imu_link", "sensor_mast"}
    check("key links present", expected_links <= links,
          str(expected_links - links))

    sensors = {s.get("name"): s for s in m.iter("sensor")}
    for sname, stype, topic in (("rgb_camera", "camera", "/lunabot/camera/image_raw"),
                                ("depth_camera", "depth_camera", "/lunabot/depth/image_raw"),
                                ("lidar", "gpu_lidar", "/lunabot/lidar/scan"),
                                ("imu", "imu", "/lunabot/imu")):
        check(f"sensor {sname} (type {stype})",
              sname in sensors and sensors[sname].get("type") == stype)
        if sname in sensors:
            check(f"sensor {sname} topic {topic}",
                  sensors[sname].find("topic").text == topic,
                  sensors[sname].find("topic").text)

    dd = next(p for p in m.iter("plugin")
              if p.get("filename") == "ignition-gazebo-diff-drive-system")
    dd_joints = [e.text for e in dd.findall("left_joint")] + [e.text for e in dd.findall("right_joint")]
    check("DiffDrive joints all exist", all(j in joints for j in dd_joints),
          str([j for j in dd_joints if j not in joints]))
    check("DiffDrive odom topic /lunabot/odom",
          dd.find("odom_topic").text == "/lunabot/odom")
    check("DiffDrive tf frames odom/chassis",
          dd.find("frame_id").text == "odom" and dd.find("child_frame_id").text == "chassis")
    check("DiffDrive cmd topic /cmd_vel", dd.find("topic").text == "/cmd_vel")

    js_pubs = [p for p in m.iter("plugin")
               if p.get("filename") == "ignition-gazebo-joint-position-controller-system"]
    bad = [e.text for p in js_pubs for e in p.findall("joint_name") if e.text not in joints]
    check("steer/mast controllers reference real joints", not bad, str(bad))
    jsp = [p for p in m.iter("plugin")
           if p.get("filename") == "ignition-gazebo-joint-state-publisher-system"]
    bad = [e.text for p in jsp for e in p.findall("joint_name") if e.text not in joints]
    check("joint-state publisher references real joints", not bad, str(bad))


def validate_topic_contract():
    m = load_model_sdf(MODEL)
    topics = {s.find("topic").text for s in m.iter("sensor")
              if s.find("topic") is not None}
    launch = open(LAUNCH).read()
    for t in sorted(topics):
        check(f"bridge maps {t} (gazebo->ros)", f"{t}@sensor_msgs" in launch
              or f"{t}@nav_msgs" in launch)
    check("bridge maps /cmd_vel (ros->gazebo)",
          re.search(r"/cmd_vel@geometry_msgs/msg/Twist\]\S+\.Twist", launch) is not None)
    check("bridge maps /tf Pose_V -> TFMessage",
          re.search(r"/tf@tf2_msgs/msg/TFMessage\[\S+\.Pose_V", launch) is not None)


def validate_spawn_height():
    launch = open(LAUNCH).read()
    m = re.search(r'SPAWN_Z="(-?[\d.]+)"', launch)
    stats = open(STATS).read()
    h = re.search(r"spawn_height_m\s*:\s*(-?[\d.]+)", stats)
    if m and h:
        ok = abs(float(m.group(1)) - (float(h.group(1)) + 0.48)) < 0.02
        check("SPAWN_Z matches terrain stats (h0 + 0.48)", ok,
              f"launch={m.group(1)} h0={h.group(1)}")
    else:
        check("SPAWN_Z / terrain stats parseable", False, "regex mismatch")


def validate_obj(path, expect_color):
    n_v = n_vn = n_c = 0
    max_idx = 0
    bounds = [[1e9] * 3, [-1e9] * 3]
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if not p:
                continue
            if p[0] == "v":
                n_v += 1
                for i, v in enumerate(p[1:4]):
                    v = float(v)
                    bounds[0][i] = min(bounds[0][i], v)
                    bounds[1][i] = max(bounds[1][i], v)
            elif p[0] == "vn":
                n_vn += 1
            elif p[0] == "c":
                n_c += 1
            elif p[0] == "f":
                for ref in p[1:]:
                    max_idx = max(max_idx, int(ref.split("/")[0]))
    name = os.path.basename(path)
    check(f"{name}: has vertices", n_v > 0, str(n_v))
    check(f"{name}: vertex count square grid", int(round(n_v ** 0.5)) ** 2 == n_v, str(n_v))
    check(f"{name}: normals for all vertices", n_vn == n_v, f"vn={n_vn} v={n_v}")
    check(f"{name}: color vertices expected={expect_color}",
          (n_c == n_v) if expect_color else (n_c == 0), f"c={n_c}")
    check(f"{name}: face indices in bounds", 0 < max_idx <= n_v, str(max_idx))
    ext = [bounds[1][i] - bounds[0][i] for i in range(3)]
    check(f"{name}: extent ~400x400 m", 395 < ext[0] < 405 and 395 < ext[1] < 405,
          str([round(e, 1) for e in ext]))
    check(f"{name}: relief within [-12, +10] m",
          bounds[1][2] < 10 and bounds[0][2] > -12,
          f"z=[{bounds[0][2]:.2f}, {bounds[1][2]:.2f}]")
    return n_v


def validate_scripts():
    for sh in (LAUNCH, ENTRYP, os.path.join(EVIDENCE, "collect_evidence.sh")):
        if not os.path.isfile(sh):
            check(f"bash syntax: {os.path.relpath(sh, REPO)}", False, "missing")
            continue
        r = subprocess.run(["bash", "-n", sh], capture_output=True)
        check(f"bash syntax: {os.path.relpath(sh, REPO)}", r.returncode == 0,
              r.stderr.decode()[:200])
    for py in (TELEOP,
               os.path.join(REPO, "tools", "generate_lunar_terrain.py"),
               os.path.join(REPO, "tools", "plot_lidar_scan.py")):
        r = subprocess.run([sys.executable, "-m", "py_compile", py], capture_output=True)
        check(f"python compile: {os.path.relpath(py, REPO)}", r.returncode == 0,
              r.stderr.decode()[:200])
    for f in (LAUNCH, ENTRYP):
        check(f"executable: {os.path.relpath(f, REPO)}", os.access(f, os.X_OK))


def validate_rviz():
    if not os.path.isfile(RVIZ):
        check("rviz config exists", False)
        return
    txt = open(RVIZ).read()
    check("rviz config exists", True)
    check("rviz fixed frame odom", "Fixed Frame: odom" in txt)
    check("rviz displays LaserScan topic", "type: sensor_msgs/msg/LaserScan" in txt)
    check("rviz displays Image topic", "type: sensor_msgs/msg/Image" in txt)
    check("rviz displays depth image", "Value: /lunabot/depth/image_raw" in txt)
    check("rviz TF display", "rviz_default_plugins/TF" in txt)


def validate_docs():
    check("docs/phase-a-launch-a.md exists", os.path.isfile(DOCS))
    if os.path.isfile(DOCS):
        txt = open(DOCS).read()
        for sec in ("## 1. Objective", "## 11. Launch Command", "## 16. Success Criteria",
                    "## 22. Evidence", "## 23. Known Limitations"):
            check(f"docs section {sec}", sec in txt)
    check("terrain stats file", os.path.isfile(STATS))
    check("evidence README", os.path.isfile(os.path.join(EVIDENCE, "README.md")))
    check("verification checklist",
          os.path.isfile(os.path.join(EVIDENCE, "verification_checklist.md")))


def main():
    print("=" * 60)
    print("LUNABOT V4 - PHASE A - STATIC VALIDATION")
    print("=" * 60)
    print("-- SDF files --")
    validate_sdfs()
    print("-- topic contract --")
    validate_topic_contract()
    print("-- spawn height --")
    validate_spawn_height()
    print("-- meshes --")
    validate_obj(VIS_OBJ, expect_color=True)
    validate_obj(COL_OBJ, expect_color=False)
    print("-- scripts --")
    validate_scripts()
    print("-- rviz --")
    validate_rviz()
    print("-- docs --")
    validate_docs()

    fails = [r for r in results if not r[1]]
    print("=" * 60)
    print(f"RESULT: {len(results) - len(fails)}/{len(results)} checks passed"
          + (f", {len(fails)} FAILED" if fails else " - ALL PASS"))
    print("=" * 60)

    os.makedirs(EVIDENCE, exist_ok=True)
    out = os.path.join(EVIDENCE, "static_validation.txt")
    with open(out, "w") as fh:
        fh.write("LunaBot V4 - Phase A - static validation report\n")
        fh.write(f"date: {os.popen('date -u +%FT%TZ').read().strip()}\n")
        fh.write("=" * 60 + "\n")
        for name, ok, detail in results:
            fh.write(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else "") + "\n")
        fh.write("=" * 60 + "\n")
        fh.write(f"RESULT: {len(results) - len(fails)}/{len(results)} checks passed\n")
    print(f"report: {out}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
```

## `tools/validate_phase_b.py`

```python
"""Static Phase B gate: Control & Odometry.

This validator never claims Gazebo runtime success. It checks that the
independent launch, ROS control boundary, odometry monitor, docs and Phase A
baseline are internally consistent. Runtime evidence must come from the user's
ROS 2/Gazebo workstation via evidence/phase-b-launch-b/collect_evidence.sh.
"""

from pathlib import Path
import ast
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []

def check(name, ok, detail=''):
    checks.append((name, bool(ok), detail))


def text(path):
    return path.read_text(encoding='utf-8')


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)

required = [
    'launch-a', 'launch-b', 'scripts/launch-a.sh', 'scripts/launch-b.sh',
    'scripts/wasd_teleop.py', 'scripts/control_odometry.py',
    'scripts/odometry_monitor.py', 'rviz/phase_a.rviz', 'rviz/phase_b.rviz',
    'src/lunabot_gazebo/worlds/lunar_world.sdf',
    'src/lunabot_gazebo/models/lunabot_v4/model.sdf',
    'tools/validate_phase_a.py', 'evidence/phase-a-launch-a/README.md',
    'evidence/phase-b-launch-b/README.md',
    'evidence/phase-b-launch-b/collect_evidence.sh',
]
for rel in required:
    p = ROOT / rel
    check(f'file exists: {rel}', p.is_file())
for rel in ['launch-b', 'scripts/launch-b.sh', 'scripts/control_odometry.py',
            'scripts/odometry_monitor.py', 'evidence/phase-b-launch-b/collect_evidence.sh']:
    check(f'executable: {rel}', (ROOT / rel).stat().st_mode & 0o111)

for rel, kind in [('src/lunabot_gazebo/worlds/lunar_world.sdf', 'world'),
                  ('src/lunabot_gazebo/models/lunabot_v4/model.sdf', 'model')]:
    try:
        root = ET.parse(ROOT / rel).getroot()
        check(f'{kind} SDF well-formed: {rel}', True)
    except Exception as exc:
        check(f'{kind} SDF well-formed: {rel}', False, str(exc))

for rel in ['scripts/wasd_teleop.py', 'scripts/control_odometry.py',
            'scripts/odometry_monitor.py', 'tools/validate_phase_a.py',
            'tools/plot_lidar_scan.py']:
    try:
        ast.parse(text(ROOT / rel))
        check(f'Python syntax: {rel}', True)
    except Exception as exc:
        check(f'Python syntax: {rel}', False, str(exc))

for rel in ['scripts/launch-a.sh', 'scripts/launch-b.sh', 'launch-a', 'launch-b']:
    r = run(['bash', '-n', rel])
    check(f'bash syntax: {rel}', r.returncode == 0, r.stderr.strip())

control = text(ROOT / 'scripts/control_odometry.py')
check('control node class exists', 'class ControlNode(Node)' in control)
check('control input defaults to /cmd_vel_in', "default='/cmd_vel_in'" in control)
check('control output defaults to /cmd_vel', "default='/cmd_vel'" in control)
check('control clamps linear and angular values',
      '_clamp(msg.linear.x' in control and '_clamp(msg.angular.z' in control)
check('control applies acceleration limits',
      'max_linear_accel' in control and 'max_angular_accel' in control and
      '_approach(' in control)
check('control watchdog stops stale input',
      'WATCHDOG_STOP' in control and 'watchdog_sec' in control)
check('control status topic', '/lunabot/control/status' in control)
check('control publishes Twist output', 'self.cmd_pub.publish(out)' in control)
check('control guards shutdown publish',
      'if rclpy.ok()' in control and 'publisher context is still valid' in control)

monitor = text(ROOT / 'scripts/odometry_monitor.py')
check('monitor subscribes to DiffDrive odometry',
      "'/lunabot/odom'" in monitor and 'Odometry' in monitor)
check('monitor imports Gazebo-compatible QoS',
      'qos_profile_sensor_data' in monitor and
      'ExternalShutdownException' in monitor)
check('monitor publishes quality status', '/lunabot/odometry/status' in monitor)
check('monitor records CSV samples', 'odometry_samples.csv' in monitor and
      'csv.writer' in monitor)
check('monitor writes report', 'odometry_report.txt' in monitor and
      'quality_result' in monitor)
check('monitor checks odom and chassis frames',
      "'odom' in self.frame_ids" in monitor and
      "'chassis' in self.child_frame_ids" in monitor)
check('monitor checks continuity', 'max_step' in monitor and 'self.max_step < 0.5' in monitor)

teleop = text(ROOT / 'scripts/wasd_teleop.py')
check('teleop default preserves Phase A /cmd_vel', "topic='/cmd_vel'" in teleop)
check('teleop supports --topic override', 'ap.add_argument("--topic"' in teleop)
check('teleop publishes selected topic', 'create_publisher(Twist, topic' in teleop)
check('teleop demo supports sim time', 'Clock' in teleop and 'node.sim_t' in teleop)
check('teleop writes failure evidence',
      'failure_reason' in teleop and 'demo_drive_result.txt' in teleop)
check('teleop reads ROS Odometry twist correctly',
      'self.odom.twist.twist' in teleop)
check('teleop demo spins in one ROS thread',
      'Synchronous spin_once' in teleop and 'threading' not in teleop)
check('teleop uses Gazebo-compatible QoS',
      'qos_profile_sensor_data' in teleop and
      'ExternalShutdownException' not in teleop)

launch = text(ROOT / 'scripts/launch-b.sh')
check('launch-b has Phase B banner', 'PHASE B' in launch and 'CONTROL & ODOMETRY' in launch)
launch_commands = '\n'.join(
    line for line in launch.splitlines() if not line.lstrip().startswith('#'))
check('launch-b does not invoke launch-a',
      not re.search(r'(^|\s)(bash\s+)?[^#\n]*launch-a', launch_commands))
check('launch-b uses Phase B evidence directory', 'phase-b-launch-b' in launch)
check('launch-b starts controller', 'control_odometry.py' in launch and 'CONTROL_PID' in launch)
check('launch-b starts odometry monitor', 'odometry_monitor.py' in launch and 'ODOM_PID' in launch)
check('launch-b separates control input/output',
      '--input-topic /cmd_vel_in' in launch and '--output-topic /cmd_vel' in launch)
check('launch-b explicitly unpauses headless Gazebo',
      'WorldControl' in launch and "--req 'pause: false'" in launch)
check('launch-b uses real topic-message validation',
      'sample="$(timeout 30 ros2 topic echo' in launch and
      '| head -n 3' not in launch)
check('launch-b cleans stale Phase B nodes',
      'ros_ign_bridge.*parameter_bridge' in launch and
      'scripts/control_odometry.py' in launch)
check('launch-b bridges simulation clock', '"/clock@rosgraph_msgs/msg/Clock' in launch)
check('launch-b retries dynamic TF validation',
      'for _ in 1 2 3' in launch and 'tf2_echo' in launch)
check('launch-b has watchdog status runtime check',
      '/lunabot/control/status' in launch)
check('launch-b has odometry status runtime check',
      '/lunabot/odometry/status' in launch)
check('launch-b records IMU diagnostics on failure',
      'imu_gazebo_topics.txt' in launch and
      'imu_topic_info.txt' in launch and
      'imu_diagnostics.txt' in launch)
check('launch-b has clean shutdown',
      'kill -TERM "$CONTROL_PID"' in launch and
      'kill -TERM "$ODOM_PID"' in launch and 'shutdown()' in launch)
check('launch-b has demo nonzero exit propagation',
      'shutdown "$EXIT_CODE"' in launch and 'demo_rc' in launch)

rviz_b = text(ROOT / 'rviz/phase_b.rviz')
world_sdf = text(ROOT / 'src/lunabot_gazebo/worlds/lunar_world.sdf')
check('world enables Fortress IMU system plugin',
      'ignition-gazebo-imu-system' in world_sdf and
      'ignition::gazebo::systems::Imu' in world_sdf)
check('Phase B RViz fixed frame odom', 'Fixed Frame: odom' in rviz_b)
check('Phase B RViz displays odometry',
      'rviz_default_plugins/Odometry' in rviz_b and '/lunabot/odom' in rviz_b)
check('Phase B RViz LaserScan topic is explicit',
      'Value: /lunabot/lidar/scan' in rviz_b)
check('Phase B RViz camera topic is explicit',
      'Value: /lunabot/camera/image_raw' in rviz_b)
check('Phase B RViz sensor QoS is best effort',
      rviz_b.count('Reliability Policy: Best Effort') >= 2)
for rel in ['docs/phase-b-launch-b.md']:
    p = ROOT / rel
    check(f'document exists: {rel}', p.is_file())
    if p.is_file():
        d = text(p)
        for i in range(1, 24):
            check(f'doc {rel} section {i}',
                  re.search(rf'^## {i}\. ', d, re.MULTILINE) is not None)

collector = text(ROOT / 'evidence/phase-b-launch-b/collect_evidence.sh')
check('collector launches Phase B directly',
      'scripts/launch-b.sh' in collector and 'DEMO=1' in collector)

r = run([sys.executable, 'tools/validate_phase_a.py'])
check('Phase A static baseline remains 80/80',
      r.returncode == 0 and '80/80' in r.stdout,
      r.stdout[-200:].strip())

passed = sum(ok for _, ok, _ in checks)
print('=' * 60)
print('LUNABOT V4 PHASE B STATIC VALIDATION')
print('=' * 60)
for name, ok, detail in checks:
    suffix = f' - {detail}' if detail else ''
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{suffix}")
print('=' * 60)
print(f'RESULT: {passed}/{len(checks)} checks passed - ' +
      ('ALL PASS' if passed == len(checks) else 'FAIL'))
print('=' * 60)
report = ROOT / 'evidence/phase-b-launch-b/static_validation.txt'
report.write_text('\n'.join(
    f"[{'PASS' if ok else 'FAIL'}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding='utf-8')
print(f'report: {report}')
sys.exit(0 if passed == len(checks) else 1)
```

## `tools/validate_phase_c.py`

```python
"""Static Phase C gate: SLAM & Localization.

This validator checks repository structure and the independent launch contract.
It deliberately does not claim that ROS 2, Gazebo, slam_toolbox, /map, TF, or
IMU runtime data work; those checks belong to the workstation run of
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c.
"""

from pathlib import Path
import ast
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []


def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)


required = [
    "launch-a", "launch-b", "launch-c", "scripts/launch-a.sh",
    "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/wasd_teleop.py", "scripts/control_odometry.py",
    "scripts/odometry_monitor.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "docs/phase-b-launch-b.md", "docs/phase-3-launch-c.md",
    "evidence/phase-b-launch-b/README.md",
    "evidence/phase-c-launch-c/README.md",
    "evidence/phase-c-launch-c/verification_checklist.md",
    "evidence/phase-c-launch-c/collect_evidence.sh",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
    "tools/validate_phase_a.py", "tools/validate_phase_b.py",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-c", "scripts/launch-c.sh", "tools/validate_phase_c.py",
            "evidence/phase-c-launch-c/collect_evidence.sh"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/wasd_teleop.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "tools/validate_phase_a.py",
            "tools/validate_phase_b.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c",
            "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

for rel in ["tools/validate_phase_a.py", "tools/validate_phase_b.py"]:
    r = run([sys.executable, rel])
    expected = "80/80" if rel.endswith("phase_a.py") else "103/103"
    check(f"existing static gate remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-c.sh")
wrapper = read("launch-c")
config = read("config/slam_toolbox_phase_c.yaml")
rviz = read("rviz/phase_c.rviz")
world = read("src/lunabot_gazebo/worlds/lunar_world.sdf")
model = read("src/lunabot_gazebo/models/lunabot_v4/model.sdf")
docs = read("docs/phase-3-launch-c.md")

check("root launch-c resolves its real path", "readlink -f" in wrapper)
check("root launch-c execs scripts/launch-c.sh", "scripts/launch-c.sh" in wrapper)
noncomment_launch = "\n".join(
    line for line in launch.splitlines() if not line.lstrip().startswith("#"))
check("Phase C script does not invoke launch-a",
      not re.search(r"(^|[;&|()\s])(?:bash\s+)?[^\n]*launch-a", noncomment_launch))
check("Phase C script does not invoke launch-b", "launch-b" not in noncomment_launch)
check("Phase C script uses its own evidence directory",
      "phase-c-launch-c" in launch and "phase-b-launch-b" not in noncomment_launch)
check("Phase C has symlink-safe script directory resolution",
      'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
collector = read("evidence/phase-c-launch-c/collect_evidence.sh")
check("Phase C evidence collector resolves the repository", "readlink -f" in collector and
      "scripts/launch-c.sh" in collector)
check("Phase C evidence collector enables evidence and demo", "DEMO=1 EVIDENCE=1" in collector)
check("Phase C uses clean process-group startup", "setsid" in launch)
check("Phase C has trap-controlled shutdown", "trap 'shutdown 130' INT TERM" in launch)
check("Phase C has bounded process-group shutdown", "stop_group()" in launch and
      "kill -KILL -\"$pid\"" in launch)
check("Phase C shuts down slam_toolbox first", 'stop_group "$SLAM_PID"' in launch)
check("Phase C shuts down control and odometry", 'stop_group "$CONTROL_PID"' in launch and
      'stop_group "$ODOM_PID"' in launch)
check("Phase C shuts down Gazebo process group", 'kill -TERM -"$GAZEBO_PID"' in launch)
check("Phase C cleans stale SLAM processes", "slam_toolbox.*online_async" in launch)

check("slam_toolbox is the selected SLAM system", "slam_toolbox" in config and
      "slam_toolbox" in launch)
check("no Cartographer configuration", "cartographer" not in config.lower() and
      "cartographer" not in launch.lower())
check("no AMCL configuration", "amcl" not in config.lower() and
      "amcl" not in launch.lower())
check("no second mapping/localization stack", not any(x in config.lower() for x in
      ["hector_slam", "karto_slam", "rtabmap"]))
check("slam toolbox launch executable is explicit",
      "ros2 launch slam_toolbox online_async_launch.py" in launch)
check("SLAM config path is explicit", 'slam_params_file:="$SLAM_CONFIG"' in launch)
check("slam toolbox uses simulation time", "use_sim_time:=true" in launch and
      "use_sim_time: true" in config)
check("mapping mode is explicit", "mode: mapping" in config)
check("scan topic is explicit in config", "scan_topic: /lunabot/lidar/scan" in config)
check("map frame is explicit", "map_frame: map" in config)
check("odom frame is explicit", "odom_frame: odom" in config)
check("base frame is explicit", "base_frame: chassis" in config)
check("SLAM publishes at a map update interval", "map_update_interval:" in config)
check("SLAM loop closure remains enabled", "do_loop_closing: true" in config)
check("SLAM transform publish period is configured", "transform_publish_period:" in config)

check("world and rover paths are inherited", 'WORLD_PATH="$WORLD_DIR/lunar_world.sdf"' in launch and
      'MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"' in launch)
check("Phase C keeps the validated spawn height", 'SPAWN_Z="-2.308"' in launch)
check("headless mode is supported", 'HEADLESS="${HEADLESS:-0}"' in launch and
      'gazebo -s' in launch)
check("Gazebo is explicitly unpaused", "WorldControl" in launch and
      "--req 'pause: false'" in launch)
check("Gazebo waits for readiness", 'world/$WORLD_NAME/info' in launch and
      'for _ in $(seq 1 90)' in launch)
check("bridge detects ros_ign or ros_gz", "ros_ign_bridge" in launch and
      "ros_gz_bridge" in launch)
check("bridge includes simulation clock", '"/clock@rosgraph_msgs/msg/Clock' in launch)
check("bridge includes explicit LaserScan", '"/lunabot/lidar/scan@sensor_msgs/msg/LaserScan' in launch)
check("bridge includes explicit camera", '"/lunabot/camera/image_raw@sensor_msgs/msg/Image' in launch)
check("bridge includes explicit IMU", '"/lunabot/imu@sensor_msgs/msg/Imu' in launch)
check("bridge includes preserved odometry", '"/lunabot/odom@nav_msgs/msg/Odometry' in launch)
check("static TF includes chassis to sensor head", '"chassis|sensor_head|' in launch)
check("static TF includes sensor head to lidar", '"sensor_head|lidar|' in launch)
check("static TF includes exact Gazebo LaserScan frame", '"sensor_head|lunabot_v4/sensor_head/lidar|' in launch)
check("control input boundary is preserved", "--input-topic /cmd_vel_in" in launch)
check("control output boundary is preserved", "--output-topic /cmd_vel" in launch)
check("watchdog remains configured", "--watchdog-sec 0.5" in launch)
check("odometry monitor is started", "odometry_monitor.py" in launch and "ODOM_PID" in launch)
check("IMU Fortress plugin remains enabled", "ignition-gazebo-imu-system" in world and
      "ignition::gazebo::systems::Imu" in world)
check("model publishes the required odometry topic", "<odom_topic>/lunabot/odom</odom_topic>" in model)
check("model odometry frame pair is unchanged", "<frame_id>odom</frame_id>" in model and
      "<child_frame_id>chassis</child_frame_id>" in model)

check("runtime validation uses topic echo once", 'sample="$(timeout 30 ros2 topic echo' in launch and
      "--once" in launch)
check("runtime checks do not use head-based early passes", "| head" not in launch)
check("map message is runtime-validated", 'topic_ok "topic /map"' in launch)
check("map type is OccupancyGrid-validated", "nav_msgs/msg/OccupancyGrid" in launch and
      'topic type /map' in launch)
check("map sample structure is OccupancyGrid-shaped", '[[ "$sample" != *"info:"* ]]' in launch and
      '[[ "$sample" != *"data:"* ]]' in launch)
check("map to odom TF is runtime-validated", 'tf_ok "TF map -> odom" "map" "odom"' in launch)
check("odom topic is runtime-validated", 'topic_ok "topic /lunabot/odom"' in launch)
check("odom to chassis TF is runtime-validated", 'tf_ok "TF odom -> chassis" "odom" "chassis"' in launch)
check("LaserScan topic is runtime-validated", 'topic_ok "topic /lunabot/lidar/scan"' in launch)
check("camera topic is runtime-validated", 'topic_ok "topic /lunabot/camera/image_raw"' in launch)
check("IMU topic is runtime-validated", 'topic_ok "topic /lunabot/imu"' in launch)
check("IMU failure diagnostics are preserved", "imu_gazebo_topics.txt" in launch and
      "imu_topic_info.txt" in launch and "imu_diagnostics.txt" in launch)
check("sensor-frame TF is runtime-validated", 'tf_ok "TF chassis -> sensor_head"' in launch)
check("scoped LaserScan TF is runtime-validated", 'tf_ok "TF sensor_head -> scoped LaserScan frame"' in launch and
      "lunabot_v4/sensor_head/lidar" in launch)
check("map motion has before and after samples", "map_before_motion.txt" in launch and
      "map_after_motion.txt" in launch)
check("map motion gate compares real samples", "cmp -s" in launch and
      "live map updates during rover motion: PASS" in launch)
check("demo failures propagate to process exit", "demo_rc" in launch and
      'shutdown "$EXIT_CODE"' in launch)
check("clean shutdown message is explicit", "Launch C environment cleanly closed." in launch)

check("map saver package is detected without a false pass",
      "MAP_SAVER_AVAILABLE" in launch and "nav2_map_server" in launch)
check("official map_saver_cli is used", "map_saver_cli -f" in launch)
check("map saver result is recorded", "map_saver.log" in launch)
check("final map saves happen before SLAM shutdown", "save_final_map" in launch and
      "saved map evidence (YAML + PGM): PASS" in launch)
check("map evidence path is documented", "phase_c_map.yaml" in docs and
      "phase_c_map.pgm" in docs)

check("Phase C RViz fixed frame is odom", "Fixed Frame: odom" in rviz)
check("Phase C RViz has a map display", "rviz_default_plugins/Map" in rviz and
      "Name: SLAM Map" in rviz)
check("Phase C RViz map topic is explicit", "Value: /map" in rviz)
check("Phase C RViz LaserScan topic is explicit", "Value: /lunabot/lidar/scan" in rviz)
check("Phase C RViz camera topic is explicit", "Value: /lunabot/camera/image_raw" in rviz)
check("Phase C RViz sensor QoS remains best effort", rviz.count("Reliability Policy: Best Effort") >= 2)
check("Phase C RViz odometry topic is explicit", "Value: /lunabot/odom" in rviz)
check("Phase C RViz TF tree includes map and odom", "map:\n          odom:" in rviz)

for phrase, name in [
    ("sudo apt install -y", "Ubuntu installation commands"),
    ("ros-humble-slam-toolbox", "slam toolbox package instruction"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-c", "automated launch instruction"),
    ("live map updates during rover motion: PASS", "motion-map acceptance"),
    ("map -> odom", "map TF acceptance"),
    ("/lunabot/imu", "IMU acceptance"),
    ("clean relaunch", "relaunch instruction"),
    ("before Phase D", "Phase D gate"),
]:
    check(f"docs contain {name}", phrase in docs)

check("validator states that static checks are not runtime acceptance",
      "does not claim" in read("tools/validate_phase_c.py") and
      "runtime" in read("tools/validate_phase_c.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE C STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-c-launch-c/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
```

## `tools/validate_phase_d.py`

```python
"""Static Phase D gate: A* autonomous navigation.

This validator checks the independent Phase D repository contract and the
inherited Phase A/B/C safety interfaces. It does not claim that a real rover
reached a goal; that requires EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d.
"""

from pathlib import Path
import ast
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
checks = []


def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))


def read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)


required = [
    "launch-a", "launch-b", "launch-c", "launch-d",
    "scripts/launch-a.sh", "scripts/launch-b.sh", "scripts/launch-c.sh",
    "scripts/launch-d.sh", "scripts/wasd_teleop.py",
    "scripts/control_odometry.py", "scripts/odometry_monitor.py",
    "scripts/astar_navigation.py", "config/slam_toolbox_phase_c.yaml",
    "rviz/phase_a.rviz", "rviz/phase_b.rviz", "rviz/phase_c.rviz",
    "rviz/phase_d.rviz", "tools/validate_phase_a.py",
    "tools/validate_phase_b.py", "tools/validate_phase_c.py",
    "docs/phase-3-launch-c.md", "docs/phase-4-launch-d.md",
    "evidence/phase-c-launch-c/README.md",
    "evidence/phase-d-launch-d/README.md",
    "evidence/phase-d-launch-d/verification_checklist.md",
    "src/lunabot_gazebo/worlds/lunar_world.sdf",
    "src/lunabot_gazebo/models/lunabot_v4/model.sdf",
]
for rel in required:
    check(f"file exists: {rel}", (ROOT / rel).is_file())

for rel in ["launch-d", "scripts/launch-d.sh", "scripts/astar_navigation.py",
            "tools/validate_phase_d.py"]:
    p = ROOT / rel
    check(f"executable: {rel}", p.is_file() and bool(p.stat().st_mode & 0o111))

for rel in ["scripts/wasd_teleop.py", "scripts/control_odometry.py",
            "scripts/odometry_monitor.py", "scripts/astar_navigation.py",
            "tools/validate_phase_a.py", "tools/validate_phase_b.py",
            "tools/validate_phase_c.py"]:
    try:
        ast.parse(read(rel))
        check(f"Python syntax: {rel}", True)
    except Exception as exc:
        check(f"Python syntax: {rel}", False, str(exc))

for rel in ["launch-a", "launch-b", "launch-c", "launch-d",
            "scripts/launch-a.sh", "scripts/launch-b.sh",
            "scripts/launch-c.sh", "scripts/launch-d.sh"]:
    r = run(["bash", "-n", rel])
    check(f"bash syntax: {rel}", r.returncode == 0, r.stderr.strip())

for rel, kind in [("src/lunabot_gazebo/worlds/lunar_world.sdf", "world"),
                  ("src/lunabot_gazebo/models/lunabot_v4/model.sdf", "model")]:
    try:
        ET.parse(ROOT / rel)
        check(f"{kind} SDF well-formed", True)
    except Exception as exc:
        check(f"{kind} SDF well-formed", False, str(exc))

for rel, expected in [("tools/validate_phase_a.py", "80/80"),
                      ("tools/validate_phase_b.py", "103/103"),
                      ("tools/validate_phase_c.py", "133/133")]:
    r = run([sys.executable, rel])
    check(f"existing static gate remains green: {rel}",
          r.returncode == 0 and expected in r.stdout, r.stdout[-180:].strip())

launch = read("scripts/launch-d.sh")
wrapper = read("launch-d")
planner = read("scripts/astar_navigation.py")
rviz = read("rviz/phase_d.rviz")
world = read("src/lunabot_gazebo/worlds/lunar_world.sdf")
model = read("src/lunabot_gazebo/models/lunabot_v4/model.sdf")
docs = read("docs/phase-4-launch-d.md")

noncomment = "\n".join(line for line in launch.splitlines()
                           if not line.lstrip().startswith("#"))
check("root launch-d resolves its real path", "readlink -f" in wrapper)
check("root launch-d execs scripts/launch-d.sh", "scripts/launch-d.sh" in wrapper)
for old in ["launch-a", "launch-b", "launch-c"]:
    check(f"Phase D does not invoke {old}", old not in noncomment)
check("Phase D has its own evidence directory",
      "phase-d-launch-d" in launch and "phase-c-launch-c" not in noncomment)
check("Phase D is symlink-safe", 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"' in launch)
check("Phase D uses process groups", "setsid" in launch and "stop_group()" in launch)
check("Phase D has a shutdown trap", "trap 'shutdown 130' INT TERM" in launch)
check("Phase D shuts down navigation before SLAM", 'stop_group "$NAV_PID"' in launch and
      launch.index('stop_group "$NAV_PID"') < launch.index('stop_group "$SLAM_PID"'))
check("Phase D has bounded shutdown", 'kill -KILL -"$pid"' in launch)
check("Phase D interrupts and cleans the goal wait", "GOAL_WAIT_PID" in launch and
      'stop_group "$GOAL_WAIT_PID"' in launch)
check("Phase D cleans stale navigation nodes", "scripts/astar_navigation.py" in launch)
check("Phase D avoids head-based early passes", "| head" not in launch)

check("slam_toolbox remains the only SLAM system", "slam_toolbox" in launch and
      "slam_toolbox" in read("config/slam_toolbox_phase_c.yaml"))
for forbidden in ["cartographer", "nav2_amcl", "hector_slam", "karto_slam", "rtabmap"]:
    check(f"no second stack: {forbidden}", forbidden not in (launch + planner).lower())
check("no AMCL runtime", "amcl" not in (launch + planner).lower())
check("Phase D starts slam_toolbox directly", "ros2 launch slam_toolbox online_async_launch.py" in launch)
check("Phase D uses the approved SLAM config", 'slam_params_file:="$SLAM_CONFIG"' in launch)
check("Phase D uses simulated time", "use_sim_time:=true" in launch)

check("planner has a ROS node", "class AStarNavigation(Node)" in planner)
check("planner consumes OccupancyGrid", "OccupancyGrid" in planner and "_map_callback" in planner)
check("planner implements a heap-based open set", "heapq" in planner and "open_set" in planner)
check("planner tracks A* g scores", "g_score" in planner and "came_from" in planner)
check("planner uses eight-connected neighbors", "(1, 1), (1, -1), (-1, 1), (-1, -1)" in planner)
check("planner publishes nav_msgs Path", "Path" in planner and "self.path_pub.publish(path)" in planner)
check("planner publishes goal pose", "self.goal_pub.publish(goal)" in planner)
check("planner consumes goal pose", "PoseStamped" in planner and "_goal_callback" in planner)
check("planner uses real odometry", "Odometry" in planner and "self.odom_msg" in planner)
check("planner transforms odom pose into map", "lookup_transform" in planner and
      "self.map_frame" in planner)
check("planner handles map origin orientation", "_world_to_grid" in planner and
      "origin.orientation" in planner)
check("planner rejects occupied cells", "occupied_threshold" in planner and
      "_cell_is_free" in planner)
check("planner handles unknown cells explicitly", "unknown_is_obstacle" in planner)
check("planner inflates obstacles", "inflation_radius" in planner and "_safe_cell" in planner)
check("planner has goal tolerance", "goal_tolerance" in planner and "GOAL_REACHED" in planner)
check("planner publishes conservative cmd_vel_in", "cmd_topic" in planner and
      "self.cmd_pub.publish(cmd)" in planner)
check("planner stops on goal", "self._publish_stop()" in planner and "goal_distance" in planner)
check("planner publishes auditable status", "/lunabot/navigation/status" in planner and
      "PLANNING_PASS" in planner and "NO_PATH" in planner)
check("planner provides a deterministic auto goal", "auto_goal_distance" in planner and
      "AUTO_GOAL_SENT" in planner)
check("GUI defaults to operator-selected goal", 'AUTO_GOAL="${AUTO_GOAL:-false}"' in launch and
      'if [ "$DEMO" = "1" ]; then' in launch and "AUTO_GOAL=true" in launch)
check("planner uses transient-local map input", "TRANSIENT_LOCAL" in planner and
      "map_qos" in planner)
check("planner makes path/status late-join safe and republishes goals",
      planner.count("latched_qos") >= 1 and "self.status_pub" in planner and
      "self.path_pub" in planner and "goal_pub.publish(self.goal_msg)" in planner)
check("planner has no direct Gazebo dependency", "ignition" not in planner.lower() and
      "gazebo" not in planner.lower())

check("Phase D uses the validated world", 'WORLD_PATH="$WORLD_DIR/lunar_world.sdf"' in launch)
check("Phase D resolves its A* planner path", 'NAV_PATH="$REPO_DIR/scripts/astar_navigation.py"' in launch)
check("Phase D preserves spawn height", 'SPAWN_Z="-2.308"' in launch)

check("Phase D supports headless Gazebo", 'HEADLESS="${HEADLESS:-0}"' in launch and
      "gazebo -s" in launch)
check("Phase D explicitly unpauses Gazebo", "WorldControl" in launch and
      "--req 'pause: false'" in launch)
check("Phase D detects both bridge families", "ros_ign_bridge" in launch and
      "ros_gz_bridge" in launch)
check("Phase D bridges clock", '"/clock@rosgraph_msgs/msg/Clock' in launch)
check("Phase D bridges LaserScan", '"/lunabot/lidar/scan@sensor_msgs/msg/LaserScan' in launch)
check("Phase D bridges camera", '"/lunabot/camera/image_raw@sensor_msgs/msg/Image' in launch)
check("Phase D bridges IMU", '"/lunabot/imu@sensor_msgs/msg/Imu' in launch)
check("Phase D bridges odometry", '"/lunabot/odom@nav_msgs/msg/Odometry' in launch)
check("Phase D keeps scoped LaserScan TF", '"sensor_head|lunabot_v4/sensor_head/lidar|' in launch)
check("Phase D keeps controller input boundary", "--input-topic /cmd_vel_in" in launch)
check("Phase D keeps controller output boundary", "--output-topic /cmd_vel" in launch)
check("Phase D keeps watchdog", "--watchdog-sec 0.5" in launch)
check("Phase D starts odometry monitor", "odometry_monitor.py" in launch and "ODOM_PID" in launch)
check("Phase D keeps Fortress IMU plugin", "ignition-gazebo-imu-system" in world and
      "ignition::gazebo::systems::Imu" in world)
check("model odometry contract is unchanged", "<odom_topic>/lunabot/odom</odom_topic>" in model and
      "<child_frame_id>chassis</child_frame_id>" in model)

check("runtime validates real topic messages", 'sample="$(timeout 30 ros2 topic echo' in launch)
check("runtime has no head-based topic validation", "| head" not in launch)
check("runtime validates OccupancyGrid type", "nav_msgs/msg/OccupancyGrid" in launch)
check("runtime validates map structure", '"info:"' in launch and '"data:"' in launch)
check("runtime validates map TF", 'tf_ok "TF map -> odom" "map" "odom"' in launch)
check("runtime validates odom TF", 'tf_ok "TF odom -> chassis" "odom" "chassis"' in launch)
check("runtime validates scoped lidar TF", "TF sensor_head -> scoped LaserScan frame" in launch)
check("runtime validates IMU", 'topic_ok "topic /lunabot/imu"' in launch)
check("runtime validates Camera and LaserScan", 'topic_ok "topic /lunabot/camera/image_raw"' in launch and
      'topic_ok "topic /lunabot/lidar/scan"' in launch)
check("runtime validates A* goal/path/status", 'topic_ok "topic /goal_pose"' in launch and
      'topic_ok "topic /plan"' in launch and
      'topic_ok "topic /lunabot/navigation/status"' in launch)
check("runtime validates A* message types", 'type_ok "type /goal_pose geometry_msgs/PoseStamped"' in launch and
      'type_ok "type /plan nav_msgs/Path"' in launch and
      'type_ok "type navigation status std_msgs/String"' in launch)
check("runtime requires a non-empty path", '[[ "$sample" == *"poses: []"* ]]' in launch)
check("runtime waits for goal reached", "wait_for_goal" in launch and "GOAL_REACHED" in launch)
check("runtime compares map before/after A*", "map_before_navigation.txt" in launch and
      "map_after_navigation.txt" in launch and "map changed during A* navigation" in launch)
check("runtime saves Phase D map evidence", "phase_d_map.yaml" in launch and
      "map_saver_cli" in launch)
check("demo failure affects exit code", "EXIT_CODE=1" in launch and
      "PHASE D RUN COMPLETE" in launch)

check("Phase D RViz uses map fixed frame", "Fixed Frame: map" in rviz)
check("Phase D RViz has SLAM Map", "rviz_default_plugins/Map" in rviz and
      "Value: /map" in rviz)
check("Phase D RViz has A* Path", "rviz_default_plugins/Path" in rviz and
      "Name: A* Path" in rviz and "Value: /plan" in rviz)
check("Phase D RViz has explicit LaserScan", "Value: /lunabot/lidar/scan" in rviz)
check("Phase D RViz has explicit Camera", "Value: /lunabot/camera/image_raw" in rviz)
check("Phase D RViz uses best effort sensor QoS", rviz.count("Reliability Policy: Best Effort") >= 2)
check("Phase D RViz has explicit odometry", "Value: /lunabot/odom" in rviz)
check("Phase D RViz provides Set Goal tool", "rviz_default_plugins/SetGoal" in rviz and
      "Topic: /goal_pose" in rviz)
check("Phase D RViz displays selected goal", "Name: Selected Goal" in rviz and
      "Value: /goal_pose" in rviz)
check("Phase D RViz stores map/odom TF", "map:\n          odom:" in rviz)

for phrase, name in [
    ("sudo apt install -y", "installation"),
    ("EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d", "automated command"),
    ("A* goal reached: PASS", "goal acceptance"),
    ("/plan", "path output"),
    ("/cmd_vel_in", "controller boundary"),
    ("live map updates during autonomous navigation", "map motion acceptance"),
    ("phase_d_map.yaml", "map evidence"),
    ("clean relaunch", "relaunch"),
    ("before Phase E", "Phase E gate"),
]:
    check(f"docs contain {name}", phrase in docs)
check("validator is honest about runtime", "does not claim" in read("tools/validate_phase_d.py") and
      "runtime" in read("tools/validate_phase_d.py"))

passed = sum(ok for _, ok, _ in checks)
print("=" * 64)
print("LUNABOT V4 PHASE D STATIC VALIDATION")
print("=" * 64)
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    print(f"[{('PASS' if ok else 'FAIL')}] {name}{suffix}")
print("=" * 64)
print(f"RESULT: {passed}/{len(checks)} checks passed - " +
      ("ALL PASS" if passed == len(checks) else "FAIL"))
print("=" * 64)
report = ROOT / "evidence/phase-d-launch-d/static_validation.txt"
report.write_text("\n".join(
    f"[{('PASS' if ok else 'FAIL')}] {name}{(' - ' + detail) if detail else ''}"
    for name, ok, detail in checks
) + f"\n\nRESULT: {passed}/{len(checks)} checks passed\n", encoding="utf-8")
print(f"report: {report}")
sys.exit(0 if passed == len(checks) else 1)
```

## `config/slam_toolbox_phase_c.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: chassis
    scan_topic: /lunabot/lidar/scan
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05
    min_laser_range: 0.15
    max_laser_range: 60.0
    minimum_time_interval: 0.1
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.20
    minimum_travel_heading: 0.20
    check_min_dist_and_heading_precisely: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 60.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 8.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

## `src/lunabot_gazebo/worlds/lunar_world.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="lunar_world">

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -300 200 0 0.55 1.57</pose>
        <clip>
          <near>1.0</near>
          <far>5000.0</far>
        </clip>
      </camera>
    </gui>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -1.62</gravity>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>


    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>

    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 200 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.5 -0.6</direction>
    </light>


    <model name="lunar_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://meshes/lunar_terrain_collision.obj</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://meshes/lunar_terrain.obj</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>


    <model name="lunar_habitat_main">
      <static>true</static>
      <pose>-3.5 3.5 -1.65 0 0 0</pose>
      <link name="habitat_body">
        <collision name="body_collision"><geometry><box><size>4.0 3.0 2.3</size></box></geometry></collision>
        <visual name="body_visual"><geometry><box><size>4.0 3.0 2.3</size></box></geometry><material><ambient>0.32 0.36 0.42 1</ambient><diffuse>0.32 0.36 0.42 1</diffuse></material></visual>
        <visual name="roof_visual"><pose>0 0 1.25 0 0 0</pose><geometry><box><size>4.4 3.4 0.18</size></box></geometry><material><ambient>0.12 0.16 0.20 1</ambient><diffuse>0.12 0.16 0.20 1</diffuse></material></visual>
        <visual name="window_front"><pose>0 -1.51 0.25 0 0 0</pose><geometry><box><size>1.6 0.04 0.55</size></box></geometry><material><ambient>0.08 0.25 0.35 1</ambient><diffuse>0.08 0.25 0.35 1</diffuse></material></visual>
      </link>
    </model>

    <model name="lunar_habitat_equipment">
      <static>true</static>
      <pose>-3.0 -2.8 -2.18 0 0 0</pose>
      <link name="equipment_body">
        <collision name="equipment_collision"><geometry><box><size>2.2 1.5 1.2</size></box></geometry></collision>
        <visual name="equipment_visual"><geometry><box><size>2.2 1.5 1.2</size></box></geometry><material><ambient>0.48 0.30 0.12 1</ambient><diffuse>0.48 0.30 0.12 1</diffuse></material></visual>
        <visual name="panel"><pose>0 0 0.72 0.25 0 0</pose><geometry><box><size>1.8 0.04 0.65</size></box></geometry><material><ambient>0.04 0.12 0.22 1</ambient><diffuse>0.04 0.12 0.22 1</diffuse></material></visual>
      </link>
    </model>

    <model name="presentation_obstacle_forward">
      <static>true</static>
      <pose>2.0 0.0 -2.39 0 0 0</pose>
      <link name="obstacle_body">
        <collision name="obstacle_collision"><geometry><box><size>0.95 0.95 0.80</size></box></geometry></collision>
        <visual name="obstacle_visual"><geometry><box><size>0.95 0.95 0.80</size></box></geometry><material><ambient>0.88 0.27 0.05 1</ambient><diffuse>0.88 0.27 0.05 1</diffuse></material></visual>
      </link>
    </model>

    <model name="presentation_rock_left">
      <static>true</static>
      <pose>1.4 2.1 -2.48 0 0 0</pose>
      <link name="rock_body">
        <collision name="rock_collision"><geometry><sphere><radius>0.55</radius></sphere></geometry></collision>
        <visual name="rock_visual"><geometry><sphere><radius>0.55</radius></sphere></geometry><material><ambient>0.28 0.29 0.30 1</ambient><diffuse>0.28 0.29 0.30 1</diffuse></material></visual>
      </link>
    </model>


    <model name="lunar_horizon">
      <static>true</static>
      <link name="horizon_link">
        <collision name="horizon_collision">
          <pose>0 0 -12.4 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <center>800 800</center>
            </plane>
          </geometry>
        </collision>
        <visual name="horizon_visual">
          <pose>0 0 -12.4 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <center>800 800</center>
            </plane>
          </geometry>
          <material>
            <ambient>0.42 0.42 0.42 1</ambient>
            <diffuse>0.42 0.42 0.42 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## `src/lunabot_gazebo/models/lunabot_v4/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="lunabot_v4">
    <pose>0 0 0.52 0 0 0</pose>


    <link name="chassis">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 -0.3 0 0 0</pose><mass>24.0</mass>
        <inertia><ixx>1.0216</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1.9018</iyy><iyz>0</iyz><izz>2.7298</izz></inertia>
      </inertial>
      <collision name="body_collision"><geometry><box><size>0.95 0.68 0.22</size></box></geometry></collision>
      <visual name="body_visual">
        <geometry><box><size>0.95 0.68 0.22</size></box></geometry>
        <material><ambient>0.82 0.82 0.80 1</ambient><diffuse>0.82 0.82 0.80 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
    </link>


    <link name="lower_armor">
      <pose relative_to="chassis">-0.05 0 -0.14 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.05714</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.13017</iyy><iyz>0</iyz><izz>0.18517</izz></inertia></inertial>
      <collision name="armor_collision"><geometry><box><size>0.88 0.58 0.08</size></box></geometry></collision>
      <visual name="armor_visual">
        <geometry><box><size>0.88 0.58 0.08</size></box></geometry>
        <material><ambient>0.15 0.15 0.17 1</ambient><diffuse>0.15 0.15 0.17 1</diffuse></material>
      </visual>
    </link>
    <joint name="lower_armor_joint" type="fixed"><parent>chassis</parent><child>lower_armor</child></joint>


    <link name="upper_deck">
      <pose relative_to="chassis">-0.03 0 0.17 0 0 0</pose>
      <inertial><mass>3.0</mass><inertia><ixx>0.0592</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.1172</iyy><iyz>0</iyz><izz>0.1732</izz></inertia></inertial>
      <collision name="deck_collision"><geometry><box><size>0.68 0.48 0.08</size></box></geometry></collision>
      <visual name="deck_visual">
        <geometry><box><size>0.68 0.48 0.08</size></box></geometry>
        <material><ambient>0.75 0.75 0.72 1</ambient><diffuse>0.75 0.75 0.72 1</diffuse></material>
      </visual>
    </link>
    <joint name="upper_deck_joint" type="fixed"><parent>chassis</parent><child>upper_deck</child></joint>


    <link name="front_armor">
      <pose relative_to="chassis">0.51 0 -0.01 0 0.32 0</pose>
      <inertial><mass>1.4</mass><inertia><ixx>0.0677</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01349</iyy><iyz>0</iyz><izz>0.0602</izz></inertia></inertial>
      <collision name="front_armor_collision"><geometry><box><size>0.16 0.70 0.30</size></box></geometry></collision>
      <visual name="front_armor_visual">
        <geometry><box><size>0.16 0.70 0.30</size></box></geometry>
        <material><ambient>0.72 0.55 0.15 1</ambient><diffuse>0.72 0.55 0.15 1</diffuse><specular>0.8 0.7 0.4 1</specular></material>
      </visual>
    </link>
    <joint name="front_armor_joint" type="fixed"><parent>chassis</parent><child>front_armor</child></joint>


    <link name="left_side_rail">
      <pose relative_to="chassis">-0.02 0.37 0.05 0 0 0</pose>
      <inertial><mass>0.5</mass><inertia><ixx>0.000684</ixx><iyy>0.02862</iyy><izz>0.02810</izz></inertia></inertial>
      <visual name="rail_visual"><geometry><box><size>0.82 0.045 0.12</size></box></geometry><material><ambient>0.08 0.09 0.10 1</ambient></material></visual>
    </link>
    <joint name="left_side_rail_joint" type="fixed"><parent>chassis</parent><child>left_side_rail</child></joint>

    <link name="right_side_rail">
      <pose relative_to="chassis">-0.02 -0.37 0.05 0 0 0</pose>
      <inertial><mass>0.5</mass><inertia><ixx>0.000684</ixx><iyy>0.02862</iyy><izz>0.02810</izz></inertia></inertial>
      <visual name="rail_visual"><geometry><box><size>0.82 0.045 0.12</size></box></geometry><material><ambient>0.08 0.09 0.10 1</ambient></material></visual>
    </link>
    <joint name="right_side_rail_joint" type="fixed"><parent>chassis</parent><child>right_side_rail</child></joint>


    <joint name="left_rocker_joint" type="revolute">
      <pose relative_to="chassis">0 0.40 -0.08 0 0 0</pose>
      <parent>chassis</parent><child>left_rocker</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-0.55</lower><upper>0.55</upper><effort>600</effort><velocity>1.0</velocity></limit>
        <dynamics><damping>400</damping><friction>50</friction></dynamics>
      </axis>
    </joint>
    <link name="left_rocker">
      <pose relative_to="left_rocker_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.8</mass><inertia><ixx>0.004019</ixx><iyy>0.12329</iyy><izz>0.12265</izz></inertia></inertial>
      <collision name="rocker_collision"><geometry><box><size>0.72 0.085 0.10</size></box></geometry></collision>
      <visual name="rocker_visual">
        <geometry><box><size>0.72 0.085 0.10</size></box></geometry>
        <material><ambient>0.38 0.40 0.43 1</ambient><diffuse>0.38 0.40 0.43 1</diffuse></material>
      </visual>
    </link>


    <joint name="right_rocker_joint" type="revolute">
      <pose relative_to="chassis">0 -0.40 -0.08 0 0 0</pose>
      <parent>chassis</parent><child>right_rocker</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-0.55</lower><upper>0.55</upper><effort>600</effort><velocity>1.0</velocity></limit>
        <dynamics><damping>400</damping><friction>50</friction></dynamics>
      </axis>
    </joint>
    <link name="right_rocker">
      <pose relative_to="right_rocker_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.8</mass><inertia><ixx>0.004019</ixx><iyy>0.12329</iyy><izz>0.12265</izz></inertia></inertial>
      <collision name="rocker_collision"><geometry><box><size>0.72 0.085 0.10</size></box></geometry></collision>
      <visual name="rocker_visual">
        <geometry><box><size>0.72 0.085 0.10</size></box></geometry>
        <material><ambient>0.38 0.40 0.43 1</ambient><diffuse>0.38 0.40 0.43 1</diffuse></material>
      </visual>
    </link>


    <joint name="left_bogie_joint" type="revolute">
      <pose relative_to="left_rocker">0.20 0 0 0 0 0</pose>
      <parent>left_rocker</parent><child>left_bogie</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-0.65</lower><upper>0.65</upper><effort>450</effort><velocity>1.2</velocity></limit>
        <dynamics><damping>150</damping><friction>30</friction></dynamics>
      </axis>
    </joint>
    <link name="left_bogie">
      <pose relative_to="left_bogie_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.001875</ixx><iyy>0.065</iyy><izz>0.065</izz></inertia></inertial>
      <collision name="bogie_collision"><geometry><box><size>0.62 0.075 0.075</size></box></geometry></collision>
      <visual name="bogie_visual">
        <geometry><box><size>0.62 0.075 0.075</size></box></geometry>
        <material><ambient>0.45 0.47 0.50 1</ambient><diffuse>0.45 0.47 0.50 1</diffuse></material>
      </visual>
    </link>


    <joint name="right_bogie_joint" type="revolute">
      <pose relative_to="right_rocker">0.20 0 0 0 0 0</pose>
      <parent>right_rocker</parent><child>right_bogie</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-0.65</lower><upper>0.65</upper><effort>450</effort><velocity>1.2</velocity></limit>
        <dynamics><damping>150</damping><friction>30</friction></dynamics>
      </axis>
    </joint>
    <link name="right_bogie">
      <pose relative_to="right_bogie_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.001875</ixx><iyy>0.065</iyy><izz>0.065</izz></inertia></inertial>
      <collision name="bogie_collision"><geometry><box><size>0.62 0.075 0.075</size></box></geometry></collision>
      <visual name="bogie_visual">
        <geometry><box><size>0.62 0.075 0.075</size></box></geometry>
        <material><ambient>0.45 0.47 0.50 1</ambient><diffuse>0.45 0.47 0.50 1</diffuse></material>
      </visual>
    </link>


    <joint name="left_front_steer_joint" type="revolute">
      <pose relative_to="left_bogie">0.20 0 -0.18 0 0 0</pose>
      <parent>left_bogie</parent><child>left_front_knuckle</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-0.6</lower><upper>0.6</upper><effort>80</effort><velocity>2.0</velocity></limit>
        <dynamics><damping>2</damping><friction>0.5</friction></dynamics>
      </axis>
    </joint>
    <link name="left_front_knuckle">
      <pose relative_to="left_front_steer_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.00018</ixx><iyy>0.00025</iyy><izz>0.00025</izz></inertia></inertial>
      <visual name="knuckle_visual"><geometry><box><size>0.08 0.06 0.06</size></box></geometry><material><ambient>0.5 0.5 0.52 1</ambient></material></visual>
    </link>
    <joint name="left_front_wheel_joint" type="revolute">
      <pose relative_to="left_front_knuckle">0 0 0 0 0 0</pose>
      <parent>left_front_knuckle</parent><child>left_front_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="left_front_wheel">
      <pose relative_to="left_front_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision">
        <geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry>
        <surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction></surface>
      </collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>


    <joint name="right_front_steer_joint" type="revolute">
      <pose relative_to="right_bogie">0.20 0 -0.18 0 0 0</pose>
      <parent>right_bogie</parent><child>right_front_knuckle</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-0.6</lower><upper>0.6</upper><effort>80</effort><velocity>2.0</velocity></limit>
        <dynamics><damping>2</damping><friction>0.5</friction></dynamics>
      </axis>
    </joint>
    <link name="right_front_knuckle">
      <pose relative_to="right_front_steer_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.00018</ixx><iyy>0.00025</iyy><izz>0.00025</izz></inertia></inertial>
      <visual name="knuckle_visual"><geometry><box><size>0.08 0.06 0.06</size></box></geometry><material><ambient>0.5 0.5 0.52 1</ambient></material></visual>
    </link>
    <joint name="right_front_wheel_joint" type="revolute">
      <pose relative_to="right_front_knuckle">0 0 0 0 0 0</pose>
      <parent>right_front_knuckle</parent><child>right_front_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="right_front_wheel">
      <pose relative_to="right_front_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision">
        <geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry>
        <surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction></surface>
      </collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>


    <joint name="left_rear_steer_joint" type="revolute">
      <pose relative_to="left_rocker">-0.40 0 -0.18 0 0 0</pose>
      <parent>left_rocker</parent><child>left_rear_knuckle</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-0.6</lower><upper>0.6</upper><effort>80</effort><velocity>2.0</velocity></limit>
        <dynamics><damping>2</damping><friction>0.5</friction></dynamics>
      </axis>
    </joint>
    <link name="left_rear_knuckle">
      <pose relative_to="left_rear_steer_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.00018</ixx><iyy>0.00025</iyy><izz>0.00025</izz></inertia></inertial>
      <visual name="knuckle_visual"><geometry><box><size>0.08 0.06 0.06</size></box></geometry><material><ambient>0.5 0.5 0.52 1</ambient></material></visual>
    </link>
    <joint name="left_rear_wheel_joint" type="revolute">
      <pose relative_to="left_rear_knuckle">0 0 0 0 0 0</pose>
      <parent>left_rear_knuckle</parent><child>left_rear_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="left_rear_wheel">
      <pose relative_to="left_rear_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision">
        <geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry>
        <surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction></surface>
      </collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>


    <joint name="right_rear_steer_joint" type="revolute">
      <pose relative_to="right_rocker">-0.40 0 -0.18 0 0 0</pose>
      <parent>right_rocker</parent><child>right_rear_knuckle</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-0.6</lower><upper>0.6</upper><effort>80</effort><velocity>2.0</velocity></limit>
        <dynamics><damping>2</damping><friction>0.5</friction></dynamics>
      </axis>
    </joint>
    <link name="right_rear_knuckle">
      <pose relative_to="right_rear_steer_joint">0 0 0 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.00018</ixx><iyy>0.00025</iyy><izz>0.00025</izz></inertia></inertial>
      <visual name="knuckle_visual"><geometry><box><size>0.08 0.06 0.06</size></box></geometry><material><ambient>0.5 0.5 0.52 1</ambient></material></visual>
    </link>
    <joint name="right_rear_wheel_joint" type="revolute">
      <pose relative_to="right_rear_knuckle">0 0 0 0 0 0</pose>
      <parent>right_rear_knuckle</parent><child>right_rear_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="right_rear_wheel">
      <pose relative_to="right_rear_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision">
        <geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry>
        <surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction></surface>
      </collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>


    <joint name="left_middle_wheel_joint" type="revolute">
      <pose relative_to="left_bogie">-0.20 0 -0.18 0 0 0</pose>
      <parent>left_bogie</parent><child>left_middle_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="left_middle_wheel">
      <pose relative_to="left_middle_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2></ode></friction></surface></collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>

    <joint name="right_middle_wheel_joint" type="revolute">
      <pose relative_to="right_bogie">-0.20 0 -0.18 0 0 0</pose>
      <parent>right_bogie</parent><child>right_middle_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit><lower>-100000</lower><upper>100000</upper><effort>160</effort><velocity>25</velocity></limit>
        <dynamics><damping>3.0</damping><friction>0.03</friction></dynamics>
      </axis>
    </joint>
    <link name="right_middle_wheel">
      <pose relative_to="right_middle_wheel_joint">0 0 0 1.5708 0 0</pose>
      <inertial><mass>2.2</mass><inertia><ixx>0.01899</ixx><iyy>0.01899</iyy><izz>0.03179</izz></inertia></inertial>
      <collision name="wheel_collision"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><surface><friction><ode><mu>4.0</mu><mu2>4.0</mu2></ode></friction></surface></collision>
      <visual name="wheel_visual"><geometry><cylinder><radius>0.17</radius><length>0.13</length></cylinder></geometry><material><ambient>0.05 0.05 0.055 1</ambient></material></visual>
    </link>


    <link name="left_solar_panel">
      <pose relative_to="chassis">-0.05 0.54 0.18 0.12 0 0</pose>
      <inertial><mass>0.8</mass><inertia><ixx>0.006912</ixx><iyy>0.03084</iyy><izz>0.02409</izz></inertia></inertial>
      <visual name="panel"><geometry><box><size>0.60 0.035 0.32</size></box></geometry><material><ambient>0.02 0.05 0.12 1</ambient><diffuse>0.02 0.05 0.12 1</diffuse><specular>0.5 0.55 0.6 1</specular></material></visual>
    </link>
    <joint name="left_solar_panel_joint" type="fixed"><parent>chassis</parent><child>left_solar_panel</child></joint>

    <link name="right_solar_panel">
      <pose relative_to="chassis">-0.05 -0.54 0.18 -0.12 0 0</pose>
      <inertial><mass>0.8</mass><inertia><ixx>0.006912</ixx><iyy>0.03084</iyy><izz>0.02409</izz></inertia></inertial>
      <visual name="panel"><geometry><box><size>0.60 0.035 0.32</size></box></geometry><material><ambient>0.02 0.05 0.12 1</ambient><diffuse>0.02 0.05 0.12 1</diffuse><specular>0.5 0.55 0.6 1</specular></material></visual>
    </link>
    <joint name="right_solar_panel_joint" type="fixed"><parent>chassis</parent><child>right_solar_panel</child></joint>


    <link name="sensor_mast">
      <pose relative_to="chassis">0.18 0 0.54 0 0 0</pose>
      <inertial><mass>1.4</mass><inertia><ixx>0.036</ixx><iyy>0.036</iyy><izz>0.0014175</izz></inertia></inertial>
      <visual name="mast"><geometry><cylinder><radius>0.045</radius><length>0.55</length></cylinder></geometry><material><ambient>0.7 0.55 0.2 1</ambient><diffuse>0.7 0.55 0.2 1</diffuse><specular>0.8 0.7 0.4 1</specular></material></visual>
    </link>
    <joint name="sensor_mast_joint" type="fixed"><parent>chassis</parent><child>sensor_mast</child></joint>


    <joint name="mast_pan_joint" type="revolute">
      <pose relative_to="sensor_mast">0 0 0.28 0 0 0</pose>
      <parent>sensor_mast</parent><child>mast_pan_link</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-3.14</lower><upper>3.14</upper><effort>15</effort><velocity>1.0</velocity></limit><dynamics><damping>1</damping></dynamics></axis>
    </joint>
    <link name="mast_pan_link">
      <pose relative_to="mast_pan_joint">0 0 0 0 0 0</pose>
      <inertial><mass>0.2</mass><inertia><ixx>0.0001217</ixx><iyy>0.0001217</iyy><izz>0.00016</izz></inertia></inertial>
      <visual name="pan_visual"><geometry><cylinder><radius>0.04</radius><length>0.05</length></cylinder></geometry><material><ambient>0.5 0.5 0.52 1</ambient></material></visual>
    </link>


    <joint name="mast_tilt_joint" type="revolute">
      <pose relative_to="mast_pan_link">0 0 0.03 0 0 0</pose>
      <parent>mast_pan_link</parent><child>sensor_head</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-0.6</lower><upper>0.6</upper><effort>10</effort><velocity>1.0</velocity></limit><dynamics><damping>1</damping></dynamics></axis>
    </joint>
    <link name="sensor_head">
      <pose relative_to="mast_tilt_joint">0 0 0 0 0 0</pose>
      <inertial><mass>1.0</mass><inertia><ixx>0.00567</ixx><iyy>0.00817</iyy><izz>0.01057</izz></inertia></inertial>
      <visual name="head"><geometry><box><size>0.28 0.22 0.14</size></box></geometry><material><ambient>0.06 0.06 0.07 1</ambient></material></visual>

      <sensor name="rgb_camera" type="camera">
        <pose relative_to="sensor_head">0.14 0 0 0 0 0</pose>
        <update_rate>20</update_rate>
        <camera><horizontal_fov>1.047</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.05</near><far>80</far></clip></camera>
        <always_on>true</always_on><visualize>true</visualize><topic>/lunabot/camera/image_raw</topic>
      </sensor>

      <sensor name="depth_camera" type="depth_camera">
        <pose relative_to="sensor_head">0.14 0 -0.04 0 0 0</pose>
        <update_rate>15</update_rate>
        <camera><horizontal_fov>1.047</horizontal_fov><image><width>640</width><height>480</height></image><clip><near>0.10</near><far>40</far></clip></camera>
        <always_on>true</always_on><visualize>true</visualize><topic>/lunabot/depth/image_raw</topic>
      </sensor>

      <sensor name="lidar" type="gpu_lidar">
        <pose relative_to="sensor_head">0.02 0 0.09 0 0.5 0</pose>
        <update_rate>10</update_rate>
        <topic>/lunabot/lidar/scan</topic>
        <ray><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan><range><min>0.15</min><max>60</max><resolution>0.01</resolution></range></ray>
        <always_on>true</always_on><visualize>true</visualize>
      </sensor>
    </link>


    <link name="antenna">
      <pose relative_to="chassis">-0.35 0 0.40 0 0 0</pose>
      <inertial><mass>0.3</mass><inertia><ixx>0.002584</ixx><iyy>0.002584</iyy><izz>0.0000486</izz></inertia></inertial>
      <visual name="antenna_stem"><geometry><cylinder><radius>0.018</radius><length>0.32</length></cylinder></geometry><material><ambient>0.45 0.45 0.45 1</ambient></material></visual>
    </link>
    <joint name="antenna_joint" type="fixed"><parent>chassis</parent><child>antenna</child></joint>


    <link name="imu_link">
      <pose relative_to="chassis">0 0 0.10 0 0 0</pose>
      <inertial><mass>0.05</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity><x><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></x><y><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></y><z><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></z></angular_velocity>
          <linear_acceleration><x><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></x><y><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></y><z><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></z></linear_acceleration>
        </imu>
        <topic>/lunabot/imu</topic>
      </sensor>
    </link>
    <joint name="imu_joint" type="fixed"><parent>chassis</parent><child>imu_link</child></joint>


    <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_front_wheel_joint</left_joint>
      <left_joint>left_middle_wheel_joint</left_joint>
      <left_joint>left_rear_wheel_joint</left_joint>
      <right_joint>right_front_wheel_joint</right_joint>
      <right_joint>right_middle_wheel_joint</right_joint>
      <right_joint>right_rear_wheel_joint</right_joint>
      <wheel_separation>0.80</wheel_separation>
      <wheel_radius>0.17</wheel_radius>
      <max_linear_velocity>0.45</max_linear_velocity>
      <min_linear_velocity>-0.45</min_linear_velocity>
      <max_angular_velocity>1.0</max_angular_velocity>
      <min_angular_velocity>-1.0</min_angular_velocity>
      <max_linear_acceleration>0.4</max_linear_acceleration>
      <max_angular_acceleration>0.8</max_angular_acceleration>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>/cmd_vel</topic>
      <odom_topic>/lunabot/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>chassis</child_frame_id>
    </plugin>


    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>left_front_steer_joint</joint_name><topic>/lunabot/steer/front_left</topic><p_gain>8</p_gain><i_gain>0.1</i_gain><d_gain>0.2</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>right_front_steer_joint</joint_name><topic>/lunabot/steer/front_right</topic><p_gain>8</p_gain><i_gain>0.1</i_gain><d_gain>0.2</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>left_rear_steer_joint</joint_name><topic>/lunabot/steer/rear_left</topic><p_gain>8</p_gain><i_gain>0.1</i_gain><d_gain>0.2</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>right_rear_steer_joint</joint_name><topic>/lunabot/steer/rear_right</topic><p_gain>8</p_gain><i_gain>0.1</i_gain><d_gain>0.2</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>mast_pan_joint</joint_name><topic>/lunabot/mast/pan</topic><p_gain>5</p_gain><i_gain>0.05</i_gain><d_gain>0.1</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>mast_tilt_joint</joint_name><topic>/lunabot/mast/tilt</topic><p_gain>5</p_gain><i_gain>0.05</i_gain><d_gain>0.1</d_gain>
    </plugin>
    <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">
      <joint_name>left_front_wheel_joint</joint_name><joint_name>left_middle_wheel_joint</joint_name><joint_name>left_rear_wheel_joint</joint_name><joint_name>right_front_wheel_joint</joint_name><joint_name>right_middle_wheel_joint</joint_name><joint_name>right_rear_wheel_joint</joint_name><joint_name>left_front_steer_joint</joint_name><joint_name>right_front_steer_joint</joint_name><joint_name>left_rear_steer_joint</joint_name><joint_name>right_rear_steer_joint</joint_name><joint_name>left_rocker_joint</joint_name><joint_name>right_rocker_joint</joint_name><joint_name>left_bogie_joint</joint_name><joint_name>right_bogie_joint</joint_name><joint_name>mast_pan_joint</joint_name><joint_name>mast_tilt_joint</joint_name><topic>/lunabot/joint_states</topic>
    </plugin>
  </model>
</sdf>
```

## `rviz/phase_a.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 10
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 40
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        odom:
          chassis:
            sensor_head:
              {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 80; 80
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/lidar/scan
        latched: false
        type: sensor_msgs/msg/LaserScan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Width: 640
      Name: Camera
      Normalize Range:
        Max: 1
        Min: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/camera/image_raw
        latched: false
        type: sensor_msgs/msg/Image
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Width: 640
      Name: Depth Camera
      Normalize Range:
        Max: 8
        Min: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/depth/image_raw
        latched: false
        type: sensor_msgs/msg/Image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 45
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.7123888
    Saved: ~
Window Geometry:
  Camera:
    collapsed: false
  Displays:
    collapsed: false
  Height: 840
  Width: 1280
  X: 60
  Y: 60
```

## `rviz/phase_b.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 10
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 40
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        odom:
          chassis:
            sensor_head:
              {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 80; 80
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/lidar/scan
        latched: false
        type: sensor_msgs/msg/LaserScan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.1
      Angle Tolerance: 0.1
      Shape:
        Alpha: 1
        Axes Length: 0.3
        Axes Radius: 0.03
        Head Length: 0.1
        Head Radius: 0.05
        Shaft Length: 0.1
        Shaft Radius: 0.02
        Value: Axes
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lunabot/odom
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Width: 640
      Name: Camera
      Normalize Range:
        Max: 1
        Min: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/camera/image_raw
        latched: false
        type: sensor_msgs/msg/Image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 45
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.7123888
    Saved: ~
Window Geometry:
  Camera:
    collapsed: false
  Displays:
    collapsed: false
  Height: 840
  Width: 1280
  X: 60
  Y: 60
```

## `rviz/phase_c.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 10
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 40
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          odom:
            chassis:
              sensor_head:
                {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 80; 80
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/lidar/scan
        latched: false
        type: sensor_msgs/msg/LaserScan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: SLAM Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic: false
      Use Timestamp: false
      Value: true
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.1
      Angle Tolerance: 0.1
      Shape:
        Alpha: 1
        Axes Length: 0.3
        Axes Radius: 0.03
        Head Length: 0.1
        Head Radius: 0.05
        Shaft Length: 0.1
        Shaft Radius: 0.02
        Value: Axes
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lunabot/odom
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Width: 640
      Name: Camera
      Normalize Range:
        Max: 1
        Min: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/camera/image_raw
        latched: false
        type: sensor_msgs/msg/Image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 45
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.7123888
    Saved: ~
Window Geometry:
  Camera:
    collapsed: false
  Displays:
    collapsed: false
  Height: 840
  Width: 1280
  X: 60
  Y: 60
```

## `rviz/phase_d.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 10
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 40
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          odom:
            chassis:
              sensor_head:
                {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 80; 80
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/lidar/scan
        latched: false
        type: sensor_msgs/msg/LaserScan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: SLAM Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic: false
      Use Timestamp: false
      Value: true
    - Class: rviz_default_plugins/Path
      Color: 0; 255; 0
      Enabled: true
      Head Diameter: 0.1
      Head Length: 0.1
      Length: 0.1
      Line Style: Lines
      Line Width: 0.03
      Name: A* Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
      Value: true
    - Class: rviz_default_plugins/Pose
      Arrow Color: 50; 255; 50
      Axes Length: 0.6
      Axes Radius: 0.08
      Enabled: true
      Head Length: 0.20
      Head Radius: 0.12
      Name: Selected Goal
      Shaft Length: 0.40
      Shaft Radius: 0.06
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
      Value: true
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.1
      Angle Tolerance: 0.1
      Shape:
        Alpha: 1
        Axes Length: 0.3
        Axes Radius: 0.03
        Head Length: 0.1
        Head Radius: 0.05
        Shaft Length: 0.1
        Shaft Radius: 0.02
        Value: Axes
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lunabot/odom
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Width: 640
      Name: Camera
      Normalize Range:
        Max: 1
        Min: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lunabot/camera/image_raw
        latched: false
        type: sensor_msgs/msg/Image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 45
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.7123888
    Saved: ~
Window Geometry:
  Camera:
    collapsed: false
  Displays:
    collapsed: false
  Height: 840
  Width: 1280
  X: 60
  Y: 60
```
