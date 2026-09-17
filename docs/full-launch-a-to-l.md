# LunaBot V4 — Complete Phase A-to-L Code and Command Guide

This is the consolidated operating manual for the repository. It explains the
code, launchers, ROS interfaces, validation commands, runtime modes, evidence,
and acceptance procedure from `launch-a` through `launch-l`.

The individual phase guides remain the authoritative detailed specifications:

- `docs/phase-a-launch-a.md`
- `docs/phase-b-launch-b.md`
- `docs/phase-3-launch-c.md`
- `docs/phase-4-launch-d.md`
- `docs/phase-5-launch-e.md`
- `docs/phase-6-launch-f.md`
- `docs/phase-7-launch-g.md`
- `docs/phase-8-launch-h.md`
- `docs/phase-9-launch-i.md`
- `docs/phase-10-launch-j.md`
- `docs/phase-11-launch-k.md`
- `docs/phase-12-launch-l.md`

This file is the shorter single-reference manual. Source code, not this
summary, is the final authority if a command or interface changes.

---

## 1. System purpose and architecture

LunaBot V4 is a simulated lunar rover stack using Gazebo Sim, ROS 2 Humble,
RViz2, `slam_toolbox`, a single safe velocity controller, and a sequence of
increasingly capable perception and planning nodes.

The complete data path is:

```text
Gazebo lunar world and rover
  ├─ RGB camera       /lunabot/camera/image_raw
  ├─ depth camera     /lunabot/depth/image_raw
  ├─ LiDAR            /lunabot/lidar/scan
  ├─ IMU              /lunabot/imu
  └─ DiffDrive        /lunabot/odom + odom -> chassis TF
          │
          ▼
  ROS-Gazebo bridge + static sensor TF
          │
          ├─ RGB-D terrain perception
          ├─ semantic terrain map
          ├─ terrain cost map
          ├─ terrain-aware planner
          └─ real LiDAR obstacle detector
                            │
                            ▼
  map / cost / obstacle-aware plans
          │
          ▼
  /cmd_vel_in -> one safety controller -> /cmd_vel -> Gazebo DiffDrive
```

There is intentionally only one active velocity safety boundary:

```text
planner or teleop -> /cmd_vel_in -> control_odometry.py -> /cmd_vel
```

The diagnostic A* node used by Phase D and retained later publishes a plan and
an isolated diagnostic command topic. It must not bypass the Phase B controller.
The Phase I/J terrain follower is the active motion source for the integrated
terrain-navigation demonstration.

---

## 2. Requirements and installation

### 2.1 Required software

- Ubuntu 22.04 or compatible Linux workstation
- ROS 2 Humble
- Gazebo Sim Fortress or a compatible newer Gazebo Sim release
- `ros_ign_bridge` for Fortress, or `ros_gz_bridge` on newer Gazebo releases
- `rviz2`
- `slam_toolbox`
- `nav2_map_server` for saving map evidence
- `tf2_tools`
- Python 3 with the ROS 2 Python packages used by the scripts

The launchers automatically detect `ign`/`gz` and
`ros_ign_bridge`/`ros_gz_bridge`, but the ROS message and bridge packages must
be installed first.

### 2.2 Suggested Ubuntu packages

Package names can vary slightly by ROS/Gazebo distribution. On the validated
ROS 2 Humble workstation:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-ros-ign-bridge
```

If the workstation uses Gazebo Garden or newer, install the matching
`ros_gz_bridge` package instead of, or in addition to, the Fortress bridge.

### 2.3 Source ROS and enter the repository

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
```

The source checkout can also be used directly without symlinks. The symlinks
below are convenient because the phase documentation uses `~/launch-a`, etc.

### 2.4 Install the phase entry points

```bash
cd ~/lunabot-v4
for p in a b c d e f g h i j k l; do
  ln -sfn "$PWD/launch-$p" "$HOME/launch-$p"
done

chmod +x launch-{a,b,c,d,e,f,g,h,i,j,k,l}
chmod +x scripts/launch-{a,b,c,d,e,f,g,h,i,j,k,l}.sh
chmod +x scripts/*.py tools/*.py
```

Each root entry point resolves its own real path and executes only its matching
`scripts/launch-*.sh`. The launchers are independent; `launch-f`, for example,
does not call `launch-e`.

---

## 3. Common launch variables

All launchers support the common variables below unless a phase guide states an
additional variable.

| Variable | Default | Meaning |
|---|---:|---|
| `HEADLESS` | `0` | `1` starts Gazebo server mode without Gazebo GUI and omits RViz |
| `DEMO` | `0` | `1` enables the deterministic automated drive/navigation test |
| `EVIDENCE` | `0` | `1` writes runtime topic, status, map, motion, and log evidence |
| `AUTO_GOAL` | phase-dependent | enables the deterministic development/regression goal |
| `FINAL_DEMO` | `0` | Phase L manual final-presentation mode |
| `REQUIRE_MANUAL_GOAL` | `true` in Phase L | requires a goal selected by the operator in RViz |

Typical modes:

```bash
# Interactive GUI mode: Gazebo + RViz + keyboard input
EVIDENCE=1 ~/launch-a

# Headless automated regression mode
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-a

# Manual GUI mode with evidence
EVIDENCE=1 ~/launch-d
```

`DEMO=1` is for regression and repeatability. It is not sufficient as final
presentation acceptance because the final mission must use an operator-selected
goal.

---

## 4. Repository code map

### 4.1 Root launchers

| File | Purpose |
|---|---|
| `launch-a` ... `launch-l` | Symlink-safe root wrappers |
| `scripts/launch-a.sh` ... `scripts/launch-l.sh` | Complete independent phase launch orchestration |

Each launcher performs some or all of the following:

1. Resolve repository paths.
2. Check world, model, mesh, configuration, and executable files.
3. Source ROS 2 Humble.
4. Detect Gazebo and the ROS-Gazebo bridge.
5. Clean stale processes associated with the lunar world.
6. Start Gazebo and unpause simulation.
7. Spawn the rover at the known terrain-safe height.
8. Start the bridge and static transforms.
9. Start that phase's ROS nodes.
10. Start RViz unless `HEADLESS=1`.
11. Validate real topic types, topic content, TF, status messages, motion, and
    phase-specific outputs.
12. Save evidence if `EVIDENCE=1`.
13. Stop all child processes and report clean shutdown.

### 4.2 Rover and world

| File | Purpose |
|---|---|
| `src/lunabot_gazebo/worlds/lunar_world.sdf` | Lunar gravity, terrain, habitat, equipment, rock, physical forward obstacle, horizon catch plane, Gazebo systems |
| `src/lunabot_gazebo/models/lunabot_v4/model.sdf` | Rover body, six wheels, rocker/bogie joints, mast, RGB camera, depth camera, LiDAR, IMU, DiffDrive, joint controllers |
| `src/lunabot_gazebo/worlds/meshes/lunar_terrain.obj` | High-detail terrain visual mesh |
| `src/lunabot_gazebo/worlds/meshes/lunar_terrain_collision.obj` | Lower-detail collision mesh for stable simulation |
| `config/slam_toolbox_phase_c.yaml` | Single approved `slam_toolbox` configuration |
| `rviz/phase_a.rviz` ... `rviz/phase_l.rviz` | Phase-specific RViz layouts and topic displays |

The rover is spawned around `SPAWN_Z=-2.308`. If the terrain is regenerated,
read the generated terrain statistics and update the spawn height in the
launchers before runtime validation.

### 4.3 ROS Python nodes

| File | Function | Publishes velocity? |
|---|---|---:|
| `scripts/wasd_teleop.py` | Keyboard or deterministic demo command producer | Publishes `/cmd_vel` in Phase A, `/cmd_vel_in` in later controlled phases |
| `scripts/control_odometry.py` | Clamp, acceleration-limit, watchdog, and forward safe commands | Publishes the only safe `/cmd_vel` |
| `scripts/odometry_monitor.py` | Collect real odometry samples and generate a motion report | No |
| `scripts/astar_navigation.py` | Diagnostic grid A*, goal handling, path/status, isolated command output | Diagnostic only |
| `scripts/terrain_segmentation.py` | Decode RGB-D and classify terrain/obstacle/unknown pixels | No |
| `scripts/semantic_terrain_mapper.py` | Project RGB-D segmentation into a map-frame semantic grid | No |
| `scripts/terrain_cost_mapper.py` | Convert semantic labels and sensed obstacles into inflated costs | No |
| `scripts/terrain_aware_planner.py` | Weighted terrain-aware planning from cost map to terrain path | No |
| `scripts/terrain_path_follower.py` | Follow terrain-aware path through `/cmd_vel_in` | Yes, through controller boundary |
| `scripts/dynamic_replan_monitor.py` | Detect changed terrain plan revisions and report replanning | No |
| `scripts/phase_k_evaluator.py` | Aggregate plan, motion, replan, goal, and controller evidence | No |
| `scripts/obstacle_detector.py` | Detect real close forward LiDAR returns and publish obstacle map/status/markers | No |
| `scripts/phase_l_mission.py` | Aggregate final manual goal, map, obstacle, replan, evaluation, and arrival evidence | No |

### 4.4 Utility and validator code

| File | Purpose |
|---|---|
| `tools/generate_lunar_terrain.py` | Deterministic terrain mesh generation |
| `tools/plot_lidar_scan.py` | Plot a recorded LiDAR sample for evidence |
| `tools/validate_phase_a.py` ... `tools/validate_phase_l.py` | Static source/config/documentation validators and nested phase gates |
| `evidence/phase-*-launch-*/` | Runtime logs, static reports, maps, samples, and checklists |

---

## 5. Core ROS interfaces

### 5.1 Sensor and rover topics

| Topic | Type | Source | Use |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo bridge | Simulation time |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Phase B controller | Safe command to DiffDrive |
| `/cmd_vel_in` | `geometry_msgs/msg/Twist` | Teleop/planner | Input to safety controller |
| `/lunabot/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive | Real wheel odometry |
| `/lunabot/camera/image_raw` | `sensor_msgs/msg/Image` | RGB camera | RGB-D perception and RViz |
| `/lunabot/depth/image_raw` | `sensor_msgs/msg/Image` | Depth camera | Depth perception and RViz |
| `/lunabot/lidar/scan` | `sensor_msgs/msg/LaserScan` | GPU LiDAR | SLAM and real obstacle detection |
| `/lunabot/imu` | `sensor_msgs/msg/Imu` | Gazebo IMU | IMU diagnostics |
| `/lunabot/joint_states` | `sensor_msgs/msg/JointState` | Gazebo | Rover joint state |
| `/lunabot/control/status` | `std_msgs/msg/String` | Controller | Watchdog and command boundary |
| `/lunabot/odometry/status` | `std_msgs/msg/String` | Odometry monitor | Real-motion quality |

### 5.2 Mapping and planning topics

| Topic | Type | Source | Use |
|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | `slam_toolbox` | SLAM map and diagnostic A* input |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz `SetGoal` or regression goal | Operator destination |
| `/plan` | `nav_msgs/msg/Path` | Diagnostic A* | Basic planner path |
| `/lunabot/navigation/status` | `std_msgs/msg/String` | Diagnostic A* | Planning, following, arrival status |
| `/lunabot/terrain/segmentation` | `sensor_msgs/msg/Image` | RGB-D segmentation | Semantic labels |
| `/lunabot/terrain/overlay` | `sensor_msgs/msg/Image` | RGB-D segmentation | Visual terrain overlay |
| `/lunabot/terrain/segmentation/status` | `std_msgs/msg/String` | RGB-D segmentation | Per-frame class statistics |
| `/lunabot/terrain/semantic_map` | `nav_msgs/msg/OccupancyGrid` | Semantic mapper | Terrain/obstacle semantic grid |
| `/lunabot/terrain/semantic_map/status` | `std_msgs/msg/String` | Semantic mapper | Semantic-map status |
| `/lunabot/terrain/cost_map` | `nav_msgs/msg/OccupancyGrid` | Cost mapper | Inflated traversability costs |
| `/lunabot/terrain/cost_map/status` | `std_msgs/msg/String` | Cost mapper | Cost-map status |
| `/lunabot/terrain/plan` | `nav_msgs/msg/Path` | Terrain-aware planner | Weighted path |
| `/lunabot/terrain/planner/status` | `std_msgs/msg/String` | Terrain-aware planner | Planner status |
| `/lunabot/autonomy/status` | `std_msgs/msg/String` | Terrain follower | Integrated motion and arrival |
| `/lunabot/autonomy/replan_status` | `std_msgs/msg/String` | Replan monitor | Dynamic replanning result |

### 5.3 Final mission topics

| Topic | Type | Source | Acceptance meaning |
|---|---|---|---|
| `/lunabot/obstacles/status` | `std_msgs/msg/String` | LiDAR obstacle detector | Must contain `OBSTACLE_DETECTED` |
| `/lunabot/obstacles/map` | `nav_msgs/msg/OccupancyGrid` | LiDAR obstacle detector | Real sensed obstacle cells |
| `/lunabot/obstacles/markers` | `visualization_msgs/msg/Marker` | LiDAR obstacle detector | RViz obstacle-return visualization |
| `/lunabot/evaluation/status` | `std_msgs/msg/String` | Phase K evaluator | Must contain `EVALUATION_PASS` |
| `/lunabot/mission/status` | `std_msgs/msg/String` | Phase L mission monitor | Must contain `MISSION_DEMO_PASS` |

The status publishers use Reliable and often Transient Local QoS so late
joining validators and RViz can receive the latest result. Sensor input uses
sensor-compatible Best Effort QoS where required.

---

## 6. TF tree

The base tree is:

```text
map
 └─ odom                 published by slam_toolbox in Phase C+
     └─ chassis           published dynamically by Gazebo DiffDrive
         └─ sensor_head
             ├─ rgb_camera
             ├─ depth_camera
             └─ lidar / lunabot_v4/sensor_head/lidar
```

Phase A/B use the dynamic `odom -> chassis` transform and static sensor frames.
Phase C adds the exact scoped LiDAR frame required by `slam_toolbox` and
publishes `map -> odom`. The scoped frame prevents the Gazebo sensor frame
mismatch that can otherwise stop SLAM.

Inspect TF on a live run with:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom chassis
ros2 run tf2_ros tf2_echo chassis sensor_head
ros2 run tf2_ros tf2_echo sensor_head lunabot_v4/sensor_head/lidar
ros2 run tf2_ros tf2_echo map odom
```

---

## 7. Phase-by-phase code and commands

## Phase A — Simulation and rover foundation

### Function

Phase A creates the visible lunar scene and proves that the rover and sensors
work before navigation is added.

The world includes:

- large cratered lunar terrain;
- lunar gravity;
- habitat building;
- equipment module;
- rock landmark;
- physical forward obstacle with collision geometry;
- horizon catch plane;
- rover spawn point on the terrain.

### Main code

- `scripts/launch-a.sh`
- `src/lunabot_gazebo/worlds/lunar_world.sdf`
- `src/lunabot_gazebo/models/lunabot_v4/model.sdf`
- `scripts/wasd_teleop.py`
- `rviz/phase_a.rviz`

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_a.py

# GUI presentation and WASD driving
EVIDENCE=1 ~/launch-a

# Automated headless drive test
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-a
```

### Keyboard controls

```text
W       forward
S       reverse
A       turn left
D       turn right
Space   stop
Q       quit
```

### Expected displays

Gazebo must show the lunar terrain, habitat, equipment, rock, obstacle, and
rover. RViz uses fixed frame `odom` and shows Grid, TF, LiDAR, RGB camera, and
depth camera.

### Phase A acceptance

- real `/lunabot/camera/image_raw` messages;
- real `/lunabot/depth/image_raw` messages;
- real `/lunabot/lidar/scan` messages;
- real `/lunabot/odom` and TF;
- rover moves on terrain;
- physical obstacle is a Gazebo collision object, not a synthetic test marker;
- clean shutdown.

---

## Phase B — Safe control and odometry

### Function

Phase B replaces direct planner/teleop access to the rover with a controller
boundary. It clamps commands, applies acceleration limits, stops on stale input,
and records real odometry.

### Main code

- `scripts/launch-b.sh`
- `scripts/control_odometry.py`
- `scripts/odometry_monitor.py`
- `rviz/phase_b.rviz`

### Data path

```text
teleop or planner -> /cmd_vel_in
                 -> control_odometry.py
                 -> clamp + acceleration limit + watchdog
                 -> /cmd_vel
                 -> Gazebo DiffDrive
                 -> /lunabot/odom
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_b.py

# GUI control run
EVIDENCE=1 ~/launch-b

# Headless command, watchdog, and odometry run
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-b
```

### Acceptance status strings

Look for controller status showing an active boundary and watchdog-safe state,
plus an odometry report containing real duration, rate, distance, yaw, and
continuity measurements.

---

## Phase C — SLAM and localization

### Function

Phase C adds the single approved `slam_toolbox` system. It consumes real LiDAR
and TF, publishes `/map`, and provides `map -> odom` localization.

### Main code

- `scripts/launch-c.sh`
- `config/slam_toolbox_phase_c.yaml`
- `rviz/phase_c.rviz`
- inherited `control_odometry.py` and `odometry_monitor.py`

### Data path

```text
/lunabot/lidar/scan + sensor TF + /lunabot/odom
                     -> slam_toolbox
                     -> /map + map -> odom TF
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_c.py

# Headless SLAM/map evidence gate
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-c

# GUI map and RViz run
EVIDENCE=1 ~/launch-c
```

### Acceptance

Require a non-empty live map, correct scoped LiDAR TF, map updates while the
rover moves, saved YAML/PGM map evidence, and a clean relaunch.

---

## Phase D — Basic A* navigation and operator goal selection

### Function

Phase D adds diagnostic A* planning over the SLAM occupancy grid and a basic
path follower. The GUI default is operator-selected goal mode. The deterministic
automatic goal is retained only for `DEMO=1` regression.

### Main code

- `scripts/launch-d.sh`
- `scripts/astar_navigation.py`
- `rviz/phase_d.rviz`

### Data path

```text
/map + /lunabot/odom + map -> odom TF
              -> astar_navigation.py
              -> /plan + /lunabot/navigation/status
              -> /cmd_vel_in
              -> Phase B controller
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_d.py

# Regression goal and headless navigation
HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-d

# Operator-selected GUI goal
AUTO_GOAL=false EVIDENCE=1 ~/launch-d
```

### RViz goal selection

Select the `Set Goal` tool, click the destination in the map, and drag to set
heading. The selected goal is displayed on `/goal_pose` and as the `Selected
Goal` pose display. A goal can also be published from another terminal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

### Acceptance

Require a non-empty `/plan`, real motion, goal arrival, map updates, controller
boundary evidence, and clean shutdown/relaunch. Do not use auto-goal output as
proof of final operator-goal acceptance.

---

## Phase E — RGB-D terrain perception

### Function

Phase E reads real RGB and depth images and publishes terrain, obstacle, and
unknown image classifications. It does not publish velocity or replace A*.

### Main code

- `scripts/launch-e.sh`
- `scripts/terrain_segmentation.py`
- `rviz/phase_e.rviz`

### Data path

```text
RGB + depth -> terrain_segmentation.py
            ├─ /lunabot/terrain/segmentation
            ├─ /lunabot/terrain/overlay
            └─ /lunabot/terrain/segmentation/status
```

Labels are:

```text
0  unknown
1  terrain
2  obstacle
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_e.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-e
EVIDENCE=1 ~/launch-e
```

### Acceptance status

The status must contain `SEGMENTATION_PASS` with image size, valid-depth, and
class statistics. A running process without real topic content is not a pass.

---

## Phase F — Semantic terrain mapping

### Function

Phase F projects the Phase E RGB-D labels through the existing `map -> chassis`
TF chain into a persistent map-frame semantic occupancy grid.

### Main code

- `scripts/launch-f.sh`
- `scripts/semantic_terrain_mapper.py`
- `rviz/phase_f.rviz`

### Data path

```text
segmentation + depth + map -> chassis TF
                  -> semantic_terrain_mapper.py
                  -> /lunabot/terrain/semantic_map
                  -> /lunabot/terrain/semantic_map/status
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_f.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-f
EVIDENCE=1 ~/launch-f
```

### Acceptance status

Require `SEMANTIC_MAP_PASS`, real semantic-map cells, correct map geometry, and
clean relaunch. This node must not publish velocity.

---

## Phase G — Terrain cost map

### Function

Phase G converts semantic terrain classes into numeric traversability costs and
inflates obstacle cells. It consumes the semantic map and, in the final stack,
can fuse the sensed LiDAR obstacle overlay.

### Main code

- `scripts/launch-g.sh`
- `scripts/terrain_cost_mapper.py`
- `rviz/phase_g.rviz`

### Cost values

```text
20   normal terrain candidate
80   unknown/caution
100  obstacle
inflation halo  -> high cost around obstacle cells
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_g.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-g
EVIDENCE=1 ~/launch-g
```

### Acceptance status

Require `COST_MAP_PASS`, real cost-map messages, obstacle/inflation statistics,
and clean relaunch.

---

## Phase H — Terrain-aware path planning

### Function

Phase H adds weighted planning over the terrain cost map. It publishes a
terrain-aware path and planner status but does not yet connect that path to the
active rover motion controller.

### Main code

- `scripts/launch-h.sh`
- `scripts/terrain_aware_planner.py`
- `rviz/phase_h.rviz`

### Data path

```text
cost map + /goal_pose + map -> chassis TF
                 -> terrain_aware_planner.py
                 -> /lunabot/terrain/plan
                 -> /lunabot/terrain/planner/status
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_h.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-h
EVIDENCE=1 ~/launch-h
```

### Acceptance status

Require `TERRAIN_PLAN_PASS`, a non-empty terrain path, map/cost integration,
and clean relaunch. The planner must not publish a second velocity stream.

---

## Phase I — Full terrain-navigation integration

### Function

Phase I connects the terrain-aware plan to `terrain_path_follower.py`, which
publishes `/cmd_vel_in` and therefore remains behind the Phase B controller.

### Main code

- `scripts/launch-i.sh`
- `scripts/terrain_path_follower.py`
- `rviz/phase_i.rviz`
- inherited terrain segmentation, semantic mapping, cost mapping, and planner

### Data path

```text
RGB-D -> semantic map -> cost map -> terrain planner
                                      -> /lunabot/terrain/plan
                                      -> terrain_path_follower.py
                                      -> /cmd_vel_in
                                      -> control_odometry.py
                                      -> /cmd_vel
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_i.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-i
EVIDENCE=1 ~/launch-i
```

### Acceptance status

Require integrated goal arrival, real odometry motion, terrain planner output,
controller boundary evidence, map evidence, and clean relaunch.

---

## Phase J — Dynamic replanning

### Function

Phase J detects changed terrain-plan revisions while the rover is moving and
requires the active follower to continue through a newly generated plan.

### Main code

- `scripts/launch-j.sh`
- `scripts/dynamic_replan_monitor.py`
- `scripts/terrain_path_follower.py`
- `rviz/phase_j.rviz`

### Data path

```text
changed /lunabot/terrain/plan revisions
              -> dynamic_replan_monitor.py
              -> /lunabot/autonomy/replan_status
              -> terrain_path_follower.py
              -> /cmd_vel_in -> controller -> /cmd_vel
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_j.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-j
EVIDENCE=1 ~/launch-j
```

### Acceptance status

Require at least two changed terrain-plan revisions after real odometry motion,
`DYNAMIC_REPLAN_PASS`, final goal arrival, saved map evidence, and clean
relaunch. A static or synthetic plan change is not sufficient.

---

## Phase K — Runtime evaluation

### Function

Phase K evaluates the integrated system rather than adding a new navigation
algorithm. It combines plan, replan, controller, motion, odometry, and goal
status into an auditable result.

### Main code

- `scripts/launch-k.sh`
- `scripts/phase_k_evaluator.py`
- `rviz/phase_k.rviz`

### Evaluation inputs

```text
terrain plan
replan status
integrated autonomy status
controller status
odometry
/cmd_vel_in
/cmd_vel
```

### Commands

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_k.py

HEADLESS=1 DEMO=1 EVIDENCE=1 ~/launch-k
EVIDENCE=1 ~/launch-k
```

### Acceptance status

Require `EVALUATION_PASS`, real motion, integrated goal arrival, dynamic
replanning, controller activity, saved map evidence, and clean relaunch.

---

## Phase L — Final mission demonstration

### Function

Phase L is the final presentation stack. It preserves the approved A-K
architecture and adds:

- real LiDAR obstacle detection;
- sensed obstacle occupancy overlay;
- obstacle marker visualization;
- obstacle-aware cost-map fusion;
- final mission status aggregation;
- mandatory manual-goal mode for presentation.

### Main code

- `scripts/launch-l.sh`
- `scripts/obstacle_detector.py`
- `scripts/phase_l_mission.py`
- `rviz/phase_l.rviz`

### Real obstacle path

```text
Gazebo physical obstacle
       -> real LiDAR returns
       -> obstacle_detector.py
       ├─ /lunabot/obstacles/status
       ├─ /lunabot/obstacles/map
       └─ /lunabot/obstacles/markers
              -> terrain_cost_mapper.py
              -> terrain-aware planner
              -> dynamic replan monitor
              -> terrain follower
              -> Phase B controller
```

The obstacle detector only reports `OBSTACLE_DETECTED` from actual LiDAR scan
ranges. It does not fabricate pass/fail evidence and does not publish velocity.

### Manual final GUI mode

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
python3 tools/validate_phase_l.py
FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true \
  EVIDENCE=1 ~/launch-l
```

In RViz:

1. Select the `Set Goal` tool.
2. Click a reachable destination on the map.
3. Drag to select the final heading.
4. Confirm the selected goal marker, terrain path, obstacle map, cost map, and
   replanned active path.
5. Wait for `OBSTACLE_DETECTED` and dynamic replanning.
6. Wait for integrated goal arrival.
7. Stop the launcher with Ctrl+C and verify clean shutdown.

### Automated regression mode

This mode is useful for repeatable infrastructure testing but is not final
operator acceptance:

```bash
cd ~/lunabot-v4
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  HEADLESS=1 EVIDENCE=1 ~/launch-l
```

### Final mission acceptance

The final mission status must contain:

```text
MISSION_DEMO_PASS plan=1 replan=1 goal=1 evaluation=1 \
  goal_pose=1 map=1 obstacle=1 manual_goal=1
```

The final acceptance requires two independent GUI demonstrations with a clean
relaunch between them. Automatic-goal mode must not be used as the only proof.

---

## 8. Static validation commands

Run validators individually:

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_a.py
python3 tools/validate_phase_b.py
python3 tools/validate_phase_c.py
python3 tools/validate_phase_d.py
python3 tools/validate_phase_e.py
python3 tools/validate_phase_f.py
python3 tools/validate_phase_g.py
python3 tools/validate_phase_h.py
python3 tools/validate_phase_i.py
python3 tools/validate_phase_j.py
python3 tools/validate_phase_k.py
python3 tools/validate_phase_l.py
```

Run source syntax and compilation checks:

```bash
cd ~/lunabot-v4
bash -n launch-a scripts/launch-a.sh
bash -n launch-b scripts/launch-b.sh
bash -n launch-c scripts/launch-c.sh
bash -n launch-d scripts/launch-d.sh
bash -n launch-e scripts/launch-e.sh
bash -n launch-f scripts/launch-f.sh
bash -n launch-g scripts/launch-g.sh
bash -n launch-h scripts/launch-h.sh
bash -n launch-i scripts/launch-i.sh
bash -n launch-j scripts/launch-j.sh
bash -n launch-k scripts/launch-k.sh
bash -n launch-l scripts/launch-l.sh

python3 -m py_compile \
  scripts/wasd_teleop.py \
  scripts/control_odometry.py \
  scripts/odometry_monitor.py \
  scripts/astar_navigation.py \
  scripts/terrain_segmentation.py \
  scripts/semantic_terrain_mapper.py \
  scripts/terrain_cost_mapper.py \
  scripts/terrain_aware_planner.py \
  scripts/terrain_path_follower.py \
  scripts/dynamic_replan_monitor.py \
  scripts/phase_k_evaluator.py \
  scripts/obstacle_detector.py \
  scripts/phase_l_mission.py \
  tools/generate_lunar_terrain.py \
  tools/plot_lidar_scan.py \
  tools/validate_phase_a.py \
  tools/validate_phase_b.py \
  tools/validate_phase_c.py \
  tools/validate_phase_d.py \
  tools/validate_phase_e.py \
  tools/validate_phase_f.py \
  tools/validate_phase_g.py \
  tools/validate_phase_h.py \
  tools/validate_phase_i.py \
  tools/validate_phase_j.py \
  tools/validate_phase_k.py \
  tools/validate_phase_l.py
```

A validator is static evidence only. It proves source/configuration contracts;
it does not claim that Gazebo published a live message or that the rover
reached a real goal.

---

## 9. Runtime inspection commands

Use these only while a launcher is running.

### Topic discovery and type checks

```bash
ros2 topic list
ros2 topic type /lunabot/odom
ros2 topic type /lunabot/lidar/scan
ros2 topic type /lunabot/camera/image_raw
ros2 topic type /lunabot/depth/image_raw
ros2 topic type /map
ros2 topic type /goal_pose
ros2 topic type /lunabot/terrain/plan
ros2 topic type /lunabot/obstacles/status
ros2 topic type /lunabot/mission/status
```

### Topic content checks

```bash
ros2 topic echo /lunabot/odom --once
ros2 topic echo /lunabot/lidar/scan --once
ros2 topic echo /lunabot/camera/image_raw --once
ros2 topic echo /lunabot/depth/image_raw --once
ros2 topic echo /map --once
ros2 topic echo /plan --once
ros2 topic echo /lunabot/terrain/plan --once
ros2 topic echo /lunabot/obstacles/status --once
ros2 topic echo /lunabot/autonomy/replan_status --once
ros2 topic echo /lunabot/mission/status --once
```

For retained status topics, use reliable/transient-local QoS when necessary:

```bash
ros2 topic echo /lunabot/mission/status --once \
  --qos-reliability reliable --qos-durability transient_local
```

### Rates and publishers

```bash
ros2 topic hz /lunabot/odom
ros2 topic hz /lunabot/lidar/scan
ros2 topic hz /lunabot/camera/image_raw
ros2 topic hz /lunabot/depth/image_raw
ros2 topic info -v /cmd_vel_in
ros2 topic info -v /cmd_vel
ros2 topic info -v /lunabot/obstacles/status
```

### Manual goal publication

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

Use the RViz `SetGoal` tool for final evidence instead of this command; the
command is useful for debugging.

---

## 10. Evidence locations

Each phase writes to its own directory:

```text
evidence/phase-a-launch-a/
evidence/phase-b-launch-b/
evidence/phase-c-launch-c/
evidence/phase-d-launch-d/
evidence/phase-e-launch-e/
evidence/phase-f-launch-f/
evidence/phase-g-launch-g/
evidence/phase-h-launch-h/
evidence/phase-i-launch-i/
evidence/phase-j-launch-j/
evidence/phase-k-launch-k/
evidence/phase-l-launch-l/
```

Typical files include:

| File | Meaning |
|---|---|
| `last_run.log` | Launcher output and shutdown record |
| `gazebo.log` / `bridge.log` | Gazebo and bridge logs |
| `topics.txt` | Topic discovery snapshot |
| `topic_info.txt` | Topic type/QoS information |
| `tf_*.txt` | TF evidence |
| `odom_sample.txt` / `odometry_report.txt` | Real odometry evidence |
| `map_before_navigation.txt` / `phase_l_map.yaml` / `phase_l_map.pgm` | Map evidence |
| `plan.txt` / `terrain_plan.txt` | Planner path samples |
| `obstacle_status.txt` / `obstacle_map.txt` | Real obstacle detector evidence |
| `replan_status.txt` | Dynamic replanning evidence |
| `evaluation_status.txt` | Phase K evaluation result |
| `mission_status.txt` | Final Phase L mission result |
| `static_validation.txt` | Generated static validator report |

Do not edit runtime evidence to make a test pass. Evidence must be generated by
live ROS/Gazebo execution.

---

## 11. Clean shutdown and relaunch procedure

For every phase:

1. Let the launcher finish its validation or press Ctrl+C.
2. Wait for `Launch X environment cleanly closed.`
3. Confirm no stale processes remain.
4. Relaunch the same phase.
5. For final acceptance, perform two GUI runs and retain both result sets.

Useful diagnostics after a failed run:

```bash
pgrep -af 'lunar_world|gz sim|ign gazebo|rviz2|ros2 run|python3.*lunabot'
ros2 node list
ros2 topic list
```

Only terminate stale processes after confirming they belong to this checkout.
The launchers already perform scoped cleanup for the lunar world and their own
child processes.

---

## 12. Terrain regeneration

Do not regenerate terrain as part of an ordinary launch. Only do it when the
mesh must be intentionally changed:

```bash
cd ~/lunabot-v4
python3 tools/generate_lunar_terrain.py
```

After regeneration:

1. Read `evidence/phase-a-launch-a/terrain_stats.txt`.
2. Confirm the rover spawn height in every relevant launcher remains valid.
3. Run `python3 tools/validate_phase_a.py`.
4. Repeat the Phase A GUI and headless checks.
5. Revalidate all later phases because map/planning behavior depends on the
   terrain geometry.

---

## 13. Final acceptance checklist

### Static

- [ ] All Phase A-L validators pass.
- [ ] All launchers pass `bash -n`.
- [ ] All Python files compile.
- [ ] No launcher invokes an earlier launcher.
- [ ] One controller remains the only safe `/cmd_vel` boundary.
- [ ] One `slam_toolbox` system is used.

### Gazebo and RViz

- [ ] Gazebo shows lunar terrain, habitat, equipment, rock, obstacle, and rover.
- [ ] Rover remains on the terrain.
- [ ] RGB and depth camera panels show live data.
- [ ] LiDAR and TF follow rover motion.
- [ ] RViz fixed frame and sensor QoS are correct.

### Final mission

- [ ] Operator selects the goal with RViz `SetGoal`.
- [ ] Real LiDAR detects the physical forward obstacle.
- [ ] Obstacle map and cost map update.
- [ ] Terrain-aware path changes after sensed obstacle information.
- [ ] Dynamic replanning status passes.
- [ ] Rover reaches the selected goal.
- [ ] Mission status reports `MISSION_DEMO_PASS` with `manual_goal=1`.
- [ ] Two independent GUI demonstrations pass.
- [ ] Both runs shut down and relaunch cleanly.

---

## 14. Important distinction: regression versus presentation

The following command is useful for repeatable infrastructure tests:

```bash
DEMO=1 AUTO_GOAL=true REQUIRE_MANUAL_GOAL=false \
  HEADLESS=1 EVIDENCE=1 ~/launch-l
```

It is not the final presentation proof. Final acceptance must use:

```bash
FINAL_DEMO=1 AUTO_GOAL=false REQUIRE_MANUAL_GOAL=true \
  EVIDENCE=1 ~/launch-l
```

The difference is intentional: the first command supplies a deterministic goal;
the second requires a real operator-selected goal and displays the complete
Gazebo/RViz mission behavior.
