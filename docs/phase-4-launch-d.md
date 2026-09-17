# Phase D — Basic Autonomous Navigation (`launch-d`)

## 1. Scope and gate

Phase D adds one bounded A* grid planner and waypoint follower to the
runtime-approved Phase C stack. It consumes the live `slam_toolbox` map and
publishes planner commands through the already validated Phase B control
boundary. It does not add Nav2, AMCL, Cartographer, a second SLAM system, or a
second velocity controller.

Phase D is independently launchable. It starts the validated lunar world,
rover, bridge, scoped sensor TF, control node, odometry monitor,
`slam_toolbox`, A* navigation, and RViz directly. It does not invoke Phase C
or any earlier launch command.

The runtime gate was completed on the workstation. A real run confirmed a
non-empty A* path, autonomous motion to a goal, live map updates, all inherited
Phase B/C sensor checks, map evidence, and clean shutdown/relaunch. Phase D was
explicitly approved before Phase E began.

## 2. Data flow

```text
/lunabot/lidar/scan + TF + /lunabot/odom
                         │
                         ▼
                    slam_toolbox
                         │
              /map + map -> odom TF
                         │
                         ▼
                 lunabot_astar_navigation
                  ├─ /goal_pose
                  ├─ A* grid search with inflation
                  ├─ /plan (nav_msgs/Path)
                  └─ /cmd_vel_in
                                  │
                                  ▼
                 Phase B control/watchdog boundary
                                  │
                           /cmd_vel -> DiffDrive
```

The planner transforms the real `/lunabot/odom` pose through `map -> odom`,
plans in the `nav_msgs/OccupancyGrid` frame, and follows the resulting path.
The map subscription requests transient-local QoS so a newly started planner
receives the current SLAM map instead of depending on a future publication.
Planner path, goal, and status outputs are also transient-local for reliable
late-joining evidence and RViz subscribers.

## 3. Planner contract

| Interface | Type | Role |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | live SLAM planning grid |
| `/goal_pose` | `geometry_msgs/PoseStamped` | navigation goal in `map` or transformable frame |
| `/lunabot/odom` | `nav_msgs/Odometry` | real motion pose source |
| `/plan` | `nav_msgs/Path` | non-empty A* result |
| `/cmd_vel_in` | `geometry_msgs/Twist` | planner output to inherited controller |
| `/lunabot/navigation/status` | `std_msgs/String` | waiting/planning/following/goal status |

The GUI launch waits for an operator-selected goal; RViz provides a `Set
Goal` tool and a selected-goal arrow, so the operator can click a real
destination in the map before the rover moves. For regression only, `DEMO=1`
enables the deterministic automatic goal 1.5 m ahead of the current map pose;
a manually published `/goal_pose` replaces it. Occupied cells at or
above 65 and unknown cells are blocked by default; a 0.25 m inflation radius
keeps the rover away from mapped obstacles. Eight-connected A* uses the map
resolution and map origin orientation rather than assuming an axis-aligned
world.

The follower uses a conservative 0.25 m/s maximum linear speed and 0.7 rad/s
maximum angular speed. It publishes only to `/cmd_vel_in`; the Phase B node
continues to clamp, accelerate-limit, watchdog, and publish the safe `/cmd_vel`.

## 4. Ubuntu installation

Phase D uses the Phase C packages and the Python ROS messages already present
in the validated installation. If starting from the Phase C workstation:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-ros-ign-bridge
source /opt/ros/humble/setup.bash
```

Install the home command from the checkout:

```bash
cd ~/lunabot-v4
ln -sfn "$PWD/launch-d" ~/launch-d
chmod +x launch-d scripts/launch-d.sh scripts/astar_navigation.py
```

## 5. Static validation

Run the Phase D validator before launching Gazebo:

```bash
cd ~/lunabot-v4
python3 tools/validate_phase_d.py
```

It checks the Phase A/B/C gates, independent launch behavior, exact scoped
LaserScan TF, explicit sensor topics and QoS, headless unpause, real message
validation, single `slam_toolbox`, A* implementation, `/map` transient-local
handling, path/goal/status interfaces, controller isolation, motion evidence,
map saving, clean shutdown, and documentation.

Static validation never claims that A* reached a real goal. That requires the
runtime command below.

## 6. Automated headless navigation run

Run the Phase D autonomous demo:

```bash
cd ~/lunabot-v4
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d
```

The launcher explicitly unpauses Gazebo, starts the one SLAM system, starts A*
with an automatic goal, and waits for `GOAL_REACHED`. It validates these real
runtime interfaces:

```text
/map                         nav_msgs/OccupancyGrid
map -> odom                  slam_toolbox TF
/lunabot/odom               DiffDrive odometry
odom -> chassis              dynamic TF
sensor_head -> scoped lidar  Gazebo LaserScan TF
/goal_pose                   PoseStamped
/plan                        non-empty nav_msgs/Path
/lunabot/navigation/status   planner status
/cmd_vel_in                  A* command input
/lunabot/imu                 bridged Imu
```

The run must report:

```text
A* goal reached: PASS
live map updates during autonomous navigation: PASS
saved map evidence (YAML + PGM): PASS
PHASE D RUN COMPLETE - overall result: PASS
```

The path follower moves only through `/cmd_vel_in`; it never bypasses the
Phase B controller. If planning fails, inspect `navigation.log` and the saved
map/TF evidence instead of treating the planner process being alive as a pass.

## 7. GUI/RViz run

After the headless run passes:

```bash
source /opt/ros/humble/setup.bash
cd ~/lunabot-v4
EVIDENCE=1 ~/launch-d
```

Phase D RViz uses fixed frame `map` and displays:

- `SLAM Map` on `/map`;
- `A* Path` on `/plan`;
- `LaserScan` on `/lunabot/lidar/scan` with Best Effort QoS;
- `Camera` on `/lunabot/camera/image_raw` with Best Effort QoS;
- `TF` including `map -> odom -> chassis` and the scoped LaserScan frame;
- `Odometry` on `/lunabot/odom`.

The GUI launch waits for the operator to click the RViz `Set Goal` tool; the
regression command above enables the automatic goal. A replacement goal can
also be sent from a separate sourced terminal, for example:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

Confirm the green A* path is visible, the rover follows it, the Camera and
LaserScan remain error-free, and RViz shows the live map. Press Ctrl+C in the
launcher terminal for clean shutdown.

## 8. Evidence files

Runtime evidence is written to `evidence/phase-d-launch-d/`:

| File | Meaning |
|---|---|
| `navigation.log` | A* node log and planner diagnostics |
| `navigation_status.txt` | real planner status sample |
| `goal_pose.txt` | real automatic or manual goal |
| `plan.txt` | real `nav_msgs/Path` sample |
| `map_before_navigation.txt` / `map_after_navigation.txt` | live-map motion gate |
| `phase_d_map.yaml` / `phase_d_map.pgm` | final map evidence |
| `tf_map_odom.txt` | real SLAM localization TF |
| `tf_lidar_scoped.txt` | exact Gazebo LaserScan frame TF |
| `odometry_samples.csv` / `odometry_report.txt` | inherited real odometry evidence |
| `map_saver.log` | final map saver result |
| `last_run.log` | ordered launcher result |

Do not hand-edit runtime evidence. The map saver is checked when
`nav2_map_server` is installed; its absence is reported explicitly rather
than converted into a false success.

## 9. Clean relaunch

After the autonomous goal run or GUI run ends, verify:

```text
Launch D environment cleanly closed.
```

Then run the autonomous headless command a second time:

```bash
EVIDENCE=1 DEMO=1 HEADLESS=1 ~/launch-d
```

Both runs must clean stale Phase D processes, unpause Gazebo, create one rover,
start one `slam_toolbox` and one A* node, reach a goal, save the map, and close
all process groups without orphaned nodes.

## 10. Acceptance checklist

- [ ] Phase A/B/C runtime interfaces remain passing.
- [ ] `/map` is a real `nav_msgs/OccupancyGrid`.
- [ ] `map -> odom` and scoped LaserScan TF remain valid.
- [ ] A* produces a non-empty `/plan`.
- [ ] `/goal_pose` and planner status are real messages.
- [ ] `/cmd_vel_in` is the only planner command output.
- [ ] The inherited controller remains between A* and `/cmd_vel`.
- [ ] The rover reaches the automatic goal using real odometry.
- [ ] The map changes during autonomous navigation.
- [ ] IMU, Camera, LaserScan, and RViz QoS/topic fixes remain error-free.
- [ ] Final map YAML/PGM evidence is saved.
- [ ] Clean shutdown and a second independent relaunch pass.

## 11. Repository files

| File | Role |
|---|---|
| `launch-d` | canonical `~/launch-d` entry point |
| `scripts/launch-d.sh` | independent Phase D startup, validation, evidence, shutdown |
| `scripts/astar_navigation.py` | A* planner and conservative follower |
| `rviz/phase_d.rviz` | map/path/sensor visualization |
| `tools/validate_phase_d.py` | static Phase D gate |
| `evidence/phase-d-launch-d/` | runtime evidence and checklist |

============================================================
PHASE D IMPLEMENTATION — PENDING RUNTIME VALIDATION
============================================================

Phase D workstation runtime, map evidence, clean shutdown, and clean relaunch
were explicitly approved before Phase E implementation began. Phase E must
complete its own runtime gate before Phase F.
