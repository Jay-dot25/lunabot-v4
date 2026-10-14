# Launch 1 — Clean Phase 1 Simulation and Rover Foundation

## Purpose and separation

`launch-1` is a single-command entry point for the standalone Phase 1 workspace
at `lunabot_phase1/`. It starts the Humble + Gazebo Fortress world, rover,
bridges, static transforms, command guard, and diagnostics-only obstacle
monitor using the existing `phase1.launch.py`. The underlying Phase 1 runtime
was reported passing via direct `ros2 launch`; this repository-level wrapper is
new and still needs one target-workstation launch plus smoke check after pull.

This launch is intentionally separate from the repository's established
`launch-a` through `launch-l` sequence. It does not replace, call, or modify
those launchers. It also does not add SLAM, mapping, machine learning,
autonomous planning, obstacle avoidance, or mission control.

## Requirements

- Ubuntu 22.04 and ROS 2 Humble.
- Gazebo Fortress (`ign gazebo`) and `ros_ign_bridge`.
- The four packages in `lunabot_phase1/` built once with `colcon`.
- A graphics-capable session for Gazebo's GUI and GPU camera/LiDAR sensors.

Build instructions and dependency details are in
[`../lunabot_phase1/README.md`](../lunabot_phase1/README.md).
`launch-1` does not install packages or rebuild automatically; it checks that
the workspace is built and the required ROS/Gazebo tools are available.

## Install the optional command link

From the repository root:

```bash
ln -sfn "$(pwd)/launch-1" "$HOME/launch-1"
```

The link is optional for launching: `./launch-1` works from the repository root. The later teleop and smoke-test examples use `~/launch-1` to locate the checkout; if you skip the link, open those terminals from the repository root and set `REPO_DIR="$PWD"` instead.

## Launch and stop

```bash
~/launch-1
```

Or run `./launch-1` from the repository root. The launcher sources ROS 2
Humble and this checkout's `lunabot_phase1/install` overlay, validates the
bringup package and Fortress dependencies, then runs:

```bash
ros2 launch lunabot_phase1_bringup phase1.launch.py
```

Press `Ctrl+C` in the launch terminal to shut down the integrated launch.
Manual teleoperation uses a separate interactive terminal. If you created the optional command link above, locate the workspace from it:

```bash
REPO_DIR="$(dirname "$(readlink -f "$HOME/launch-1")")"
cd "$REPO_DIR/lunabot_phase1"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lunabot_phase1_tools keyboard_teleop --ros-args -p linear_speed_mps:=0.05
```

Controls: `W` forward, `S` reverse, `A`/`D` rotate, `X` or Space stop, and `Q`
stop-and-exit. Commands persist until changed; use the explicit stop after each
short movement.

## Runtime checks

With the `launch-1` simulation still running, use a separate sourced terminal for the smoke check (the example below assumes the optional `~/launch-1` link exists):

```bash
REPO_DIR="$(dirname "$(readlink -f "$HOME/launch-1")")"
cd "$REPO_DIR/lunabot_phase1"
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/phase1_smoke_test.sh
```

See [`../lunabot_phase1/PHASE1_ACCEPTANCE.md`](../lunabot_phase1/PHASE1_ACCEPTANCE.md)
for the P1-01 through P1-17 acceptance matrix and the recorded Humble/Fortress
runtime result. The current matrix records all mandatory Phase 1 runtime criteria as passed
based on the user's report; the agent host cannot independently run ROS or
Gazebo, and the latest raw terminal outputs were not provided here. The new
`launch-1` wrapper itself has only passed local shell/help/preflight checks so
far; use it once on the target and rerun the smoke test to verify the wrapper.

## Known boundaries

- The obstacle monitor reports range/status only. It does not command or stop
the rover.
- `/odom` is wheel-model odometry; Gazebo world pose is a separate simulation
ground-truth stream.
- The world has a flat central lane, simple obstacles, and shallow primitive
mounds; it is not a detailed lunar heightmap.
- This Phase 1 launcher is for manual simulation and sensor verification only.
