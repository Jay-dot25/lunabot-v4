# Phase 2 — ROS 2 package foundation

## Scope

Phase 2 makes the repository a discoverable ROS 2 workspace without replacing
the validated standalone A–L scripts. Algorithm migration occurs only in later
phases.

Packages:

| Package | Build type | Responsibility |
|---|---|---|
| `lunabot_common` | `ament_python` | Shared terrain configuration/contracts |
| `lunabot_msgs` | `ament_cmake` | Message package foundation (definitions in Phase 3) |
| `lunabot_gazebo` | `ament_cmake` | Existing world/model/mesh installation |
| `lunabot_perception` | `ament_python` | ML perception foundation |
| `lunabot_mapping` | `ament_python` | Semantic/traversability mapping foundation |
| `lunabot_planning` | `ament_python` | A* and D*-Lite foundation |
| `lunabot_control` | `ament_python` | Following and safety foundation |
| `lunabot_evaluation` | `ament_python` | Metrics/experiments foundation |
| `lunabot_bringup` | `ament_python` | Launch and shared parameter foundation |

The top-level `lunabot_common/` module is a source-checkout compatibility facade.
The canonical installable implementation is in
`src/lunabot_common/lunabot_common/`.

## Static verification (works without ROS)

```bash
python3 tools/validate_ros_packages.py
python3 -m unittest tests.test_ros_package_foundation
python3 tools/validate_all.py --no-phase-validators --no-write
```

## ROS workstation build

```bash
cd ~/lunabot-v4
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep '^lunabot_'
ros2 launch lunabot_bringup foundation.launch.py
```

Expected launch message:

```text
LunaBot ROS 2 package foundation is discoverable.
```

The foundation launch intentionally starts no robot process. Existing
`./launch-a` through `./launch-l` remain the behavioral entry points until
later phases migrate and runtime-test each subsystem.
