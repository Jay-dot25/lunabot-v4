# Phase 1 / Launch 1 (legacy name: Launch A)

The original Launch A entry point was a hand-written script tied to `$HOME/lunabot-v4`, ROS 2 Humble, Ignition-era commands, and interfaces that did not match the checked-in SDF paths. It was replaced with a package-based ROS 2/Gazebo Sim launch and a relocatable wrapper.

Use from the repository root:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ament_python
rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys=ament_python -y
colcon build --symlink-install
source install/setup.bash
./scripts/launch-1.sh
```

The older `./scripts/launch-a.sh` remains as a compatibility alias to Launch 1. Launch details, actual interfaces, manual control, validation, supported environment assumptions, and current acceptance blockers are maintained in the root [README](../README.md) and [Phase 1 Acceptance Report](../PHASE1_ACCEPTANCE.md).

The implementation now targets the developer-reported ROS 2 Humble + Ignition Fortress stack (`ign gazebo`). The first launch used the wrong `gz sim` CLI and failed; the updated Fortress-specific launch and plugin configuration have not yet been runtime-tested. The agent sandbox itself has no ROS 2/Gazebo tools.
