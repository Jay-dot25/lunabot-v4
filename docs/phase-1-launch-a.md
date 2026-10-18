# Phase 1 / Launch 1 (legacy name: Launch A)

The original Launch A entry point was a hand-written script tied to `$HOME/lunabot-v4`, ROS 2 Humble, Ignition-era commands, and interfaces that did not match the checked-in SDF paths. It was replaced with a package-based ROS 2/Gazebo Sim launch and a relocatable wrapper.

Use from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source install/setup.bash
./scripts/launch-1.sh
```

The older `./scripts/launch-a.sh` remains as a compatibility alias to Launch 1. Launch details, actual interfaces, manual control, validation, supported environment assumptions, and current acceptance blockers are maintained in the root [README](../README.md) and [Phase 1 Acceptance Report](../PHASE1_ACCEPTANCE.md).

The chosen implementation targets ROS 2 Jazzy + Gazebo Harmonic and is not runtime-certified in the current Debian 12 sandbox, which has no ROS 2 or Gazebo tools installed. Do not interpret this document as a claim that the simulator was executed here.
