#!/usr/bin/env bash
# ============================================================
# LunaBot - Phase 1 - Lunar Base Camp simulation
#
# Starts Gazebo with the lunar_base_camp world (rover included),
# the ROS 2 <-> Gazebo bridge, sensor static transforms and the
# WASD teleop.
#
# Usage:
#   scripts/run_phase1_sim.sh [--headless] [--no-teleop] [--ros-distro DISTRO]
#
# --headless    run Gazebo without the GUI (server only)
# --no-teleop   do not start the keyboard teleop (use for acceptance runs)
# ============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

ROS_DISTRO="${ROS_DISTRO:-humble}"
HEADLESS=0
START_TELEOP=1

while [ $# -gt 0 ]; do
    case "$1" in
        --headless)   HEADLESS=1 ;;
        --no-teleop)  START_TELEOP=0 ;;
        --ros-distro) ROS_DISTRO="$2"; shift ;;
        *) echo "Unknown option: $1"; exit 2 ;;
    esac
    shift
done

# Gazebo message package and CLI differ across Gazebo generations:
#   Fortress (ign):  ignition.msgs / "ign gazebo"
#   Garden+ (gz):    gz.msgs       / "gz sim"
GZ_MSGS="${LUNABOT_GZ_MSGS:-ignition.msgs}"
GZ_CMD="${LUNABOT_GZ_CMD:-ign gazebo}"

WORLD_PATH="$REPO_DIR/src/lunabot_gazebo/worlds/lunar_base_camp.sdf"
MODELS_PATH="$REPO_DIR/src/lunabot_description/models"
TELEOP_PATH="$REPO_DIR/src/lunabot_teleop/lunabot_teleop/wasd_teleop.py"

echo "============================================================"
echo " LunaBot - Phase 1 - Lunar Base Camp"
echo "============================================================"

# ------------------------------------------------------------ files
for f in "$WORLD_PATH" "$MODELS_PATH/lunabot_v4/model.sdf"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: required file missing: $f"
        exit 1
    fi
done
echo "[1/6] Project files present."

# ------------------------------------------------------------ ROS 2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    echo "[2/6] ROS 2 ($ROS_DISTRO) loaded."
else
    echo "ERROR: /opt/ros/$ROS_DISTRO/setup.bash not found."
    echo "       Install ROS 2 or pass --ros-distro <distro>."
    exit 1
fi

if [ -f "$REPO_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO_DIR/install/setup.bash"
    echo "       Workspace overlay loaded."
fi

# ------------------------------------------------- Gazebo resource path
RESOURCE_PATH="$MODELS_PATH"
for var in IGN_GAZEBO_RESOURCE_PATH GZ_SIM_RESOURCE_PATH; do
    if [ -n "${!var:-}" ]; then
        RESOURCE_PATH="$RESOURCE_PATH:${!var}"
    fi
done
export IGN_GAZEBO_RESOURCE_PATH="$RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH"
echo "[3/6] Gazebo resource path set."

# -------------------------------------------------------- process bookkeeping
PIDS=()
cleanup() {
    echo ""
    echo "Shutting down Lunar Base Camp..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Shutdown complete."
}
trap cleanup EXIT INT TERM

# ------------------------------------------------------------- Gazebo
echo "[4/6] Starting Gazebo..."
if [ "$HEADLESS" -eq 1 ]; then
    $GZ_CMD -s -r -v 1 "$WORLD_PATH" &
else
    $GZ_CMD -r -v 1 "$WORLD_PATH" &
fi
PIDS+=($!)
sleep 8

# -------------------------------------------------------------- bridge
echo "[5/6] Starting ROS 2 <-> Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    "/cmd_vel@geometry_msgs/msg/Twist]${GZ_MSGS}.Twist" \
    "/lunabot/odom@nav_msgs/msg/Odometry[${GZ_MSGS}.Odometry" \
    "/tf@tf2_msgs/msg/TFMessage[${GZ_MSGS}.Pose_V" \
    "/lunabot/camera/image_raw@sensor_msgs/msg/Image[${GZ_MSGS}.Image" \
    "/lunabot/camera/camera_info@sensor_msgs/msg/CameraInfo[${GZ_MSGS}.CameraInfo" \
    "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[${GZ_MSGS}.LaserScan" \
    "/lunabot/imu@sensor_msgs/msg/Imu[${GZ_MSGS}.IMU" \
    "/model/lunabot_v4/pose@tf2_msgs/msg/TFMessage[${GZ_MSGS}.Pose_V" \
    > /dev/null 2>&1 &
PIDS+=($!)

# Sensor static transforms (base_link -> sensor frames).
ros2 run tf2_ros static_transform_publisher \
    0.20 0 0.725 0 0 0 base_link lunabot_camera > /dev/null 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    0.08 0 0.835 0 0 0 base_link lunabot_lidar > /dev/null 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    0 0 0.05 0 0 0 base_link lunabot_imu > /dev/null 2>&1 &
PIDS+=($!)

sleep 2

echo "[6/6] Ready."
echo ""
echo "============================================================"
echo " Lunar Base Camp is running."
echo ""
echo " Interfaces:"
echo "   /cmd_vel                  (geometry_msgs/Twist)   drive"
echo "   /lunabot/odom             (nav_msgs/Odometry)     wheel odometry"
echo "   /lunabot/camera/image_raw (sensor_msgs/Image)     camera"
echo "   /lunabot/lidar/scan       (sensor_msgs/LaserScan) range sensor"
echo "   /lunabot/imu              (sensor_msgs/Imu)       IMU"
echo ""
echo " Evaluation-only ground truth (NEVER used by the nav stack):"
echo "   /model/lunabot_v4/pose    (tf2_msgs/TFMessage)"
echo "============================================================"
echo ""

if [ "$START_TELEOP" -eq 1 ]; then
    python3 "$TELEOP_PATH"
else
    echo "Teleop disabled (--no-teleop). Press Ctrl+C to stop."
    # Keep the script alive until interrupted.
    while true; do sleep 3600; done
fi
