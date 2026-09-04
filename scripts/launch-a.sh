cat > ~/lunabot-v4/scripts/launch-a.sh << 'EOF'
#!/bin/bash

# ============================================================
# Lunabot V4 - Phase 1 - Launch A
# Simulation & Hardware Baseline Validation
# ============================================================

set -u

echo "============================================================"
echo "        Lunabot V4 - Launch A Validation"
echo "============================================================"
echo ""

# ------------------------------------------------------------
# 1. Paths
# ------------------------------------------------------------

REPO_DIR="$HOME/lunabot-v4"
WORKSPACE_DIR="$HOME/lunabot_ws"

MODEL_PATH="$REPO_DIR/src/lunabot_gazebo/models/lunabot_v4/model.sdf"
WORLD_PATH="$REPO_DIR/src/lunabot_gazebo/worlds/lunar_world.sdf"
WASD_PATH="$REPO_DIR/scripts/wasd_teleop.py"

# ------------------------------------------------------------
# 2. Check required files
# ------------------------------------------------------------

echo "[1/8] Checking project files..."

if [ ! -f "$MODEL_PATH" ]; then
    echo "ERROR: Lunabot model not found:"
    echo "$MODEL_PATH"
    exit 1
fi

if [ ! -f "$WORLD_PATH" ]; then
    echo "ERROR: Gazebo world not found:"
    echo "$WORLD_PATH"
    exit 1
fi

if [ ! -f "$WASD_PATH" ]; then
    echo "ERROR: WASD controller not found:"
    echo "$WASD_PATH"
    exit 1
fi

echo "      Model:      OK"
echo "      World:      OK"
echo "      Controller: OK"
echo ""

# ------------------------------------------------------------
# 3. Source ROS 2
# ------------------------------------------------------------

echo "[2/8] Loading ROS 2 Humble..."

source /opt/ros/humble/setup.bash

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "      lunabot_ws workspace loaded."
else
    echo "WARNING: $WORKSPACE_DIR/install/setup.bash not found."
    echo "         Continuing with system ROS 2."
fi

echo ""

# ------------------------------------------------------------
# 4. Start Gazebo
# ------------------------------------------------------------

echo "[3/8] Starting Gazebo..."

ign gazebo -v 4 "$WORLD_PATH" &
GAZEBO_PID=$!

echo "      Gazebo PID: $GAZEBO_PID"
echo "      Waiting for Gazebo to initialize..."
sleep 7

# ------------------------------------------------------------
# 5. Spawn Lunabot V4
# ------------------------------------------------------------

echo "[4/8] Spawning Lunabot V4..."

ign service -s /world/lunar_world/create \
    --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean \
    --timeout 1000 \
    --req "sdf_filename: \"$MODEL_PATH\", pose: {position: {z: 80.2}}"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to spawn Lunabot V4."
    kill "$GAZEBO_PID" 2>/dev/null
    exit 1
fi

echo "      Lunabot V4 spawn request sent."
echo ""

# ------------------------------------------------------------
# 6. Start ROS 2 <-> Gazebo bridge
# ------------------------------------------------------------

echo "[5/8] Starting ROS 2 - Gazebo bridge..."

ros2 run ros_ign_bridge parameter_bridge \
    "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" \
    "/lunabot/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image" \
    "/lunabot/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
    "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V" \
    "/lunabot/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
    > /dev/null 2>&1 &

BRIDGE_PID=$!

echo "      Bridge PID: $BRIDGE_PID"
echo ""

# ------------------------------------------------------------
# 7. Start TF + RViz2
# ------------------------------------------------------------

echo "[6/8] Starting LiDAR TF..."

ros2 run tf2_ros static_transform_publisher \
    0.2 0 0.9 0 0.5 0 \
    chassis \
    lunabot_v4/sensor_head/lidar \
    > /dev/null 2>&1 &

TF_PID=$!

echo "      TF PID: $TF_PID"
echo ""

echo "[7/8] Launching RViz2..."

rviz2 -f lunabot_v4/sensor_head/lidar \
    > /dev/null 2>&1 &

RVIZ_PID=$!

echo "      RViz2 PID: $RVIZ_PID"
echo ""

# ------------------------------------------------------------
# 8. Start WASD controller
# ------------------------------------------------------------

echo "[8/8] Launching WASD Controller..."
echo ""
echo "============================================================"
echo " Launch A environment is running."
echo ""
echo " Controls:"
echo "   W = Forward"
echo "   S = Reverse"
echo "   A = Turn Left"
echo "   D = Turn Right"
echo "   Q = Quit"
echo ""
echo " RViz2:"
echo "   Camera -> /lunabot/camera/image_raw"
echo "   LiDAR  -> /lunabot/lidar/scan"
echo ""
echo "============================================================"
echo ""

python3 "$WASD_PATH"

# ------------------------------------------------------------
# Shutdown
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo "Shutting down Launch A..."
echo "============================================================"

echo "Stopping RViz2..."
kill "$RVIZ_PID" 2>/dev/null

echo "Stopping TF publisher..."
kill "$TF_PID" 2>/dev/null

echo "Stopping ROS-Gazebo bridge..."
kill "$BRIDGE_PID" 2>/dev/null

echo "Stopping Gazebo..."
kill "$GAZEBO_PID" 2>/dev/null

wait "$GAZEBO_PID" 2>/dev/null

echo ""
echo "============================================================"
echo "Launch A environment cleanly closed."
echo "============================================================"
EOF
