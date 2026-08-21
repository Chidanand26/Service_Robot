#!/bin/bash

# ============================================================
# AMR ServiceBot — ROS 2 Startup Script
# ============================================================
# Usage:
#   ./start_amr.sh              → Navigation mode (default)
#   ./start_amr.sh mapping      → Mapping mode (SLAM)
# ============================================================

LOCKFILE="/tmp/start_amr.lock"

# ---- Clean up previous background jobs ----
cleanup() {
    rm -f "$LOCKFILE" /dev/shm/fastrtps_port* 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Check if already running
if [ -f "$LOCKFILE" ]; then
    OLD_PID=$(cat "$LOCKFILE")
    if kill -0 "$OLD_PID" 2>/dev/null; then
        echo "⚠️  AMR is already running (PID $OLD_PID). Stopping old instance..."
        kill -9 "$OLD_PID" 2>/dev/null || true
    fi
    rm -f "$LOCKFILE"
fi

# ---- ROS 2 Environment ----
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# ---- Clean stale nodes and shared memory ----
echo "🧹 Preparing fresh environment..."
pkill -9 -f "sllidar_node" 2>/dev/null || true
pkill -9 -f "esp32_driver_node" 2>/dev/null || true
pkill -9 -f "twist_to_wheel_cmd" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "joy_node" 2>/dev/null || true
pkill -9 -f "teleop_node" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "velocity_smoother" 2>/dev/null || true
pkill -9 -f "collision_monitor" 2>/dev/null || true
pkill -9 -f "behavior_server" 2>/dev/null || true
pkill -9 -f "smoother_server" 2>/dev/null || true
pkill -9 -f "waypoint_follower" 2>/dev/null || true
pkill -9 -f "docking_server" 2>/dev/null || true
pkill -9 -f "route_server" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
sleep 2

# ---- Serial Device Permissions ----
sudo chmod 666 /dev/ttyUSB* /dev/input/js* 2>/dev/null || true

# ---- Record current PID ----
echo $$ > "$LOCKFILE"

# ---- Launch ----
MODE="${1:-navigation}"

if [ "$MODE" = "mapping" ]; then
    echo "🗺️  Starting AMR in MAPPING mode..."
    ros2 launch amr_assembly_description mapping.launch.py \
        use_esp32:=true \
        enable_camera:=false
else
    echo "🧭  Starting AMR in NAVIGATION mode..."
    ros2 launch amr_assembly_description navigation.launch.py \
        use_esp32:=true \
        map:=/home/ar08/ros2_ws/src/amr_assembly_description/maps/service_room.yaml \
        enable_camera:=false
fi
