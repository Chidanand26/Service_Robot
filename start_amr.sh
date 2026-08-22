#!/bin/bash

# ==============================================================================
# AMR ServiceBot — Master ROS 2 Startup Script
# ==============================================================================
# Usage:
#   ./start_amr.sh                          → Navigation with default map (service_room)
#   ./start_amr.sh <map_name_or_path>       → Navigation with specified map
#   ./start_amr.sh nav <map_name_or_path>   → Navigation with specified map
#   ./start_amr.sh mapping                  → SLAM Mapping mode
#   ./start_amr.sh list-maps                → List all available saved maps
#   ./start_amr.sh save-map <map_name>      → Save current SLAM map to maps/
# ==============================================================================

MAPS_DIR="/home/ar08/ros2_ws/src/amr_assembly_description/maps"
DEFAULT_MAP="$MAPS_DIR/service_room.yaml"
LOCKFILE="/tmp/start_amr.lock"

# ---- Clean up previous background jobs ----
cleanup() {
    rm -f "$LOCKFILE" /dev/shm/fastrtps_port* 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---- Map Lister Subcommand ----
if [ "$1" = "list-maps" ] || [ "$1" = "maps" ]; then
    echo "============================================================"
    echo "📂 Available AMR Maps in $MAPS_DIR:"
    echo "============================================================"
    found=0
    for f in "$MAPS_DIR"/*.yaml; do
        if [ -f "$f" ]; then
            name=$(basename "$f" .yaml)
            echo "  📍 $name  ($(basename "$f"))"
            grep -E "image:|resolution:|origin:" "$f" | sed 's/^/      /'
            echo ""
            found=1
        fi
    done
    if [ $found -eq 0 ]; then
        echo "  (No .yaml maps found in $MAPS_DIR)"
    fi
    exit 0
fi

# ---- Map Saver Subcommand ----
if [ "$1" = "save-map" ]; then
    MAP_NAME="${2:-service_room}"
    TARGET_PATH="$MAPS_DIR/$MAP_NAME"
    echo "💾 Saving live SLAM map to $TARGET_PATH..."
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run nav2_map_server map_saver_cli -f "$TARGET_PATH"
    echo "✅ Map saved! You can now run:"
    echo "   ./start_amr.sh $MAP_NAME"
    exit 0
fi

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
pkill -9 -f "async_slam_toolbox_node" 2>/dev/null || true
pkill -9 -f "slam_lifecycle_auto_starter" 2>/dev/null || true
rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
sleep 1.5

# ---- Serial Device Permissions ----
sudo chmod 666 /dev/ttyUSB* /dev/input/js* 2>/dev/null || true

# ---- Record current PID ----
echo $$ > "$LOCKFILE"

# ---- Determine Mode and Map ----
FIRST_ARG="${1:-navigation}"

if [ "$FIRST_ARG" = "mapping" ] || [ "$FIRST_ARG" = "slam" ]; then
    echo "🗺️  Starting AMR in MAPPING mode (SLAM)..."
    echo "   Use joystick to drive. When finished, run in another terminal:"
    echo "   ./start_amr.sh save-map <my_map_name>"
    ros2 launch amr_assembly_description mapping.launch.py \
        use_esp32:=true \
        enable_camera:=false
else
    # Navigation mode
    MAP_ARG="$DEFAULT_MAP"

    if [ "$FIRST_ARG" = "nav" ] || [ "$FIRST_ARG" = "navigation" ]; then
        if [ -n "$2" ]; then
            SELECTED_MAP="$2"
        fi
    elif [ -n "$FIRST_ARG" ] && [ "$FIRST_ARG" != "navigation" ]; then
        SELECTED_MAP="$FIRST_ARG"
    fi

    if [ -n "$SELECTED_MAP" ]; then
        if [ -f "$SELECTED_MAP" ]; then
            MAP_ARG=$(realpath "$SELECTED_MAP")
        elif [ -f "$MAPS_DIR/$SELECTED_MAP" ]; then
            MAP_ARG="$MAPS_DIR/$SELECTED_MAP"
        elif [ -f "$MAPS_DIR/$SELECTED_MAP.yaml" ]; then
            MAP_ARG="$MAPS_DIR/$SELECTED_MAP.yaml"
        else
            echo "❌ Error: Map '$SELECTED_MAP' not found!"
            echo "   Checked: $SELECTED_MAP"
            echo "   Checked: $MAPS_DIR/$SELECTED_MAP.yaml"
            echo ""
            echo "📂 Available maps in $MAPS_DIR:"
            ls -1 "$MAPS_DIR"/*.yaml 2>/dev/null || echo "   (No maps found)"
            exit 1
        fi
    fi

    # Verify YAML exists and referenced PGM image exists
    if [ ! -f "$MAP_ARG" ]; then
        echo "❌ Error: Map file does not exist: $MAP_ARG"
        exit 1
    fi

    MAP_IMG=$(grep -E "^image:" "$MAP_ARG" | awk '{print $2}' | tr -d '\r')
    MAP_DIR=$(dirname "$MAP_ARG")
    if [ -n "$MAP_IMG" ] && [ ! -f "$MAP_DIR/$MAP_IMG" ] && [ ! -f "$MAP_IMG" ]; then
        echo "⚠️  Warning: Map image '$MAP_IMG' referenced in $(basename "$MAP_ARG") was not found in $MAP_DIR!"
    fi

    echo "🧭 Starting AMR in NAVIGATION mode..."
    echo "   Map: $MAP_ARG"
    ros2 launch amr_assembly_description navigation.launch.py \
        use_esp32:=true \
        map:="$MAP_ARG" \
        enable_camera:=false
fi
