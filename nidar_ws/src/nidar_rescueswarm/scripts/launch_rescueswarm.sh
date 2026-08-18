#!/usr/bin/env bash
# ============================================================================
# NIDAR RescueSwarm - Multi-Drone PX4 SITL Launch Script
# Launches Gazebo Sim with flood zone world + 2 PX4 x500_depth drones
# ============================================================================
set -e

PX4_DIR="${HOME}/PX4-Autopilot"
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"

# World file lives in nidar_gazebo package
NIDAR_WS="${HOME}/Documents/nidar/nidar_ws"
WORLD_FILE="${NIDAR_WS}/src/nidar_gazebo/worlds/rescueswarm_flood_zone.sdf"

# PX4 model paths
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
PX4_SERVER_CONFIG="${PX4_DIR}/Tools/simulation/gz/server.config"

NUM_DRONES=3

# Drone spawn positions (x,y,z,roll,pitch,yaw) — 8 meters apart
DRONE_POSES=(
    "0,-8.0,0.1,0,0,0"
    "0,0.0,0.1,0,0,0"
    "0,8.0,0.1,0,0,0"
)

echo "=============================================="
echo "  NIDAR RescueSwarm — PX4 Multi-Drone SITL"
echo "  Drones: ${NUM_DRONES} x x500_depth (OakD Camera + Aid Box)"
echo "  World:  rescueswarm_flood_zone"
echo "=============================================="

# Generate fresh randomized obstacle & survivor world for this launch
python3 "${NIDAR_WS}/src/nidar_gazebo/scripts/generate_random_world.py"

# Check that the world file exists
if [ ! -f "${WORLD_FILE}" ]; then
    echo "[ERROR] World file not found: ${WORLD_FILE}"
    exit 1
fi

# ------------------------------------------------------------------
# 1. Set Gazebo resource paths so it finds PX4 & custom models
# ------------------------------------------------------------------
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"

if [ -d "${HOME}/.simulation-gazebo/models" ]; then
    export GZ_SIM_RESOURCE_PATH="${HOME}/.simulation-gazebo/models:${GZ_SIM_RESOURCE_PATH}"
fi
if [ -d "${HOME}/gz_models" ]; then
    export GZ_SIM_RESOURCE_PATH="${HOME}/gz_models:${GZ_SIM_RESOURCE_PATH}"
fi
if [ -d "${HOME}/gz_worlds" ]; then
    export GZ_SIM_RESOURCE_PATH="${HOME}/gz_worlds:${GZ_SIM_RESOURCE_PATH}"
fi

# OGRE Render & Qt Optimization
export OGRE_RTT_MODE=Copy
export QT_X11_NO_MITSHM=1

# Use PX4's server.config and built system plugins
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_SERVER_CONFIG}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_BUILD}/src/modules/simulation/gz_plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

echo "[INFO] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "[INFO] GZ_SIM_SERVER_CONFIG_PATH=${GZ_SIM_SERVER_CONFIG_PATH}"
echo "[INFO] GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "[INFO] World file: ${WORLD_FILE}"

# ------------------------------------------------------------------
# 2. Kill any leftover processes from previous runs
# ------------------------------------------------------------------
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
sleep 1

# ------------------------------------------------------------------
# 3. Launch Gazebo Sim with the flood zone world (background)
# ------------------------------------------------------------------
echo "[INFO] Starting Gazebo Sim ..."
gz sim -r "${WORLD_FILE}" &
GZ_PID=$!
sleep 6
echo "[INFO] Gazebo started (PID: ${GZ_PID})"

# ------------------------------------------------------------------
# 4. Spawn PX4 SITL instances with x500_depth (Autostart 4002)
# ------------------------------------------------------------------
PX4_PIDS=()

for i in $(seq 0 $((NUM_DRONES - 1))); do
    INSTANCE=$i
    POSE="${DRONE_POSES[$i]}"

    echo "[INFO] Spawning PX4 instance ${INSTANCE} (x500_depth) at pose: ${POSE}"

    INSTANCE_DIR="/tmp/px4_sitl_rescueswarm_${INSTANCE}"
    rm -rf "${INSTANCE_DIR}"
    mkdir -p "${INSTANCE_DIR}"

    PX4_SYS_AUTOSTART=4002 \
    PX4_SIM_MODEL=gz_x500_depth \
    PX4_GZ_MODEL=x500_depth \
    PX4_GZ_MODEL_POSE="${POSE}" \
    PX4_GZ_WORLD=rescueswarm_flood_zone \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_NO_FOLLOW=1 \
    PX4_GZ_SIM_RENDER_ENGINE=ogre2 \
    ${PX4_BUILD}/bin/px4 \
        -i ${INSTANCE} \
        -d "${PX4_BUILD}/etc" \
        -w "${INSTANCE_DIR}" \
        -s "${PX4_BUILD}/etc/init.d-posix/rcS" \
        > "/tmp/px4_rescueswarm_${INSTANCE}.log" 2>&1 &

    PX4_PIDS+=($!)
    echo "[INFO] PX4 instance ${INSTANCE} started (PID: ${PX4_PIDS[-1]})"
    echo "       MAVSDK port: udp://:$((14540 + INSTANCE))"
    echo "       QGC port:    udp://:$((18570 + INSTANCE))"
    echo "       Log: /tmp/px4_rescueswarm_${INSTANCE}.log"

    sleep 5
done

# ------------------------------------------------------------------
# 5. Start ROS 2 <-> Gazebo Harmonic Bridge for 3 Drones
# ------------------------------------------------------------------
echo "[INFO] Launching ROS 2 Bridge for Drone 0, Drone 1 & Drone 2 ..."
source /opt/ros/humble/setup.bash 2>/dev/null || true

ros2 run ros_gz_bridge parameter_bridge \
  /world/rescueswarm_flood_zone/model/x500_depth_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /world/rescueswarm_flood_zone/model/x500_depth_1/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /world/rescueswarm_flood_zone/model/x500_depth_2/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /world/rescueswarm_flood_zone/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/rescueswarm_flood_zone/model/x500_depth_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/rescueswarm_flood_zone/model/x500_depth_2/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/rescueswarm_flood_zone/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/rescueswarm_flood_zone/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
  /world/rescueswarm_flood_zone/model/x500_depth_2/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args \
  -r /world/rescueswarm_flood_zone/model/x500_depth_0/link/link/sensor/lidar_2d_v2/scan:=/drone_0/scan \
  -r /world/rescueswarm_flood_zone/model/x500_depth_1/link/link/sensor/lidar_2d_v2/scan:=/drone_1/scan \
  -r /world/rescueswarm_flood_zone/model/x500_depth_2/link/link/sensor/lidar_2d_v2/scan:=/drone_2/scan \
  -r /world/rescueswarm_flood_zone/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/drone_0/camera/image_raw \
  -r /world/rescueswarm_flood_zone/model/x500_depth_1/link/camera_link/sensor/IMX214/image:=/drone_1/camera/image_raw \
  -r /world/rescueswarm_flood_zone/model/x500_depth_2/link/camera_link/sensor/IMX214/image:=/drone_2/camera/image_raw \
  -r /world/rescueswarm_flood_zone/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image:=/drone_0/depth/image_raw \
  -r /world/rescueswarm_flood_zone/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image:=/drone_1/depth/image_raw \
  -r /world/rescueswarm_flood_zone/model/x500_depth_2/link/camera_link/sensor/StereoOV7251/depth_image:=/drone_2/depth/image_raw > /tmp/ros_gz_bridge.log 2>&1 &

BRIDGE_PID=$!

echo ""
echo "=============================================="
echo "  All ${NUM_DRONES} drones spawned!"
echo ""
echo "  Drone 0: MAVSDK udp://:14540 (West, Y=-6m)"
echo "  Drone 1: MAVSDK udp://:14541 (Center, Y=0m)"
echo "  Drone 2: MAVSDK udp://:14542 (East, Y=+6m)"
echo ""
echo "  Run the RescueSwarm Mission & Perception Controller:"
echo "    python3 ${NIDAR_WS}/src/nidar_rescueswarm/scripts/rescueswarm_mission.py"
echo ""
echo "  Press Ctrl+C to shut down everything."
echo "=============================================="

cleanup() {
    echo ""
    echo "[INFO] Shutting down..."
    kill "$BRIDGE_PID" 2>/dev/null || true
    for pid in "${PX4_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    kill "$GZ_PID" 2>/dev/null || true
    sleep 1
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    echo "[INFO] Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $GZ_PID
