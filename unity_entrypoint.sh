#!/bin/bash
set -e

# ===============================================================================
# UNITY BRIDGE ENTRYPOINT
# ===============================================================================
#
# Launches unity.launch.py with:
#   - ROS-TCP-Endpoint (port 10000)
#   - centralized_translator (message_translator_node)
#   - optional rosbag player
#
# TRANSLATOR PARAMETERS are driven by environment variables resolved inside
# central_params.yaml at launch time.  See central_entrypoint.sh for the full
# list of supported env vars (includes TRANSLATOR_PUBLISH_INIT_READY, etc.).
#
# TOPOLOGY ARGS (launch-file concerns, passed as launch args):
#   ROS_TCP_IP, ROS_TCP_PORT, SUPPRESS_MAP_TOPICS, LAUNCH_RVIZ,
#   RVIZ_NAMESPACE, rosbag settings.
# ===============================================================================

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# -----------------------------------------------------------------------
# Topology configuration
# -----------------------------------------------------------------------

# TCP endpoint
ROS_TCP_IP="${ROS_TCP_IP:-0.0.0.0}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"

# Topic suppression (remapping concern – must stay in launch file)
SUPPRESS_MAP_TOPICS="${SUPPRESS_MAP_TOPICS:-false}"

# Rosbag (optional – alongside Unity)
BAG_FILE="${ROSBAG_FILE:-}"                   # accept ROSBAG_FILE for compat
PLAY_ROSBAG="false"
if [ -n "${BAG_FILE}" ] && [ -e "${BAG_FILE}" ]; then
    PLAY_ROSBAG="true"
fi
ROSBAG_RATE="${ROSBAG_RATE:-1.0}"
ROSBAG_LOOP="${ROSBAG_LOOP:-false}"
ROSBAG_DELAY="${ROSBAG_DELAY:-5.0}"
ROSBAG_REMAPS="${ROSBAG_REMAPS:-}"
STORAGE="${STORAGE:-sqlite3}"

# RViz
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
RVIZ_NAMESPACE="${RVIZ_NAMESPACE:-agv_0}"

# -----------------------------------------------------------------------
# Display configuration
# -----------------------------------------------------------------------
echo "═══════════════════════════════════════════════════════════"
echo "  Unity Bridge Entrypoint"
echo "  TCP endpoint : ${ROS_TCP_IP}:${ROS_TCP_PORT}"
echo "  AGV count    : ${AGV_COUNT:-1}"
echo "  Sim time     : ${USE_SIM_TIME:-false}"
echo "  Suppress maps: ${SUPPRESS_MAP_TOPICS}"
echo "  Launch RViz  : ${LAUNCH_RVIZ}"
[ "${PLAY_ROSBAG}" = "true" ] && echo "  Rosbag       : ${BAG_FILE} (rate=${ROSBAG_RATE}x, delay=${ROSBAG_DELAY}s)"
echo "═══════════════════════════════════════════════════════════"

# -----------------------------------------------------------------------
# Build launch args  (topology only – node params flow via env vars → YAML)
# -----------------------------------------------------------------------
LAUNCH_ARGS=()
LAUNCH_ARGS+=("tcp_ip:=${ROS_TCP_IP}")
LAUNCH_ARGS+=("tcp_port:=${ROS_TCP_PORT}")
LAUNCH_ARGS+=("suppress_map_topics:=${SUPPRESS_MAP_TOPICS}")
LAUNCH_ARGS+=("launch_rviz:=${LAUNCH_RVIZ}")
LAUNCH_ARGS+=("rviz_namespace:=${RVIZ_NAMESPACE}")
LAUNCH_ARGS+=("storage:=${STORAGE}")

if [ "${PLAY_ROSBAG}" = "true" ]; then
    LAUNCH_ARGS+=("play_rosbag:=true")
    LAUNCH_ARGS+=("bag_file:=${BAG_FILE}")
    LAUNCH_ARGS+=("rosbag_rate:=${ROSBAG_RATE}")
    LAUNCH_ARGS+=("rosbag_loop:=${ROSBAG_LOOP}")
    LAUNCH_ARGS+=("rosbag_delay:=${ROSBAG_DELAY}")
    [ -n "${ROSBAG_REMAPS}" ] && LAUNCH_ARGS+=("rosbag_remaps:=${ROSBAG_REMAPS}")
fi

echo "Launching unity.launch.py..."
printf '  %s\n' "${LAUNCH_ARGS[@]}"
echo "═══════════════════════════════════════════════════════════"

exec ros2 launch rises_bringup unity.launch.py "${LAUNCH_ARGS[@]}"
