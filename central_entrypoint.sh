#!/bin/bash
set -e

# ===============================================================================
# CENTRAL BRIDGE ENTRYPOINT (ROSBAG MODE)
# ===============================================================================
#
# Launches central.launch.py with:
#   - centralized_translator (message_translator_node)
#   - optional rosbag player
#
# TRANSLATOR PARAMETERS are driven entirely by environment variables resolved
# inside central_params.yaml at launch time – no need to pass them as launch
# args.  Set the env vars below to override defaults from the YAML.
#
#   AGV_COUNT                          (default 1)
#   TRANSLATOR_ENABLE_BUFFERING        (default true)
#   TRANSLATOR_BUFFER_TIMEOUT          (default 1.0)
#   TRANSLATOR_REPLAY_RATE             (default 33.0)
#   TRANSLATOR_TARGET_FRAME            (default map)
#   TRANSLATOR_TF_PREFIX               (default "")
#   TRANSLATOR_BASE_LINK_PUB_RATE      (default 20.0)
#   TRANSLATOR_WAIT_FOR_GEOFENCE_READY (default true)
#   TRANSLATOR_GEOFENCE_READY_TIMEOUT  (default 30.0)
#   TRANSLATOR_PUBLISH_INIT_READY      (default true)
#   USE_SIM_TIME                       (default false)
#
# TOPOLOGY ARGS (launch-file concerns, passed as launch args):
#   SUPPRESS_MAP_TOPICS, LAUNCH_RVIZ, RVIZ_NAMESPACE, rosbag settings.
# ===============================================================================

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# -----------------------------------------------------------------------
# Topology configuration
# -----------------------------------------------------------------------

# Topic suppression (remapping concern – must stay in launch file)
SUPPRESS_MAP_TOPICS="${SUPPRESS_MAP_TOPICS:-false}"

# Rosbag
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
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-/workspace/resources/rises.rviz}"

# -----------------------------------------------------------------------
# Display configuration
# -----------------------------------------------------------------------
echo "═══════════════════════════════════════════════════════════"
echo "  Central Bridge Entrypoint"
echo "  AGV count    : ${AGV_COUNT:-1}"
echo "  Sim time     : ${USE_SIM_TIME:-false}"
echo "  Suppres maps : ${SUPPRESS_MAP_TOPICS}"
echo "  Launch RViz  : ${LAUNCH_RVIZ}"
[ "${PLAY_ROSBAG}" = "true" ] && echo "  Rosbag       : ${BAG_FILE} (rate=${ROSBAG_RATE}x, delay=${ROSBAG_DELAY}s)"
echo "═══════════════════════════════════════════════════════════"

# -----------------------------------------------------------------------
# Build launch args  (topology only – node params flow via env vars → YAML)
# -----------------------------------------------------------------------
LAUNCH_ARGS=()
LAUNCH_ARGS+=("use_sim_time:=${USE_SIM_TIME:-false}")
LAUNCH_ARGS+=("suppress_map_topics:=${SUPPRESS_MAP_TOPICS}")
LAUNCH_ARGS+=("launch_rviz:=${LAUNCH_RVIZ}")
LAUNCH_ARGS+=("rviz_namespace:=${RVIZ_NAMESPACE}")
LAUNCH_ARGS+=("rviz_config_file:=${RVIZ_CONFIG_FILE}")
LAUNCH_ARGS+=("storage:=${STORAGE}")

if [ "${PLAY_ROSBAG}" = "true" ]; then
    LAUNCH_ARGS+=("play_rosbag:=true")
    LAUNCH_ARGS+=("bag_file:=${BAG_FILE}")
    LAUNCH_ARGS+=("rosbag_rate:=${ROSBAG_RATE}")
    LAUNCH_ARGS+=("rosbag_loop:=${ROSBAG_LOOP}")
    LAUNCH_ARGS+=("rosbag_delay:=${ROSBAG_DELAY}")
    [ -n "${ROSBAG_REMAPS}" ] && LAUNCH_ARGS+=("rosbag_remaps:=${ROSBAG_REMAPS}")
fi

echo "Launching central.launch.py..."
printf '  %s\n' "${LAUNCH_ARGS[@]}"
echo "═══════════════════════════════════════════════════════════"

exec ros2 launch rises_bringup central.launch.py "${LAUNCH_ARGS[@]}"
