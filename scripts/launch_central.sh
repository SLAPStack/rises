#!/usr/bin/env bash
# ==============================================================================
# launch_central.sh – Launch the centralized ROS bridge (central.launch.py)
#
# All settings are controlled by environment variables.  Defaults match the
# values hard-coded in central.launch.py and central_params.yaml.
#
# USAGE
#   ./scripts/launch_central.sh                   # all defaults
#   BAG_FILE=/bags/run.db3 ./scripts/launch_central.sh
#   AGV_COUNT=3 BAG_FILE=/bags/run.db3 USE_SIM_TIME=true ./scripts/launch_central.sh
# ==============================================================================

set -euo pipefail

# -----------------------------------------------------------------------
# SOURCE ROS (skip if already sourced)
# -----------------------------------------------------------------------
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

# Source the workspace overlay if present
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SETUP="${WORKSPACE_DIR}/install/setup.bash"
if [[ -f "${SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${SETUP}"
fi

# -----------------------------------------------------------------------
# NODE PARAMETERS  (resolved via $(env VAR default) in central_params.yaml)
# -----------------------------------------------------------------------

# -- Multi-robot topology ------------------------------------------------
export AGV_COUNT="${AGV_COUNT:-1}"

# -- Map-update buffering ------------------------------------------------
export TRANSLATOR_ENABLE_BUFFERING="${TRANSLATOR_ENABLE_BUFFERING:-false}"
export TRANSLATOR_BUFFER_TIMEOUT="${TRANSLATOR_BUFFER_TIMEOUT:-1.0}"       # seconds

# -- Replay rate ---------------------------------------------------------
export TRANSLATOR_REPLAY_RATE="${TRANSLATOR_REPLAY_RATE:-33.0}"             # Hz

# -- Coordinate frame ----------------------------------------------------
export TRANSLATOR_TARGET_FRAME="${TRANSLATOR_TARGET_FRAME:-map}"
export TRANSLATOR_BASE_LINK_PUB_RATE="${TRANSLATOR_BASE_LINK_PUB_RATE:-20.0}"  # Hz

# -- Simulation time -----------------------------------------------------
export USE_SIM_TIME="${USE_SIM_TIME:-true}"

# -- Geofence-ready handshake --------------------------------------------
export TRANSLATOR_WAIT_FOR_GEOFENCE_READY="${TRANSLATOR_WAIT_FOR_GEOFENCE_READY:-false}"
export TRANSLATOR_GEOFENCE_READY_TIMEOUT="${TRANSLATOR_GEOFENCE_READY_TIMEOUT:-30.0}"  # seconds

# -----------------------------------------------------------------------
# LAUNCH ARGUMENTS  (topology / deployment – not in YAML)
# -----------------------------------------------------------------------

# -- Config file ---------------------------------------------------------
# Override to use a custom parameter file instead of the built-in default.
CENTRAL_PARAMS_FILE="${CENTRAL_PARAMS_FILE:-}"

# -- Rosbag playback -----------------------------------------------------
PLAY_ROSBAG="${PLAY_ROSBAG:-true}"
BAG_FILE="${BAG_FILE:-}"                # required when PLAY_ROSBAG=true
ROSBAG_RATE="${ROSBAG_RATE:-1.0}"       # playback speed multiplier
ROSBAG_LOOP="${ROSBAG_LOOP:-false}"
ROSBAG_DELAY="${ROSBAG_DELAY:-5.0}"     # seconds to wait before starting bag
ROSBAG_REMAPS="${ROSBAG_REMAPS:-}"      # comma-separated /from:=/to remaps
STORAGE="${STORAGE:-sqlite3}"           # sqlite3 or mcap

# -- Topic suppression ---------------------------------------------------
# Remap MQTT map topics to /unused/* (use when map is pre-loaded via JSON)
SUPPRESS_MAP_TOPICS="${SUPPRESS_MAP_TOPICS:-false}"

# -- RViz ----------------------------------------------------------------
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
RVIZ_NAMESPACE="${RVIZ_NAMESPACE:-agv_0}"

# -----------------------------------------------------------------------
# BUILD LAUNCH COMMAND
# -----------------------------------------------------------------------
CMD=(
    ros2 launch rises_bringup central.launch.py
    "use_sim_time:=${USE_SIM_TIME}"
    "play_rosbag:=${PLAY_ROSBAG}"
    "rosbag_rate:=${ROSBAG_RATE}"
    "rosbag_loop:=${ROSBAG_LOOP}"
    "rosbag_delay:=${ROSBAG_DELAY}"
    "storage:=${STORAGE}"
    "suppress_map_topics:=${SUPPRESS_MAP_TOPICS}"
    "launch_rviz:=${LAUNCH_RVIZ}"
    "rviz_namespace:=${RVIZ_NAMESPACE}"
)

# Optional args – only pass if explicitly set
[[ -n "${BAG_FILE}" ]]            && CMD+=("bag_file:=${BAG_FILE}")
[[ -n "${ROSBAG_REMAPS}" ]]       && CMD+=("rosbag_remaps:=${ROSBAG_REMAPS}")
[[ -n "${CENTRAL_PARAMS_FILE}" ]] && CMD+=("central_params_file:=${CENTRAL_PARAMS_FILE}")

# -----------------------------------------------------------------------
# PRINT CONFIGURATION
# -----------------------------------------------------------------------
echo "============================================================"
echo "  central.launch.py – configuration"
echo "============================================================"
echo "  AGV count               : ${AGV_COUNT}"
echo "  Use sim time            : ${USE_SIM_TIME}"
echo "  Play rosbag             : ${PLAY_ROSBAG}"
echo "  Bag file                : ${BAG_FILE:-<none>}"
echo "  Rosbag rate             : ${ROSBAG_RATE}x"
echo "  Rosbag loop             : ${ROSBAG_LOOP}"
echo "  Rosbag delay            : ${ROSBAG_DELAY}s"
echo "  Rosbag storage          : ${STORAGE}"
echo "  Rosbag remaps           : ${ROSBAG_REMAPS:-<none>}"
echo "  Suppress map topics     : ${SUPPRESS_MAP_TOPICS}"
echo "  Enable buffering        : ${TRANSLATOR_ENABLE_BUFFERING}"
echo "  Buffer timeout          : ${TRANSLATOR_BUFFER_TIMEOUT}s"
echo "  Replay rate             : ${TRANSLATOR_REPLAY_RATE} Hz"
echo "  Target frame            : ${TRANSLATOR_TARGET_FRAME}"
echo "  Base link pub rate      : ${TRANSLATOR_BASE_LINK_PUB_RATE} Hz"
echo "  Wait for geofence ready : ${TRANSLATOR_WAIT_FOR_GEOFENCE_READY}"
echo "  Geofence ready timeout  : ${TRANSLATOR_GEOFENCE_READY_TIMEOUT}s"
echo "  Launch RViz             : ${LAUNCH_RVIZ}"
echo "  RViz namespace          : ${RVIZ_NAMESPACE}"
echo "  Params file             : ${CENTRAL_PARAMS_FILE:-<default>}"
echo "============================================================"
echo ""

exec "${CMD[@]}"
