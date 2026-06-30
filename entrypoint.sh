#!/bin/bash
set -e

# ===============================================================================
# GEOFENCE SYSTEM ENTRYPOINT
# ===============================================================================
#
# Selects the right scenario parameter file and launch file based on env vars,
# then launches the geofence stack.
#
# KEY ENV VARS
# ------------
#   SCENARIO         : Parameter scenario to use (default | rosbag | unity).
#                      Selects the YAML file passed as params_file to the launch.
#                      default → params_default.yaml  (real robot)
#                      rosbag  → params_rosbag.yaml   (use_sim_time, validation on)
#                      unity   → params_unity.yaml    (coordinate transforms)
#
#   AGV_DEPLOY_MODE  : Controls how AGVs are distributed across containers.
#
#     multi    → All AGVs run in THIS container (multi_agv_geofence.launch.py).
#                Set AGV_COUNT to the total number of AGVs.
#                Typical topology: one TCP container + this container.
#
#     per_agv  → This container handles exactly ONE AGV (geofence.launch.py).
#                Set NAMESPACE to identify which AGV (e.g. "agv_0").
#                Typical topology: one TCP container + N containers like this.
#
#     (unset)  → Legacy auto-selection: AGV_COUNT > 1 → multi, else → per_agv.
#
#   AGV_COUNT  : Number of AGVs. Used by multi mode (agv_0..agv_N-1).
#                Ignored in per_agv mode (only NAMESPACE matters there).
#
#   NAMESPACE  : ROS namespace for per_agv mode (e.g. "agv_0").
#
# NODE PARAMETER OVERRIDES
# ------------------------
#   All node parameters are stored in the selected scenario YAML and can be
#   overridden at runtime via the environment variables defined in each YAML
#   (e.g. GEOFENCE_SAFETY_RADIUS, LASERSCAN_DBSCAN_EPS).
#   See rises_bringup/config/params_*.yaml for the full list.
#
# TOPOLOGY ARGS (launch-file concerns, not in YAML)
# -------------------------------------------------
#   SCAN_TOPIC_REMAP, PUBLISH_STATIC_TF, PUBLISH_LIDAR_STATIC_TF,
#   PUBLISH_UNITY_TF, PUBLISH_SLAPSTACK_TF, static TF transform values,
#   PLAY_ROSBAG, BAG_FILE, rosbag playback settings.
#
# ===============================================================================

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# If arguments are provided, execute them directly (e.g. a shell or custom cmd)
if [ $# -gt 0 ]; then
    exec "$@"
fi

# ===============================================================================
# SCENARIO → PARAMS FILE SELECTION
# ===============================================================================

SCENARIO="${SCENARIO:-default}"
CONFIG_DIR="/workspace/install/rises_bringup/share/rises_bringup/config"

case "${SCENARIO}" in
    rosbag)
        PARAMS_FILE="${PARAMS_FILE:-${CONFIG_DIR}/params_rosbag.yaml}"
        ;;
    unity)
        PARAMS_FILE="${PARAMS_FILE:-${CONFIG_DIR}/params_unity.yaml}"
        ;;
    *)
        PARAMS_FILE="${PARAMS_FILE:-${CONFIG_DIR}/params_default.yaml}"
        ;;
esac

# ===============================================================================
# ENSURE OPTIONAL ENV VARS ARE EXPORTED (prevents $(env VAR) parse errors)
# ===============================================================================
# UNITY → YAML ENV VAR COMPATIBILITY BRIDGE
# ===============================================================================
#
# Unity's ROSGeofenceManager sends env vars WITHOUT the GEOFENCE_/LASERSCAN_
# prefix that the YAML $(env ...) substitutions expect.  Map them here so that
# both the prefixed form (set externally) and the unprefixed Unity form work.
#
# Rule: prefixed form wins if already set; otherwise fall back to Unity's form.
#
# Helper: export a GEOFENCE_* / LASERSCAN_* variable only when the resolved
# value is non-empty.  If the value is empty we leave the variable UNSET so
# that the YAML default (in the $(env VAR default) substitution) is used
# instead of being overridden by an empty string (which YAML reads as null).
_export_if_set() { local _v="${!1:-${!2:-}}"; [ -n "$_v" ] && export "$1=$_v" || true; }

# geofence_node -----------------------------------------------------------
_export_if_set GEOFENCE_ENABLE_SAFETY_CIRCLE      ENABLE_SAFETY_CIRCLE
_export_if_set GEOFENCE_SAFETY_RADIUS              SAFETY_CIRCLE_RADIUS
_export_if_set GEOFENCE_ENABLE_ROBOT_FILTERING     ENABLE_ROBOT_FILTERING
_export_if_set GEOFENCE_BASE_LINK_FRAME            BASE_LINK_FRAME
_export_if_set GEOFENCE_ROBOT_FOOTPRINT_TYPE       ROBOT_FOOTPRINT_TYPE
_export_if_set GEOFENCE_ROBOT_FOOTPRINT_RADIUS     ROBOT_FOOTPRINT_RADIUS
_export_if_set GEOFENCE_ROBOT_FOOTPRINT_WIDTH      ROBOT_FOOTPRINT_WIDTH
_export_if_set GEOFENCE_ROBOT_FOOTPRINT_HEIGHT     ROBOT_FOOTPRINT_HEIGHT
_export_if_set GEOFENCE_ROBOT_FOOTPRINT_MARGIN     ROBOT_FOOTPRINT_MARGIN
_export_if_set GEOFENCE_PUBLISH_READY_SIGNAL       PUBLISH_READY_SIGNAL
_export_if_set GEOFENCE_TRANSFORM_OBSTACLES_ENABLED TRANSFORM_OBSTACLES_ENABLED
_export_if_set GEOFENCE_OBSTACLE_TRANSFORM_MATRIX  OBSTACLE_TRANSFORM_MATRIX
_export_if_set GEOFENCE_TRANSFORM_BOUNDARIES_ENABLED TRANSFORM_BOUNDARIES_ENABLED
_export_if_set GEOFENCE_BOUNDARY_TRANSFORM_MATRIX  BOUNDARY_TRANSFORM_MATRIX
_export_if_set GEOFENCE_TRANSFORM_AREAS_ENABLED    TRANSFORM_AREAS_ENABLED
_export_if_set GEOFENCE_AREA_TRANSFORM_MATRIX      AREA_TRANSFORM_MATRIX
_export_if_set GEOFENCE_NEGATE_Y_IN_ROS            NEGATE_Y_IN_ROS
# laserscan_preprocessor_node --------------------------------------------
_export_if_set LASERSCAN_DISTANCE_THRESHOLD        SEGMENT_DISTANCE_THRESHOLD
_export_if_set LASERSCAN_ANGLE_THRESHOLD_DEG       SEGMENT_ANGLE_THRESHOLD_DEG
_export_if_set LASERSCAN_PUBLISH_POINTS_ONLY       PUBLISH_POINTS_ONLY
_export_if_set LASERSCAN_DBSCAN_EPS                DBSCAN_EPS
_export_if_set LASERSCAN_DBSCAN_MIN_POINTS         DBSCAN_MIN_POINTS
_export_if_set LASERSCAN_USE_ADAPTIVE_THRESHOLDING USE_ADAPTIVE_THRESHOLDING
# Vars with NO default in YAML – always export (even empty → YAML quotes
# in the file ensure "" not null).
export GEOFENCE_TF_PREFIX="${GEOFENCE_TF_PREFIX-${TF_PREFIX:-}}"
export GEOFENCE_OBSTACLES_JSON_FILE="${GEOFENCE_OBSTACLES_JSON_FILE:-${OBSTACLES_JSON_FILE:-}}"
export GEOFENCE_CONTOURS_JSON_FILE="${GEOFENCE_CONTOURS_JSON_FILE:-${CONTOURS_JSON_FILE:-}}"
export LASERSCAN_TF_PREFIX="${LASERSCAN_TF_PREFIX:-}"
export TRANSLATOR_TF_PREFIX="${TRANSLATOR_TF_PREFIX:-}"
# fleet_interface_node --------------------------------------------------------
export FLEET_TF_PREFIX="${FLEET_TF_PREFIX:-${TF_PREFIX:-}}"
# MQTT AGV name: explicit env wins, then derive from NAMESPACE (agv_0 → id0)
if [ -n "${FLEET_MQTT_AGV_NAME:-${MQTT_AGV_NAME:-}}" ]; then
    export FLEET_MQTT_AGV_NAME="${FLEET_MQTT_AGV_NAME:-${MQTT_AGV_NAME}}"
elif [ -n "${NAMESPACE:-}" ]; then
    export FLEET_MQTT_AGV_NAME="id${NAMESPACE##*_}"
else
    export FLEET_MQTT_AGV_NAME=""
fi

# ===============================================================================
# TOPOLOGY LAUNCH ARGS
# ===============================================================================

# Scan topic remapping (which topic laserscan subscribes to – topology concern)
SCAN_TOPIC_REMAP="${SCAN_TOPIC_REMAP:-}"

# Static TF publishers (which nodes to create – topology concern)
PUBLISH_STATIC_TF="${PUBLISH_STATIC_TF:-false}"
STATIC_TF_PARENT_FRAME="${STATIC_TF_PARENT_FRAME:-base_link}"
STATIC_TF_CHILD_FRAME="${STATIC_TF_CHILD_FRAME:-laser_link}"
STATIC_TF_USE_NAMESPACE="${STATIC_TF_USE_NAMESPACE:-true}"
STATIC_TF_X="${STATIC_TF_X:-0.0}"
STATIC_TF_Y="${STATIC_TF_Y:-0.0}"
STATIC_TF_Z="${STATIC_TF_Z:-0.0}"
STATIC_TF_ROLL="${STATIC_TF_ROLL:-0.0}"
STATIC_TF_PITCH="${STATIC_TF_PITCH:-0.0}"
STATIC_TF_YAW="${STATIC_TF_YAW:-0.0}"

PUBLISH_LIDAR_STATIC_TF="${PUBLISH_LIDAR_STATIC_TF:-false}"
LIDAR_TRANSFORM_X="${LIDAR_TRANSFORM_X:-0.0}"
LIDAR_TRANSFORM_Y="${LIDAR_TRANSFORM_Y:-0.0}"
LIDAR_TRANSFORM_Z="${LIDAR_TRANSFORM_Z:-0.0}"
LIDAR_TRANSFORM_ROLL="${LIDAR_TRANSFORM_ROLL:-0.0}"
LIDAR_TRANSFORM_PITCH="${LIDAR_TRANSFORM_PITCH:-0.0}"
LIDAR_TRANSFORM_YAW="${LIDAR_TRANSFORM_YAW:-0.0}"

PUBLISH_UNITY_TF="${PUBLISH_UNITY_TF:-false}"
PUBLISH_SLAPSTACK_TF="${PUBLISH_SLAPSTACK_TF:-false}"
TARGET_FRAME="${TARGET_FRAME:-map}"

# Rosbag player (managed by launch file)
PLAY_ROSBAG="${PLAY_ROSBAG:-false}"
BAG_FILE="${BAG_FILE:-}"
STORAGE_BACKEND="${STORAGE_BACKEND:-sqlite3}"
ROSBAG_DELAY="${ROSBAG_DELAY:-3.0}"
ROSBAG_RATE="${ROSBAG_RATE:-1.0}"
ROSBAG_LOOP="${ROSBAG_LOOP:-false}"
ROSBAG_REMAPS="${ROSBAG_REMAPS:-}"

# RViz
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
RVIZ_NAMESPACE="${RVIZ_NAMESPACE:-}"

# Optional per-AGV nodes (topology concerns – forwarded as launch args)
TRANSLATOR_ENABLED="${TRANSLATOR_ENABLED:-false}"
GRIDMAP_ENABLED="${GRIDMAP_ENABLED:-false}"
VALIDATION_ENABLED="${VALIDATION_ENABLED:-false}"
LAUNCH_LEG_FILTER="${LAUNCH_LEG_FILTER:-false}"

# ===============================================================================
# SINGLE vs MULTI-AGV LAUNCH
# ===============================================================================

AGV_COUNT="${AGV_COUNT:-1}"

# Resolve deploy mode: explicit flag takes priority; fall back to AGV_COUNT.
AGV_DEPLOY_MODE="${AGV_DEPLOY_MODE:-}"
if [ "${AGV_DEPLOY_MODE}" = "multi" ]; then
    _USE_MULTI=true
elif [ "${AGV_DEPLOY_MODE}" = "per_agv" ]; then
    _USE_MULTI=false
else
    # Legacy auto-selection
    if [ "${AGV_COUNT}" -gt 1 ]; then
        _USE_MULTI=true
    else
        _USE_MULTI=false
    fi
fi

echo "═══════════════════════════════════════════════════════════"
echo "  Geofence System Entrypoint"
echo "  Scenario    : ${SCENARIO}  →  ${PARAMS_FILE}"
echo "  Deploy mode : ${AGV_DEPLOY_MODE:-auto (AGV_COUNT=${AGV_COUNT})}"
if [ "${_USE_MULTI}" = "true" ]; then
    echo "  AGV count   : ${AGV_COUNT}  (all in this container)"
else
    echo "  Namespace   : ${NAMESPACE:-<global>}  (one AGV per container)"
fi
echo "═══════════════════════════════════════════════════════════"

# Build shared topology args
LAUNCH_ARGS=()
LAUNCH_ARGS+=("params_file:=${PARAMS_FILE}")

# Scan topic remap
[ -n "${SCAN_TOPIC_REMAP}" ] && LAUNCH_ARGS+=("scan_topic_remap:=${SCAN_TOPIC_REMAP}")

# Static TF publishers
LAUNCH_ARGS+=("publish_static_tf:=${PUBLISH_STATIC_TF}")
LAUNCH_ARGS+=("static_tf_parent_frame:=${STATIC_TF_PARENT_FRAME}")
LAUNCH_ARGS+=("static_tf_child_frame:=${STATIC_TF_CHILD_FRAME}")
LAUNCH_ARGS+=("static_tf_use_namespace:=${STATIC_TF_USE_NAMESPACE}")
LAUNCH_ARGS+=("static_tf_x:=${STATIC_TF_X}")
LAUNCH_ARGS+=("static_tf_y:=${STATIC_TF_Y}")
LAUNCH_ARGS+=("static_tf_z:=${STATIC_TF_Z}")
LAUNCH_ARGS+=("static_tf_roll:=${STATIC_TF_ROLL}")
LAUNCH_ARGS+=("static_tf_pitch:=${STATIC_TF_PITCH}")
LAUNCH_ARGS+=("static_tf_yaw:=${STATIC_TF_YAW}")
LAUNCH_ARGS+=("publish_lidar_static_tf:=${PUBLISH_LIDAR_STATIC_TF}")
LAUNCH_ARGS+=("lidar_transform_x:=${LIDAR_TRANSFORM_X}")
LAUNCH_ARGS+=("lidar_transform_y:=${LIDAR_TRANSFORM_Y}")
LAUNCH_ARGS+=("lidar_transform_z:=${LIDAR_TRANSFORM_Z}")
LAUNCH_ARGS+=("lidar_transform_roll:=${LIDAR_TRANSFORM_ROLL}")
LAUNCH_ARGS+=("lidar_transform_pitch:=${LIDAR_TRANSFORM_PITCH}")
LAUNCH_ARGS+=("lidar_transform_yaw:=${LIDAR_TRANSFORM_YAW}")
LAUNCH_ARGS+=("publish_unity_tf:=${PUBLISH_UNITY_TF}")
LAUNCH_ARGS+=("publish_slapstack_tf:=${PUBLISH_SLAPSTACK_TF}")
LAUNCH_ARGS+=("target_frame:=${TARGET_FRAME}")

# Rosbag
LAUNCH_ARGS+=("play_rosbag:=${PLAY_ROSBAG}")
[ -n "${BAG_FILE}" ] && LAUNCH_ARGS+=("bag_file:=${BAG_FILE}")
LAUNCH_ARGS+=("storage:=${STORAGE_BACKEND}")
LAUNCH_ARGS+=("rosbag_delay:=${ROSBAG_DELAY}")
LAUNCH_ARGS+=("rosbag_rate:=${ROSBAG_RATE}")
LAUNCH_ARGS+=("rosbag_loop:=${ROSBAG_LOOP}")
[ -n "${ROSBAG_REMAPS}" ] && LAUNCH_ARGS+=("rosbag_remaps:=${ROSBAG_REMAPS}")

# RViz
LAUNCH_ARGS+=("launch_rviz:=${LAUNCH_RVIZ}")
[ -n "${RVIZ_NAMESPACE}" ] && LAUNCH_ARGS+=("rviz_namespace:=${RVIZ_NAMESPACE}")

# Optional per-AGV nodes
LAUNCH_ARGS+=("translator_enabled:=${TRANSLATOR_ENABLED}")
LAUNCH_ARGS+=("gridmap_enabled:=${GRIDMAP_ENABLED}")
LAUNCH_ARGS+=("validation_enabled:=${VALIDATION_ENABLED}")
LAUNCH_ARGS+=("launch_leg_filter:=${LAUNCH_LEG_FILTER}")

if [ "${_USE_MULTI}" = "true" ]; then
    # -------------------------------------------------------------------------
    # MULTI mode: one composable container with all AGV nodes.
    # Namespaces are generated as agv_0..agv_N-1 inside the launch file.
    # -------------------------------------------------------------------------
    LAUNCH_ARGS+=("agv_count:=${AGV_COUNT}")

    echo "Launching multi_agv_geofence.launch.py (${AGV_COUNT} AGVs)..."
    printf '  %s\n' "${LAUNCH_ARGS[@]}"
    echo "═══════════════════════════════════════════════════════════"
    exec ros2 launch rises_bringup multi_agv_geofence.launch.py "${LAUNCH_ARGS[@]}"
else
    # -------------------------------------------------------------------------
    # PER-AGV mode: standard single-AGV geofence stack.
    # Each container is responsible for exactly one AGV; NAMESPACE selects it.
    # -------------------------------------------------------------------------
    [ -n "${NAMESPACE}" ] && LAUNCH_ARGS+=("namespace:=${NAMESPACE}")

    # ARISE mode: launch the full ARISE stack (skill bridge, heatmap, fiware bridge, etc.)
    if [ "${ARISE_MODE:-false}" = "true" ]; then
        [ -n "${LAUNCH_SKILL_BRIDGE}" ]        && LAUNCH_ARGS+=("launch_skill_bridge:=${LAUNCH_SKILL_BRIDGE}")
        [ -n "${LAUNCH_MISSION_CONTROLLER}" ]   && LAUNCH_ARGS+=("launch_mission_controller:=${LAUNCH_MISSION_CONTROLLER}")
        [ -n "${LAUNCH_HEATMAP}" ]              && LAUNCH_ARGS+=("launch_heatmap:=${LAUNCH_HEATMAP}")
        [ -n "${LAUNCH_FIWARE_BRIDGE}" ]        && LAUNCH_ARGS+=("launch_fiware_bridge:=${LAUNCH_FIWARE_BRIDGE}")
        # Heatmap grid geometry (center + size; defaults in rises_geofence.launch.py)
        [ -n "${HEATMAP_GRID_CENTER_X}" ]    && LAUNCH_ARGS+=("heatmap_grid_center_x:=${HEATMAP_GRID_CENTER_X}")
        [ -n "${HEATMAP_GRID_CENTER_Y}" ]    && LAUNCH_ARGS+=("heatmap_grid_center_y:=${HEATMAP_GRID_CENTER_Y}")
        [ -n "${HEATMAP_GRID_WIDTH}" ]       && LAUNCH_ARGS+=("heatmap_grid_width:=${HEATMAP_GRID_WIDTH}")
        [ -n "${HEATMAP_GRID_HEIGHT}" ]      && LAUNCH_ARGS+=("heatmap_grid_height:=${HEATMAP_GRID_HEIGHT}")
        [ -n "${HEATMAP_MIN_OBSERVATIONS}" ]    && LAUNCH_ARGS+=("heatmap_min_observations:=${HEATMAP_MIN_OBSERVATIONS}")
        [ -n "${HEATMAP_GAUSSIAN_SIGMA}" ]      && LAUNCH_ARGS+=("heatmap_gaussian_sigma:=${HEATMAP_GAUSSIAN_SIGMA}")
        [ -n "${HEATMAP_PREDICTION_HORIZON}" ]  && LAUNCH_ARGS+=("prediction_horizon_sec:=${HEATMAP_PREDICTION_HORIZON}")
        [ -n "${HEATMAP_PREDICTION_STEP}" ]     && LAUNCH_ARGS+=("heatmap_prediction_step_sec:=${HEATMAP_PREDICTION_STEP}")

        echo "Launching rises_geofence.launch.py (namespace=${NAMESPACE:-<global>}, ARISE mode)..."
        printf '  %s\n' "${LAUNCH_ARGS[@]}"
        echo "═══════════════════════════════════════════════════════════"
        exec ros2 launch rises_bringup rises_geofence.launch.py "${LAUNCH_ARGS[@]}"
    else
        echo "Launching geofence.launch.py (namespace=${NAMESPACE:-<global>})..."
        printf '  %s\n' "${LAUNCH_ARGS[@]}"
        echo "═══════════════════════════════════════════════════════════"
        exec ros2 launch rises_bringup geofence.launch.py "${LAUNCH_ARGS[@]}"
    fi
fi
