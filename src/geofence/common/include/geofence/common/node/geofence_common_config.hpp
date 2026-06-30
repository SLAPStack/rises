#pragma once

// Project headers
#include "geofence/common/policies/coordinate_transform.hpp"

// Third-party (ROS 2)
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Standard library
#include <cstdint>
#include <string>
#include <vector>

namespace rises
{

/**
 * @brief ROS parameters shared by every geofence lifecycle node (spatial and
 *        gridmap backends alike).
 *
 * Holds the 31 parameters that are identical in name and meaning across both
 * node implementations: safety circle, TF frames, robot-footprint filtering,
 * JSON pre-init, report-framing, and the coordinate-transform matrices. Each
 * concrete node config (GeofenceConfig / GridmapConfig) derives from this and
 * adds its backend-specific fields, so node code keeps accessing every field
 * directly through its own config object.
 *
 * NOTE (temporary location): this lives under spatial_node/include/geofence/
 * common/ because the geofence_common library currently sources from the
 * spatial_node include root. Relocating to a dedicated geofence/common/ tree is
 * a deferred follow-up (see the R2 plan's out-of-scope list).
 */
struct GeofenceCommonConfig
{
    // Safety / visualisation
    bool enable_safety_circle = true;
    float safety_circle_outer_radius = 0.50f;  // ROS param name: "safety_circle_radius"
    bool visualizer_enabled = true;

    // TF / frames
    bool use_namespace_for_map_frame = false;
    std::string target_frame = "map";
    std::string tf_prefix;
    std::string base_link_frame = "base_link";

    // Report framing
    bool publish_report_always = true;  // default differs per node; see declareAndRead()
    bool publish_report_in_local_frame = false;
    std::string report_output_frame;  // empty == use base_link_frame

    // Multi-robot filtering
    bool enable_robot_filtering = false;
    std::vector<std::string> robot_ids;
    std::string default_robot_footprint_type = "circle";
    double default_robot_footprint_radius = 0.5;
    double default_robot_footprint_width = 0.6;
    double default_robot_footprint_height = 0.8;
    std::vector<double> default_robot_footprint_polygon;
    double default_robot_footprint_margin = 0.1;
    int64_t robot_pose_max_age_ms = 2000;
    std::string robot_pose_topic_prefix = "robot_pose";

    // Pre-initialisation
    std::string obstacles_json_file;
    std::string contours_json_file;
    bool publish_ready_signal = false;

    // Coordinate transforms
    bool transform_obstacles_enabled = false;
    std::vector<double> obstacle_transform_matrix;
    bool transform_boundaries_enabled = false;
    std::vector<double> boundary_transform_matrix;
    bool transform_areas_enabled = false;
    std::vector<double> area_transform_matrix;
    bool negate_y_in_ros = false;

    // Auto-activate lifecycle
    bool auto_activate = false;

    /**
     * @brief Declare and read the shared parameters on @p node into @p cfg.
     *
     * Call once from the derived config's fromNode() before reading any
     * backend-specific parameters. The node must not have declared any of these
     * yet. @p publish_report_always_default supplies the one shared parameter
     * whose default legitimately differs between nodes (spatial=true,
     * gridmap=false); every other default is identical and baked in here.
     */
    static void declareAndRead(rclcpp_lifecycle::LifecycleNode& node,
                               GeofenceCommonConfig& cfg,
                               bool publish_report_always_default);

    /**
     * @brief Validate the shared safety invariants (subset common to all nodes).
     *
     * Returns human-readable error messages (empty == valid). Derived configs
     * append their own backend-specific checks.
     */
    [[nodiscard]] std::vector<std::string> validate() const;

    /**
     * @brief Build and apply the CoordinateTransform config from the loaded
     *        matrix parameters. Logs results to @p logger. Throws
     *        std::invalid_argument on a malformed (non-16-element) matrix.
     */
    void applyCoordinateTransform(rclcpp::Logger logger) const;
};

} // namespace rises
