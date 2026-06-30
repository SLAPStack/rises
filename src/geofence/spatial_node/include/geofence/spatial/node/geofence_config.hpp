#pragma once

// Project headers
#include "geofence/common/node/geofence_common_config.hpp"

// Third-party (ROS 2)
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Standard library
#include <string>
#include <vector>

namespace rises
{

/**
 * @brief All ROS parameters for the spatial GeofenceNode, loaded once in the
 *        constructor.
 *
 * Derives from GeofenceCommonConfig (the 31 parameters shared with every
 * geofence node) and adds the spatial-backend-specific parameters
 * (correspondence matching, lidar segmentation, height awareness). Node code
 * accesses every field -- shared or spatial -- directly through this object.
 */
struct GeofenceConfig : GeofenceCommonConfig
{
    // Core detection (spatial backend)
    float correspondence_tolerance = 0.10f;
    bool process_points_only = true;
    bool ignore_outside_warehouse = false;

    // Segmentation
    float segment_min_gap = 0.05f;
    float segment_gap_multiplier = 8.0f;
    float lidar_angle_increment = 0.001745f;
    float line_fit_tolerance = 0.05f;
    float error_segment_tracker_cell_size = 0.3f;
    float error_segment_tracker_max_drift = 1.0f;
    int min_segment_points = 3;  // segments with fewer points are discarded as noise
    bool enable_error_segment_tracking = true;
    bool report_error_segments_as_points = false;
    float outlier_filter_distance = 0.10f;

    // Height awareness
    bool enable_height_awareness = false;
    std::string lidar_height_source = "tf";
    double lidar_height_fixed = 0.0;
    std::string lidar_frame = "laser_link";
    float height_match_tolerance = 0.05f;
    std::string height_match_mode = "intersection";

    /**
     * @brief Declare all parameters on the node and read their values.
     *
     * Call once in the GeofenceNode constructor.  The node must not have
     * declared any of these parameters yet.
     */
    [[nodiscard]] static GeofenceConfig fromNode(rclcpp_lifecycle::LifecycleNode& node);

    /**
     * @brief Validate critical safety-config invariants (shared + spatial).
     *
     * Returns a list of human-readable error messages (empty == valid). The
     * node must refuse to configure (return FAILURE) when this is non-empty, so
     * a mis-configured safety geofence fails fast at startup instead of running
     * with silently-wrong values.
     */
    [[nodiscard]] std::vector<std::string> validate() const;
};

} // namespace rises
