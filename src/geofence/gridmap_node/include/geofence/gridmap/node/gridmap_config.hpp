#pragma once

// Project headers
#include "geofence/common/node/geofence_common_config.hpp"

// Third-party (ROS 2)
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rises
{

/**
 * @brief All ROS parameters for the GeofenceGridmapNode, loaded once in the
 *        constructor.
 *
 * Derives from GeofenceCommonConfig (the parameters shared with every geofence
 * node) and adds the six occupancy-grid-specific fields. Node code accesses
 * every field -- shared or grid -- directly through this object.
 *
 * NOTE: publish_report_always defaults to FALSE for the gridmap node (the
 * spatial node defaults it true); fromNode() passes that default explicitly.
 */
struct GridmapConfig : GeofenceCommonConfig
{
    // Occupancy-grid configuration.
    double grid_resolution = 0.05;
    double grid_width_meters = 100.0;
    double grid_height_meters = 100.0;
    double grid_origin_x = -50.0;
    double grid_origin_y = -50.0;
    float obstacle_inflation_radius = 0.0f;

    /**
     * @brief Declare all parameters on the node and read their values.
     *
     * Call once in the GeofenceGridmapNode constructor. The node must not have
     * declared any of these parameters yet.
     */
    [[nodiscard]] static GridmapConfig fromNode(rclcpp_lifecycle::LifecycleNode& node);
};

} // namespace rises
