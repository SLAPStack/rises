#pragma once

// Project headers
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/common/policies/coordinate_transform.hpp"
#include "geofence/spatial/visualization/geofence_visualizer.hpp"
#include "spatial_index_selection.hpp"

// Third-party (ROS 2)
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rises
{

/**
 * @brief Processes map obstacle inserts/removes for both the topic callback
 *        and the UpdateMap service.
 *
 * The insert/remove logic was duplicated between updatesCallback() and
 * updateMapService().  This class provides a single applyUpdates() that
 * both call sites delegate to.
 */
class MapUpdateHandler
{
public:
    struct Stats {
        int32_t added = 0;
        int32_t removed = 0;
        int32_t outside_contours = 0;
        int32_t remove_not_found = 0;
    };

    /**
     * @brief Apply a batch of obstacle updates to the map.
     *
     * Handles coordinate transformation, contour-inside checks, sample
     * logging, and optional visualizer updates.
     *
     * @param updates     The update array (inserts and deletes).
     * @param map         The geofence map to modify.
     * @param visualizer  Optional visualizer (may be nullptr).
     * @param logger      Logger for diagnostics.
     * @param source_tag  Log prefix tag ("TOPIC" or "SERVICE").
     * @return Statistics about the applied updates.
     */
    [[nodiscard]] static Stats applyUpdates(
        const rises_interfaces::msg::ObstacleUpdateArray& updates,
        rises::geofence::GeofenceMap& map,
        const std::shared_ptr<GeofenceVisualizer>& visualizer,
        rclcpp::Logger logger,
        const std::string& source_tag = "TOPIC");

    /**
     * @brief Parse a Contours message into MapBoundaryContours.
     *
     * Shared between mapBoundaryCallback and updateWarehouseLayoutService.
     */
    [[nodiscard]] static rises::shape::MapBoundaryContours parseContours(
        const rises_interfaces::msg::Contours& contours_msg);
};

} // namespace rises
