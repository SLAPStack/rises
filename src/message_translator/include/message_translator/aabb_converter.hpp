#pragma once

#include <rises_interfaces/msg/obstacle.hpp>
#include <rises_interfaces/msg/obstacle_update.hpp>
#include <string>
#include <vector>

namespace rises {

/**
 * Stateless converter from the Rises AABB obstacle-update JSON format to ROS2 typed messages.
 *
 * This format is Rises-internal (not VDA5050). Each entry describes an axis-aligned
 * bounding box with "aabb", "id", and "operation" fields. All methods are static;
 * the class cannot be instantiated.
 */
class AabbConverter
{
public:
    AabbConverter() = delete;

    /**
     * Parse an obstacle-update JSON string into a list of ObstacleUpdate messages.
     *
     * Accepts three input forms without separate pre-processing by the caller:
     *   - JSON array:              [ {...}, {...} ]
     *   - Wrapped pallet object:   { "pallets": [ {...}, {...} ] }
     *   - Single JSON object:      { "aabb": ..., "id": ..., "operation": ... }
     *
     * Items with missing or malformed fields are skipped and logged individually;
     * the returned vector contains only successfully parsed updates.
     *
     * @param json_str  Raw JSON string of the update message.
     * @return          Parsed updates, or an empty vector on top-level JSON failure.
     */
    [[nodiscard]] static std::vector<rises_interfaces::msg::ObstacleUpdate> parseObstacleUpdates(
        const std::string& json_str);

    /**
     * Parse a single validation obstacle from a JSON string.
     *
     * Expects the JSON to contain "aabb" and "id" fields. The caller should
     * check that obs.type != 0 before use, since a default-constructed Obstacle
     * (type == 0) is returned on any parse or validation failure.
     *
     * @param json_str  Raw JSON string describing the validation obstacle.
     * @return          Parsed Obstacle, or a default-constructed one on failure.
     */
    static rises_interfaces::msg::Obstacle parseValidationObstacle(
        const std::string& json_str);

    /**
     * Build a RECTANGLE Obstacle from a two-point AABB coordinate pair.
     *
     * Input: [ [x0, y0, z0?], [x1, y1, z1?] ]. Centre, width, height, and
     * 3D extent (minor_axis) are derived from the two corner points.
     * Returns a default-constructed Obstacle (type == 0) when the AABB is malformed.
     *
     * @param aabb  Two-element vector of 2- or 3-element float coordinate vectors.
     * @param id    Numeric obstacle identifier.
     */
    static rises_interfaces::msg::Obstacle createRectangleObstacle(
        const std::vector<std::vector<float>>& aabb, int64_t id);
};

} // namespace rises
