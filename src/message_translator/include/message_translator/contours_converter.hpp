#pragma once

#include <rises_interfaces/msg/contours.hpp>
#include <string>

namespace rises {

/**
 * Stateless converter from the Rises warehouse-contours JSON format to a ROS2 Contours message.
 *
 * The format encodes the full warehouse boundary as outer line segments, a concave
 * hull polygon, and a list of inner obstacle polygons (columns, walls, etc.).
 * All methods are static; the class cannot be instantiated.
 */
class ContoursConverter
{
public:
    ContoursConverter() = delete;

    /**
     * Parse a warehouse contours JSON string into a ROS2 Contours message.
     *
     * Expects the root JSON object to contain:
     *   - "outer_contour_segments": array of 2-point line segments
     *   - "outer_contour_hull":     array of [x, y] hull points
     *   - "inner_contours":         array of per-obstacle segment arrays
     *
     * Returns a Contours message with only the header set when parsing fails
     * or required keys are absent. Callers may check whether the returned
     * message carries any geometry before publishing.
     *
     * @param json_str  Raw JSON string of the warehouse layout.
     * @param frame_id  Coordinate frame written into the Contours header.
     * @return          Populated Contours message, or header-only on failure.
     */
    [[nodiscard]] static rises_interfaces::msg::Contours parse(
        const std::string& json_str,
        const std::string& frame_id);
};

} // namespace rises
