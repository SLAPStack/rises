#pragma once

#include <rises_interfaces/msg/area_state.hpp>
#include <string>

namespace rises {

/**
 * Stateless converter from the SLAPStack area-locks JSON format to a ROS2 AreaState message.
 *
 * The JSON payload carries an AABB bounding box and a lock/unlock operation.
 * A deterministic area ID is derived from the AABB coordinates so that the
 * same spatial region always maps to the same ID across lock/unlock pairs.
 * All methods are static; the class cannot be instantiated.
 */
class AreaLocksConverter
{
public:
    AreaLocksConverter() = delete;

    /**
     * Parse an area-lock JSON string into a ROS2 AreaState message.
     *
     * Expects the root JSON object to contain:
     *   - "aabb":      [[x0, y0], [x1, y1]]  (two-point bounding box)
     *   - "operation":  "lock" or "unlock"
     *
     * The area ID is computed as a positive int64 hash of the canonical AABB
     * coordinate string "x0,y0,x1,y1", ensuring the same bounding box always
     * produces the same ID for matching lock/unlock calls.
     *
     * @param json_str  Raw JSON string of the area-lock message.
     * @param[out] ok   Set to true on successful parse, false on failure.
     * @return          Populated AreaState message. Only valid when ok is true.
     */
    static rises_interfaces::msg::AreaState parse(
        const std::string& json_str, bool& ok);
};

} // namespace rises
