#include "message_translator/area_locks_converter.hpp"
#include "message_translator/json_utils.hpp"

#include <rclcpp/logging.hpp>
#include <nlohmann/json.hpp>

#include <cinttypes>
#include <cstdint>
#include <functional>
#include <sstream>

namespace {

const rclcpp::Logger LOGGER = rclcpp::get_logger("area_locks_converter");

/**
 * Derive a deterministic positive int64 ID from AABB coordinates.
 *
 * The same bounding box always produces the same hash, so that a
 * lock("aabb": X) and unlock("aabb": X) reference the same area ID.
 */
int64_t hashAabb(float x0, float y0, float x1, float y1)
{
    std::ostringstream oss;
    oss << x0 << ',' << y0 << ',' << x1 << ',' << y1;
    const std::size_t h = std::hash<std::string>{}(oss.str());
    // Mask to positive int64 range.
    return static_cast<int64_t>(h & 0x7FFFFFFFFFFFFFFF);
}

} // anonymous namespace

namespace rises {

rises_interfaces::msg::AreaState
AreaLocksConverter::parse(const std::string& json_str, bool& ok)
{
    rises_interfaces::msg::AreaState msg;
    ok = false;

    nlohmann::json j;
    if (!JsonUtils::parse(json_str, j, LOGGER, "area_locks")) {
        return msg;
    }

    if (!JsonUtils::validateFields(j, {"aabb", "operation"}, LOGGER, "area_locks")) {
        return msg;
    }

    std::string operation;
    if (!JsonUtils::get(j, "operation", operation, LOGGER)) {
        return msg;
    }

    if (operation == "lock") {
        msg.operation = rises_interfaces::msg::AreaState::LOCK;
    } else if (operation == "unlock") {
        msg.operation = rises_interfaces::msg::AreaState::UNLOCK;
    } else {
        RCLCPP_WARN(LOGGER, "Unknown area_locks operation '%s'", operation.c_str());
        return msg;
    }

    const nlohmann::json& aabb_json = j["aabb"];
    if (!aabb_json.is_array() || aabb_json.size() != 2) {
        RCLCPP_WARN(LOGGER, "area_locks: 'aabb' must be an array of 2 points");
        return msg;
    }

    for (const nlohmann::json& point : aabb_json) {
        if (!point.is_array() || point.size() < 2) {
            RCLCPP_WARN(LOGGER, "area_locks: each AABB point must have at least 2 coordinates");
            return msg;
        }
    }

    const float x0 = aabb_json[0][0].get<float>();
    const float y0 = aabb_json[0][1].get<float>();
    const float x1 = aabb_json[1][0].get<float>();
    const float y1 = aabb_json[1][1].get<float>();

    msg.id = hashAabb(x0, y0, x1, y1);
    msg.x0 = static_cast<double>(std::min(x0, x1));
    msg.y0 = static_cast<double>(std::min(y0, y1));
    msg.x1 = static_cast<double>(std::max(x0, x1));
    msg.y1 = static_cast<double>(std::max(y0, y1));

    RCLCPP_DEBUG(LOGGER,
        "Parsed area_locks: op=%s aabb=[(%.3f,%.3f),(%.3f,%.3f)] -> id=%" PRId64,
        operation.c_str(), x0, y0, x1, y1, msg.id);

    ok = true;
    return msg;
}

} // namespace rises
