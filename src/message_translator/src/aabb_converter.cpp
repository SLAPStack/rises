#include "message_translator/aabb_converter.hpp"
#include "message_translator/json_utils.hpp"

#include <rises_interfaces/msg/obstacle_update.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include <cinttypes>
#include <cmath>
#include <cstddef>

namespace {

// Named logger shared across all functions in this translation unit.
const rclcpp::Logger LOGGER = rclcpp::get_logger("aabb_converter");

// Hard cap on obstacle-update batch size. The "pallets" array (or top-level
// array) comes straight from attacker-supplied JSON; an unbounded reserve on
// the item count exhausts heap before per-item validation runs. Oversize
// payloads are rejected with an empty update vector.
constexpr std::size_t kMaxAabbObstacles = 100'000;

/**
 * Normalise the top-level JSON value into a flat list of item objects.
 *
 * Accepts: a JSON array, an object with a "pallets" key, or a single object.
 * Returns an empty vector for any other structure.
 */
std::vector<nlohmann::json> normaliseItems(const nlohmann::json &j) {
  if (j.is_array()) {
    return j.get<std::vector<nlohmann::json>>();
  }
  if (j.is_object() && j.contains("pallets") && j["pallets"].is_array()) {
    RCLCPP_INFO(LOGGER, "Processing %zu pallets from obstacle update",
                j["pallets"].size());
    return j["pallets"].get<std::vector<nlohmann::json>>();
  }
  if (j.is_object()) {
    return {j}; // Single-item fallback.
  }
  return {};
}

/**
 * Parse the "aabb" field of an obstacle item into a nested float vector.
 *
 * Expects exactly two points, each with at least two coordinates.
 * Returns an empty vector when the format is invalid so that the caller
 * can fall back to a placeholder obstacle.
 */
std::vector<std::vector<float>> parseAabb(const nlohmann::json &aabb_json) {
  if (!aabb_json.is_array() || aabb_json.size() != 2)
    return {};

  std::vector<std::vector<float>> aabb;
  aabb.reserve(2);

  for (const nlohmann::json &point : aabb_json) {
    if (!point.is_array() || point.size() < 2)
      return {};

    std::vector<float> coords;
    coords.push_back(point[0].get<float>());
    coords.push_back(point[1].get<float>());
    if (point.size() >= 3) {
      coords.push_back(point[2].get<float>());
    }
    aabb.push_back(std::move(coords));
  }

  return aabb;
}

} // anonymous namespace

namespace rises {

std::vector<rises_interfaces::msg::ObstacleUpdate>
AabbConverter::parseObstacleUpdates(const std::string &json_str) {
  std::vector<rises_interfaces::msg::ObstacleUpdate> updates;

  nlohmann::json j;
  if (!JsonUtils::parse(json_str, j, LOGGER, "obstacle updates")) {
    return updates;
  }

  const std::vector<nlohmann::json> items = normaliseItems(j);
  if (items.empty()) {
    RCLCPP_WARN(LOGGER, "Obstacle update JSON contains no processable items");
    return updates;
  }

  if (items.size() > kMaxAabbObstacles) {
    RCLCPP_ERROR(LOGGER,
                 "Obstacle update rejected: items.size()=%zu exceeds cap %zu",
                 items.size(), kMaxAabbObstacles);
    return updates;
  }

  updates.reserve(items.size());

  for (const nlohmann::json &item : items) {
    if (!JsonUtils::validateFields(item, {"id", "operation"}, LOGGER,
                                   "obstacle item")) {
      continue;
    }

    int64_t id = 0;
    std::string operation;
    if (!JsonUtils::get(item, "id", id, LOGGER) ||
        !JsonUtils::get(item, "operation", operation, LOGGER)) {
      continue;
    }

    rises_interfaces::msg::ObstacleUpdate update;

    if (operation == "INSERT") {
      update.operation = rises_interfaces::msg::ObstacleUpdate::OP_INSERT;
    } else if (operation == "DELETE") {
      update.operation = rises_interfaces::msg::ObstacleUpdate::OP_DELETE;
    } else {
      RCLCPP_WARN(LOGGER, "Unknown operation '%s' for obstacle %" PRId64,
                  operation.c_str(), id);
      continue;
    }

    if (update.operation == rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
      // DELETE only needs the ID — no geometry required.
      update.obstacle.id = id;
    } else if (item.contains("aabb")) {
      const std::vector<std::vector<float>> aabb = parseAabb(item["aabb"]);
      if (aabb.size() == 2) {
        update.obstacle = createRectangleObstacle(aabb, id);
      } else {
        update.obstacle.id = id;
        update.obstacle.type = rises_interfaces::msg::Obstacle::CIRCLE;
        update.obstacle.radius = 0.01f;
        RCLCPP_WARN(LOGGER,
                    "Malformed AABB for obstacle %" PRId64
                    "; using placeholder circle",
                    id);
      }
    } else {
      RCLCPP_WARN(LOGGER,
                  "INSERT for obstacle %" PRId64
                  " missing required 'aabb' field; skipping",
                  id);
      continue;
    }

    RCLCPP_DEBUG(LOGGER,
                 "Parsed obstacle update id=%" PRId64
                 " op=%s type=%u at (%.3f, %.3f)",
                 id, operation.c_str(), update.obstacle.type,
                 update.obstacle.position.x, update.obstacle.position.y);

    updates.push_back(std::move(update));
  }

  return updates;
}

rises_interfaces::msg::Obstacle
AabbConverter::parseValidationObstacle(const std::string &json_str) {
  rises_interfaces::msg::Obstacle obs;

  nlohmann::json j;
  if (!JsonUtils::parse(json_str, j, LOGGER, "validation obstacle")) {
    return obs;
  }

  if (!JsonUtils::validateFields(j, {"aabb", "id"}, LOGGER,
                                 "validation obstacle")) {
    return obs;
  }

  int64_t id = 0;
  if (!JsonUtils::get(j, "id", id, LOGGER)) {
    return obs;
  }

  std::vector<std::vector<float>> aabb;
  try {
    aabb = j["aabb"].get<std::vector<std::vector<float>>>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER,
                 "Failed to parse AABB for validation obstacle %" PRId64 ": %s",
                 id, e.what());
    return obs;
  }

  return createRectangleObstacle(aabb, id);
}

rises_interfaces::msg::Obstacle AabbConverter::createRectangleObstacle(
    const std::vector<std::vector<float>> &aabb, int64_t id) {
  rises_interfaces::msg::Obstacle obs;

  if (aabb.size() != 2 || aabb[0].size() < 2 || aabb[1].size() < 2) {
    RCLCPP_WARN(LOGGER, "Invalid AABB format for obstacle %" PRId64, id);
    return obs;
  }

  const float x0 = aabb[0][0];
  const float y0 = aabb[0][1];
  const float z0 = aabb[0].size() >= 3 ? aabb[0][2] : 0.0f;
  const float x1 = aabb[1][0];
  const float y1 = aabb[1][1];
  const float z1 = aabb[1].size() >= 3 ? aabb[1][2] : 0.0f;

  obs.id = id;
  obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
  obs.position.x = (x0 + x1) / 2.0f;
  obs.position.y = (y0 + y1) / 2.0f;
  obs.position.z = (z0 + z1) / 2.0f;
  obs.width = std::abs(x1 - x0);
  obs.height = std::abs(y1 - y0);
  obs.orientation = 0.0f;
  // minor_axis stores the 3D extent for downstream visualisation (RViz marker
  // scale.z).
  obs.minor_axis = std::abs(z1 - z0);

  RCLCPP_DEBUG(LOGGER,
               "Created RECTANGLE obstacle id=%" PRId64
               ": center=(%.3f, %.3f, %.3f), size=(%.3f x %.3f x %.3f)",
               id, obs.position.x, obs.position.y, obs.position.z, obs.width,
               obs.height, obs.minor_axis);

  return obs;
}

} // namespace rises
