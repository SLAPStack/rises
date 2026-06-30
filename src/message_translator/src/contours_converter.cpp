#include "message_translator/contours_converter.hpp"
#include "message_translator/json_utils.hpp"

#include <rises_interfaces/msg/line_segment.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include <cstddef>
#include <set>
#include <utility>

namespace {

// Named logger shared across all functions in this translation unit.
const rclcpp::Logger LOGGER = rclcpp::get_logger("contours_converter");

// Hard caps on attacker-supplied JSON array lengths. The hull, outer segments,
// and per-inner-contour segment lists all come straight from JSON; an
// unbounded reserve on any of them exhausts heap before per-point validation
// runs. kMaxContourPoints applies to the hull, the outer-segments array, and
// each individual inner-contour ring; kMaxInnerPolygons caps the number of
// inner contours. Oversize inputs are rejected with a header-only Contours
// message (empty hull, empty segments, empty inner_contours).
constexpr std::size_t kMaxContourPoints = 50'000;
constexpr std::size_t kMaxInnerPolygons = 500;

/**
 * Convert the outer_contour_hull JSON array to a geometry_msgs Polygon.
 * Each element must be a 2-element [x, y] array; malformed entries are skipped.
 */
geometry_msgs::msg::Polygon buildConcaveHull(const nlohmann::json &hull_json) {
  geometry_msgs::msg::Polygon poly;
  if (hull_json.size() > kMaxContourPoints) {
    RCLCPP_ERROR(LOGGER,
                 "warehouse contours: hull rejected, size=%zu exceeds cap %zu",
                 hull_json.size(), kMaxContourPoints);
    return poly;
  }
  poly.points.reserve(hull_json.size());

  for (const nlohmann::json &point : hull_json) {
    if (!point.is_array() || point.size() != 2)
      continue;

    geometry_msgs::msg::Point32 pt;
    pt.x = point[0].get<float>();
    pt.y = point[1].get<float>();
    pt.z = 0.0f;
    poly.points.push_back(pt);
  }

  return poly;
}

/**
 * Parse outer_contour_segments into a LineSegment array and build the concave
 * hull polygon. Each segment entry must be [[x0,y0],[x1,y1]]; malformed entries
 * are skipped.
 */
void parseOuterContours(
    const nlohmann::json &j,
    std::vector<rises_interfaces::msg::LineSegment> &segments_out,
    geometry_msgs::msg::Polygon &hull_out) {
  segments_out.clear();

  const nlohmann::json &segs_json = j["outer_contour_segments"];
  if (segs_json.size() > kMaxContourPoints) {
    RCLCPP_ERROR(LOGGER,
                 "warehouse contours: outer_contour_segments rejected, "
                 "size=%zu exceeds cap %zu",
                 segs_json.size(), kMaxContourPoints);
    return;
  }
  segments_out.reserve(segs_json.size());

  for (const nlohmann::json &segment : segs_json) {
    if (!segment.is_array() || segment.size() != 2)
      continue;

    rises_interfaces::msg::LineSegment seg;
    seg.start.x = segment[0][0].get<float>();
    seg.start.y = segment[0][1].get<float>();
    seg.start.z = 0.0f;
    seg.end.x = segment[1][0].get<float>();
    seg.end.y = segment[1][1].get<float>();
    seg.end.z = 0.0f;
    segments_out.push_back(seg);
  }

  hull_out = buildConcaveHull(j["outer_contour_hull"]);
}

/**
 * Parse inner_contours into a vector of closed Polygons.
 *
 * Each inner contour is an array of 2-point segments. Only the start point of
 * each segment is added; the final segment's end point is then appended to
 * explicitly close the polygon ring.
 */
void parseInnerContours(
    const nlohmann::json &j,
    std::vector<geometry_msgs::msg::Polygon> &inner_polygons_out) {
  inner_polygons_out.clear();

  const nlohmann::json &inners_json = j["inner_contours"];
  if (inners_json.size() > kMaxInnerPolygons) {
    RCLCPP_ERROR(LOGGER,
                 "warehouse contours: inner_contours rejected, "
                 "size=%zu exceeds cap %zu",
                 inners_json.size(), kMaxInnerPolygons);
    return;
  }

  for (const nlohmann::json &inner : inners_json) {
    if (!inner.is_array())
      continue;

    if (inner.size() > kMaxContourPoints) {
      RCLCPP_ERROR(LOGGER,
                   "warehouse contours: inner ring rejected, size=%zu "
                   "exceeds cap %zu",
                   inner.size(), kMaxContourPoints);
      inner_polygons_out.clear();
      return;
    }

    geometry_msgs::msg::Polygon poly;
    poly.points.reserve(inner.size() + 1);

    for (const nlohmann::json &segment : inner) {
      if (!segment.is_array() || segment.size() != 2)
        continue;

      geometry_msgs::msg::Point32 p;
      p.x = segment[0][0].get<float>();
      p.y = segment[0][1].get<float>();
      p.z = 0.0f;
      poly.points.push_back(p);
    }

    // Close the polygon ring by appending the last segment's end point.
    if (!inner.empty()) {
      const nlohmann::json &last = inner.back();
      geometry_msgs::msg::Point32 p_close;
      p_close.x = last[1][0].get<float>();
      p_close.y = last[1][1].get<float>();
      p_close.z = 0.0f;
      poly.points.push_back(p_close);
    }

    inner_polygons_out.push_back(std::move(poly));
  }
}

} // anonymous namespace

namespace rises {

rises_interfaces::msg::Contours
ContoursConverter::parse(const std::string &json_str,
                         const std::string &frame_id) {
  rises_interfaces::msg::Contours contours;
  contours.header.frame_id = frame_id;

  nlohmann::json j;
  if (!JsonUtils::parse(json_str, j, LOGGER, "warehouse contours")) {
    return contours;
  }

  // Accept both field naming conventions:
  //   New format: outer_contour_segments + outer_contour_hull + inner_contours
  //   Old format: outer_contour + inner_contours (hull derived from segments)
  const bool has_new_format = j.contains("outer_contour_segments");
  const bool has_old_format = j.contains("outer_contour");

  if (!has_new_format && !has_old_format) {
    RCLCPP_WARN(LOGGER, "warehouse contours: no outer_contour_segments or "
                        "outer_contour field found");
    return contours;
  }
  if (!j.contains("inner_contours")) {
    RCLCPP_WARN(LOGGER, "warehouse contours: missing inner_contours field");
    return contours;
  }

  // Normalize: if old format, rename outer_contour -> outer_contour_segments
  nlohmann::json normalized = j;
  if (!has_new_format && has_old_format) {
    normalized["outer_contour_segments"] = normalized["outer_contour"];
  }

  // Generate hull from segment endpoints if not provided
  if (!normalized.contains("outer_contour_hull")) {
    nlohmann::json hull = nlohmann::json::array();
    std::set<std::pair<float, float>> seen;
    for (const nlohmann::json &seg : normalized["outer_contour_segments"]) {
      if (!seg.is_array() || seg.size() != 2)
        continue;
      float x0 = seg[0][0].get<float>(), y0 = seg[0][1].get<float>();
      float x1 = seg[1][0].get<float>(), y1 = seg[1][1].get<float>();
      if (seen.emplace(x0, y0).second)
        hull.push_back({x0, y0});
      if (seen.emplace(x1, y1).second)
        hull.push_back({x1, y1});
    }
    if (!hull.empty())
      hull.push_back(hull[0]); // close the ring
    normalized["outer_contour_hull"] = hull;
  }

  try {
    parseOuterContours(normalized, contours.outer_contour_segments,
                       contours.outer_contour_hull);
    parseInnerContours(j, contours.inner_contours);

    RCLCPP_DEBUG(LOGGER,
                 "Parsed contours: %zu outer segments, %zu hull points, %zu "
                 "inner polygons",
                 contours.outer_contour_segments.size(),
                 contours.outer_contour_hull.points.size(),
                 contours.inner_contours.size());

  } catch (const std::exception &e) {
    RCLCPP_ERROR(LOGGER, "Error processing warehouse contours geometry: %s",
                 e.what());
  }

  return contours;
}

} // namespace rises
