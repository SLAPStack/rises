#pragma once

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "spatial_index_selection.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <vector>

// Use Boost.Geometry backend types
// Direct usage of rises::geofence types (formerly backend::boost)

namespace rises::geofence {

/**
 * @brief Correspondence matching for obstacles (data association) - static policy class
 * 
 * Checks if a detected obstacle "is part of" or "corresponds to" a known
 * map obstacle. This is NOT collision detection - it's about identifying
 * if a detected geometric primitive (point/line) represents a portion of
 * a larger known obstacle.
 * 
 * Examples:
 * - Detected point on edge of known rectangle → match
 * - Detected line segment contained in known line → match
 * - Detected line on edge of known polygon → match
 * 
 * Use cases:
 * - Data association: "Is this sensor reading from a known obstacle?"
 * - Sensor validation: "Is this detection consistent with the map?"
 * - Unknown obstacle detection: "Is this something new in the environment?"
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class ObstacleCorrespondenceMatcher {
public:
    struct Config {
        float position_tolerance = 0.10f;           // 10cm position tolerance
        float size_tolerance = 0.15f;               // 15% size tolerance
        float angle_tolerance = 0.1745f;            // ~10 degrees
        float point_on_line_tolerance = 0.05f;      // 5cm tolerance for point-on-line
        float line_containment_tolerance = 0.03f;   // 3cm for line-in-line matching
    };
    
    struct MatchResult {
        bool found_match;
        int64_t matched_id;
        float confidence;  // 0.0 to 1.0, based on quality of match
    };
    
    /**
     * @brief Initialize static configuration (must be called before use)
     * @param config Configuration parameters from ROS parameters
     */
    static void initialize(const Config& config);
    
    /**
     * @brief Find if detected obstacle corresponds to a known map obstacle
     * 
     * Performs data association - checks if this detection is "part of" any
     * known obstacle in the map. Uses robot safety profile for zone filtering.
     * 
     * @param map Geofence map containing known obstacles
     * @param detected_obstacle Obstacle from perception/sensors
     * @param robot_position Current robot position for safety zone filtering
     * @param robot_heading_rad Robot heading in radians (for oriented zones)
     * @param profile Robot safety profile defining detection zones
     * @return Match result with ID and confidence if found
     */
    [[nodiscard]] static MatchResult findCorrespondingObstacle(
        const GeofenceMap& map,
        const rises_interfaces::msg::Obstacle& detected_obstacle,
        const Point2D& robot_position,
        const float robot_heading_rad,
        const RobotSafetyProfile& profile);

private:
    // Static configuration
    static inline float position_tolerance_ = 0.10f;
    static inline float size_tolerance_ = 0.15f;
    static inline float angle_tolerance_ = 0.1745f;
    static inline float point_on_line_tolerance_ = 0.05f;
    static inline float line_containment_tolerance_ = 0.03f;
    
    // Core matching logic: check if detected is "part of" map obstacle
    static bool obstaclesMatch(
        const Geometry& map_obstacle,
        const rises_interfaces::msg::Obstacle& detected);
    
    // Geometric containment checks
    static bool isPointOnRectangleEdge(
        const Point2D& point,
        const rises::geofence::Rectangle& rect);
    
    static bool isPointOnPolygonEdge(
        const Point2D& point,
        const rises::geofence::Polygon& polygon);
    
    static bool isPointOnLineSegment(
        const Point2D& point,
        const rises::geofence::Line& line);
    
    static bool isLineContainedInLine(
        const rises::geofence::Line& smaller_line,
        const rises::geofence::Line& larger_line);
    
    static bool isLineOnRectangleEdge(
        const rises::geofence::Line& line,
        const rises::geofence::Rectangle& rect);
    
    static bool isLineOnPolygonEdge(
        const rises::geofence::Line& line,
        const rises::geofence::Polygon& polygon);
    
    // Compute match quality score
    static float computeMatchConfidence(
        const Geometry& map_obstacle,
        const rises_interfaces::msg::Obstacle& detected);
};

} // namespace rises::geofence
