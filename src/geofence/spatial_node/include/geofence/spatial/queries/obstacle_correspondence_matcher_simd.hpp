#pragma once

#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/policies/robot_safety_profile.hpp"
#include "spatial_index_selection.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <vector>

namespace rises::geofence {

/**
 * @brief SIMD-optimized correspondence matching for obstacles - static policy class
 * 
 * Provides the same interface as ObstacleCorrespondenceMatcher but uses SIMD
 * instructions for vectorized geometric computations.
 * 
 * This implementation is selected at compile-time when USE_SIMD
 * is defined in the CMake build configuration.
 * 
 * Performance: ~2-3x faster for batch point-on-edge checks and polygon matching
 * 
 * Checks if a detected obstacle "is part of" or "corresponds to" a known
 * map obstacle using vectorized distance and tolerance comparisons.
 * 
 * Examples:
 * - Detected point on edge of known rectangle → match
 * - Detected line segment contained in known line → match
 * - Detected line on edge of known polygon → match
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class ObstacleCorrespondenceMatcherSIMD {
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
     * @brief Find if detected obstacle corresponds to a known map obstacle (SIMD-optimized)
     * 
     * Performs data association using vectorized computations for distance checks
     * and geometric containment tests. Uses robot safety profile for zone filtering.
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
    
    // Geometric containment checks (SIMD-optimized)
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
