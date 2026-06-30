#pragma once

// Project headers
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/shape/contour.hpp"
#include "spatial_index_selection.hpp"

// Standard library
#include <optional>
#include <vector>

namespace rises::geofence {

/**
 * @brief High-level obstacle matching for laser scan segments (static policy class)
 * 
 * Determines if laser scan segments correspond to:
 * 1. Known obstacles (stored in GeofenceMap)
 * 2. Map boundary walls (stored in MapBoundaryContours)
 * 3. Unknown obstacles (no match -> alert!)
 * 
 * This is the POLICY layer - it decides what constitutes a "match"
 * and what to do with unmatched scans.
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class ObstacleMatchChecker {
public:
    struct Config {
        float position_tolerance;   // How close scan must be to edge (meters)
        float angular_tolerance;    // How parallel scan must be to edge (radians)
        bool match_obstacles;       // Check against obstacle map
        bool match_warehouse;       // Check against warehouse contours
        
        constexpr Config() noexcept
            : position_tolerance(0.05f)     // 5cm
            , angular_tolerance(0.1f)       // ~5.7 degrees
            , match_obstacles(true)
            , match_warehouse(true)
        {}
    };
    
    enum class MatchType {
        OBSTACLE,       // Scan matches known obstacle
        WAREHOUSE,      // Scan matches warehouse wall/contour
        UNKNOWN         // Scan doesn't match anything (new obstacle!)
    };
    
    struct MatchResult {
        MatchType type;
        int64_t obstacle_id;  // Valid if type == OBSTACLE
        
        [[nodiscard]] constexpr bool isUnknown() const noexcept { return type == MatchType::UNKNOWN; }
        [[nodiscard]] constexpr bool isKnown() const noexcept { return type != MatchType::UNKNOWN; }
    };
    
    /**
     * @brief Initialize static configuration (must be called before use)
     * @param config Configuration parameters from ROS parameters
     */
    static void initialize(const Config& config);
    
    // ============================================================================
    // High-Level Matching (Policy)
    // ============================================================================
    
    /**
     * @brief Check what a laser scan segment matches
     * @param map Obstacle map to query
     * @param scan_start Start of laser scan segment
     * @param scan_end End of laser scan segment
     * @return Match result indicating what scan corresponds to
     */
    [[nodiscard]] static MatchResult matchScanSegment(
        const GeofenceMap& map,
        const Point2D& scan_start,
        const Point2D& scan_end);
    
    /**
     * @brief Check multiple scan segments, return unmatched ones
     * @param map Obstacle map
     * @param scan_segments Vector of [start, end] pairs
     * @return Indices of segments that didn't match anything (unknown obstacles)
     */
    [[nodiscard]] static std::vector<std::size_t> findUnmatchedScans(
        const GeofenceMap& map,
        const std::vector<std::pair<Point2D, Point2D>>& scan_segments);
    
    /**
     * @brief Match a detected obstacle against map obstacles
     * @param map Geofence map containing known obstacles
     * @param detected_obstacle Detected obstacle from laser scan
     * @return Match result indicating what the detected obstacle corresponds to
     */
    [[nodiscard]] static MatchResult matchDetectedObstacle(
        const GeofenceMap& map,
        const rises_interfaces::msg::Obstacle& detected_obstacle);
    
    /**
     * @brief Get scan segments that match obstacles (for visualization)
     * @param map Obstacle map
     * @param scan_segments All scan segments
     * @return Pairs of [segment_index, obstacle_id]
     */
    [[nodiscard]] static std::vector<std::pair<std::size_t, int64_t>> getObstacleMatches(
        const GeofenceMap& map,
        const std::vector<std::pair<Point2D, Point2D>>& scan_segments);
    
private:
    // Static configuration
    static inline float position_tolerance_ = 0.05f;
    static inline float angular_tolerance_ = 0.1f;
    static inline bool match_obstacles_ = true;
    static inline bool match_warehouse_ = true;
    
    // Helper: Check if scan matches any obstacle in map
    static bool matchesAnyObstacle(
        const GeofenceMap& map,
        const Point2D& scan_start,
        const Point2D& scan_end,
        int64_t* matched_id = nullptr);
    
    // Helper: Check if scan line matches a geometry shape
    static bool matchesScanToShape(
        const rises::geofence::Line& scan_line,
        const Geometry& shape,
        float eps_pos,
        float eps_angle);
    
    // Geometry-specific matching helpers
    static bool matchScanToLine(
        const rises::geofence::Line& scan,
        const rises::geofence::Line& edge,
        float eps_pos,
        float eps_angle);
    
    static bool matchScanToRectangle(
        const rises::geofence::Line& scan,
        const rises::geofence::Rectangle& rect,
        float eps_pos,
        float eps_angle);
    
    static bool matchScanToPolygon(
        const rises::geofence::Line& scan,
        const rises::geofence::Polygon& poly,
        float eps_pos,
        float eps_angle);
    
    /**
     * @brief Check if detected obstacle geometry matches map obstacle geometry
     * @param detected_geom Geometry of detected obstacle
     * @param map_geom Geometry of map obstacle
     * @param eps_pos Position tolerance
     * @param eps_angle Angular tolerance
     * @return true if obstacles match
     */
    static bool matchObstacleGeometries(
        const Geometry& detected_geom,
        const Geometry& map_geom,
        float eps_pos,
        float eps_angle);
    
    // Map boundary matching
    static bool matchesWarehouse(
        const shape::MapBoundaryContours& contours,
        const Point2D& scan_start,
        const Point2D& scan_end);

    // Point-to-contour proximity check (no angle matching, for single points)
    static bool isPointNearContourEdge(
        const shape::MapBoundaryContours& contours,
        const Point2D& point,
        double tolerance);
};

} // namespace rises::geofence
