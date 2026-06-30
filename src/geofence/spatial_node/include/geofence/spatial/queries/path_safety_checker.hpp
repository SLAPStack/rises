#pragma once

// Project headers
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/spatial/queries/boundary_checker.hpp"
#include "spatial_index_selection.hpp"

// Third-party (ROS 2)
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Standard library
#include <optional>
#include <vector>

namespace rises::geofence {

/**
 * @brief Path safety checker for validating navigation paths (static policy class)
 * 
 * This class validates ROS nav_msgs::Path messages by checking each segment
 * between consecutive waypoints for collisions, clearance, and locked areas.
 * 
 * The checker expects paths to already be sampled at appropriate resolution
 * by the path planner. It validates each segment independently without
 * additional resampling.
 * 
 * Validation phases per segment:
 * 1. Map boundary check - endpoints within defined bounds
 * 2. Collision detection - segment doesn't intersect obstacles
 * 3. Clearance validation - minimum safety margin maintained
 * 4. Locked area check - endpoints not in restricted regions
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class PathSafetyChecker {
public:
    struct Config {
        float safety_margin;           // Minimum clearance from obstacles (meters)
        bool check_map_bounds;          // Validate path stays within map boundaries
        bool check_locked_areas;        // Reject paths through locked areas
        std::size_t min_path_poses;         // Minimum required waypoints (default: 2)
        std::size_t max_path_poses;         // Maximum allowed waypoints (default: 10000)
        
        // Default constructor
        constexpr Config() noexcept
            : safety_margin(0.1f)
            , check_map_bounds(true)
            , check_locked_areas(true)
            , min_path_poses(2)
            , max_path_poses(10000)
        {}
    };
    
    /**
     * @brief Initialize static configuration (must be called before use)
     * @param config Configuration parameters from ROS parameters
     */
    static void initialize(const Config& config);
    
    /**
     * @brief Get current static configuration
     * @return Current configuration
     */
    static Config getConfig() {
        Config config;
        config.safety_margin = safety_margin_;
        config.check_map_bounds = check_map_bounds_;
        config.check_locked_areas = check_locked_areas_;
        config.min_path_poses = min_path_poses_;
        config.max_path_poses = max_path_poses_;
        return config;
    }
    
    // ============================================================================
    // Path Safety Validation
    // ============================================================================
    
    /**
     * @brief Check if ROS path is safe to traverse
     * 
     * Validates each segment between consecutive waypoints in the path.
     * Iterates through poses and creates segments from pose[i] to pose[i+1],
     * checking each segment for collisions, clearance, and restrictions.
     * 
     * @param map The geofence map containing obstacles and areas
     * @param path ROS nav_msgs::Path message to validate
     * @return true if all segments are safe
     * 
     * @throws std::invalid_argument if path has too few or too many poses
     * 
     * @note Path must already be sampled at appropriate resolution
     * @note Checks segments sequentially, stops at first unsafe segment
     */
    [[nodiscard]] static bool isPathSafe(
        const GeofenceMap& map,
        const nav_msgs::msg::Path& path);
    
    /**
     * @brief Check if path segment is safe
     * 
     * Validates a single segment between two waypoints for:
     * - Map boundary containment
     * - Obstacle collisions
     * - Minimum clearance
     * - Locked area restrictions
     * 
     * @param map The geofence map to query
     * @param start Start point
     * @param end End point
     * @return true if segment passes all safety checks
     */
    [[nodiscard]] static bool isSegmentSafe(
        const GeofenceMap& map,
        const Point2D& start, 
        const Point2D& end);
    
private:
    // Static configuration
    static inline float safety_margin_ = 0.1f;
    static inline bool check_map_bounds_ = true;
    static inline bool check_locked_areas_ = true;
    static inline std::size_t min_path_poses_ = 2;
    static inline std::size_t max_path_poses_ = 10000;
    
    // Helper: Check if segment intersects any obstacle
    static bool doesSegmentIntersectObstacle(
        const GeofenceMap& map,
        const Point2D& start,
        const Point2D& end);
    
    // Helper: Check if segment intersects any locked area
    static bool doesSegmentIntersectLockedArea(
        const GeofenceMap& map,
        const Point2D& start,
        const Point2D& end);
    
    // Helper: Validate path size constraints
    static void validatePathSize(std::size_t num_poses);
};

} // namespace rises::geofence
