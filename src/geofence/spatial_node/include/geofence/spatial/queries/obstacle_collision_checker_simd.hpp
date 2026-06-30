#pragma once

#include "geofence/spatial/map/geofence_map.hpp"
#include "spatial_index_selection.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <vector>

namespace rises::geofence {

/**
 * @brief SIMD-optimized collision detection for obstacles (static policy class)
 * 
 * Provides the same interface as ObstacleCollisionChecker but uses SIMD
 * instructions for vectorized geometry intersection checks.
 * 
 * This implementation is selected at compile-time when USE_SIMD
 * is defined in the CMake build configuration.
 * 
 * Performance: ~2-4x faster than standard checker for batch operations
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class ObstacleCollisionCheckerSIMD {
public:
    struct Config {
        float collision_margin = 0.05f;  // 5cm safety margin for collision checks
    };
    
    /**
     * @brief Initialize static configuration (must be called before use)
     * @param config Configuration parameters from ROS parameters
     */
    static void initialize(const Config& config);
    
    /**
     * @brief Check if obstacle collides with any map obstacle (SIMD-optimized)
     * 
     * @param map Geofence map containing known obstacles
     * @param obstacle Obstacle to check for collisions
     * @return true if obstacle intersects with any map obstacle
     */
    [[nodiscard]] static bool checkCollision(
        const GeofenceMap& map,
        const rises_interfaces::msg::Obstacle& obstacle);
    
    /**
     * @brief Find all map obstacles that collide with given obstacle (SIMD-optimized)
     * 
     * Returns IDs of all obstacles whose geometry intersects with
     * the provided obstacle (within collision margin).
     * 
     * Uses SIMD for bounding box checks and batch intersection tests.
     * 
     * @param map Geofence map containing known obstacles
     * @param obstacle Obstacle to check
     * @return Vector of obstacle IDs that collide
     */
    [[nodiscard]] static std::vector<int64_t> findCollidingObstacles(
        const GeofenceMap& map,
        const rises_interfaces::msg::Obstacle& obstacle);

private:
    // Static configuration
    static inline float collision_margin_ = 0.05f;
};

} // namespace rises::geofence
