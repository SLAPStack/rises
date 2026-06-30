#pragma once

#include "geofence/spatial/map/geofence_map.hpp"
#include "spatial_index_selection.hpp"
#include "rises_interfaces/msg/obstacle.hpp"
#include <vector>

namespace rises::geofence {

/**
 * @brief Pure collision detection for obstacles (static policy class)
 * 
 * Checks if an obstacle message geometrically intersects/collides with
 * obstacles in the map. Uses spatial intersection algorithms.
 * 
 * Use cases:
 * - Validate if a planned obstacle placement would collide with existing ones
 * - Check if a detected obstacle overlaps with known obstacles
 * - Find all obstacles within collision margin of a given obstacle
 * 
 * Usage: Call initialize() once with config, then use static check methods.
 */
class ObstacleCollisionChecker {
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
     * @brief Check if obstacle collides with any map obstacle
     * 
     * @param map Geofence map containing known obstacles
     * @param obstacle Obstacle to check for collisions
     * @return true if obstacle intersects with any map obstacle
     */
    [[nodiscard]] static bool checkCollision(
        const GeofenceMap& map,
        const rises_interfaces::msg::Obstacle& obstacle);
    
    /**
     * @brief Find all map obstacles that collide with given obstacle
     * 
     * Returns IDs of all obstacles whose geometry intersects with
     * the provided obstacle (within collision margin).
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
