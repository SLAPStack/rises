#pragma once

#include <cstdint>

namespace rises::geofence {

/**
 * @brief Obstacle layer classification for geofence map
 * 
 * Categorizes obstacles by their role and persistence characteristics,
 * allowing policies to filter which layers are relevant for specific checks.
 * 
 * Layer selection by use case:
 * - Path planning: FIXED | STATIC (ignore transient robots)
 * - Collision avoidance: FIXED | STATIC | DYNAMIC (check everything)
 * - Robot tracking: DYNAMIC only (other robots)
 * - Map updates: STATIC | FIXED (persistent changes)
 */
enum class ObstacleLayer : uint8_t {
    /**
     * @brief Fixed infrastructure (walls, permanent boundaries)
     * 
     * Characteristics:
     * - Never moves or changes
     * - Defines workspace boundaries
     * - Loaded once at startup
     * - Examples: building walls, fence perimeter, restricted zones
     */
    FIXED = 0x01,
    
    /**
     * @brief Static map obstacles (furniture, equipment)
     * 
     * Characteristics:
     * - Persistent but can be updated
     * - Part of environment model
     * - Changes infrequently
     * - Examples: tables, shelves, parked equipment
     */
    STATIC = 0x02,
    
    /**
     * @brief Dynamic obstacles (robots, people, moving objects)
     * 
     * Characteristics:
     * - Transient with TTL (time-to-live)
     * - Frequently updated from sensors
     * - Auto-expires after timeout
     * - Examples: other robots, pedestrians, forklifts
     */
    DYNAMIC = 0x04,
    
    /**
     * @brief All layers combined (bitmask)
     */
    ALL = FIXED | STATIC | DYNAMIC
};

/**
 * @brief Bitwise OR for layer masks
 */
constexpr ObstacleLayer operator|(const ObstacleLayer lhs, const ObstacleLayer rhs) {
    return static_cast<ObstacleLayer>(
        static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs)
    );
}

/**
 * @brief Bitwise AND for layer mask testing
 */
constexpr ObstacleLayer operator&(const ObstacleLayer lhs, const ObstacleLayer rhs) {
    return static_cast<ObstacleLayer>(
        static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs)
    );
}

/**
 * @brief Check if layer mask contains specific layer
 */
constexpr bool hasLayer(const ObstacleLayer mask, const ObstacleLayer layer) {
    return (static_cast<uint8_t>(mask) & static_cast<uint8_t>(layer)) != 0;
}

} // namespace rises::geofence
