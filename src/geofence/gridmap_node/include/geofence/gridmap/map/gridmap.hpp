#pragma once

#include "geofence/gridmap/map/gridmap_data.hpp"
#include "geofence/gridmap/policies/gridmap_policies.hpp"
#include "geofence/utils/rcu.hpp"
#include <memory>

namespace rises {
namespace geofence {

/**
 * @brief Thread-safe occupancy grid map using RCU for lock-free reads
 * 
 * Architecture:
 * - GridMapData: Immutable data class with query methods
 * - Policy classes: Modify grid data during copy-on-write updates
 * - RCU snapshots: Lock-free concurrent reads, copy-on-write updates
 * 
 * Thread Safety:
 * - Queries: Lock-free, never block
 * - Updates: Lock-free via RCU atomic store
 * - Multiple readers AND writers can operate concurrently
 * - No mutexes anywhere - pure RCU synchronization
 * - Readers never see partial updates (always consistent snapshot)
 * 
 * Performance:
 * - Queries: O(1) for point queries, O(k) for range queries
 * - Updates: O(n) copy overhead, but amortized for bulk operations
 * - Memory: width * height * 1 bit (e.g., 100m×100m @ 5cm = 500KB)
 */
class GridMap {
public:
    using Config = GridMapData::Config;
    
    /**
     * @brief Construct gridmap with configuration
     */
    explicit GridMap(const Config& config);
    
    // ============================================================================
    // Query Operations (Lock-free, thread-safe for concurrent reads)
    // ============================================================================
    
    /**
     * @brief Check if point is occupied
     * @return true if occupied, false if free
     */
    [[nodiscard]] bool isOccupied(const double x, const double y) const;

    /**
     * @brief Check if point is occupied on specific layer(s)
     * @param mask Bitmask of ObstacleLayer values (e.g. FIXED | STATIC)
     * @return true if occupied on any of the specified layers
     */
    [[nodiscard]] bool isOccupied(const double x, const double y,
                                  const ObstacleLayer mask) const;
    
    /**
     * @brief Find obstacles near a point
     * @return Vector of obstacle IDs within radius
     */
    [[nodiscard]] std::vector<int64_t> findObstaclesNear(
        const double x, const double y, const double radius) const;
    
    /**
     * @brief Find obstacles with at least one cell in the safety circle
     * 
     * Returns all obstacles that have ANY cell intersecting the circle.
     * Useful for safety checks where even partial obstacle overlap matters.
     * 
     * Thread-safe: Uses RCU snapshot.
     */
    [[nodiscard]] std::vector<int64_t> findObstaclesInSafetyCircle(
        const double center_x, const double center_y, const double radius) const;
    
    /**
     * @brief Check if line segment intersects any obstacle
     * @return true if path is blocked
     */
    [[nodiscard]] bool isPathBlocked(const double x1, const double y1,
                                     const double x2, const double y2) const;

    /**
     * @brief Check if line segment intersects obstacles on specific layer(s)
     * @param mask Bitmask of ObstacleLayer values (e.g. FIXED | STATIC)
     * @return true if path is blocked on any of the specified layers
     */
    [[nodiscard]] bool isPathBlocked(const double x1, const double y1,
                                     const double x2, const double y2,
                                     const ObstacleLayer mask) const;
    
    /**
     * @brief Get grid dimensions
     */
    [[nodiscard]] inline std::size_t getGridWidth()  const { return this->snapshot_.load()->getGridWidth(); }
    [[nodiscard]] inline std::size_t getGridHeight() const { return this->snapshot_.load()->getGridHeight(); }
    [[nodiscard]] inline double getResolution() const { return this->snapshot_.load()->getResolution(); }
    
    // ============================================================================
    // Mutation Operations (Thread-safe via RCU copy-on-write)
    // ============================================================================
    
    /**
     * @brief Insert or update obstacle with optional inflation
     * 
     * Thread-safe: Creates new snapshot with modified obstacle.
     * If ID already exists, old obstacle is removed first.
     * 
     * @param id Unique obstacle identifier
     * @param obstacle ROS obstacle message
     * @param inflation_radius Additional radius (in meters) to inflate obstacle, compensates for sensor noise
     * @param layer ObstacleLayer classification (FIXED, STATIC, or DYNAMIC)
     */
    void insertObstacle(const int64_t id, const rises_interfaces::msg::Obstacle& obstacle,
                        float inflation_radius = 0.0f,
                        ObstacleLayer layer = ObstacleLayer::STATIC);

    /**
     * @brief Remove obstacle by ID.
     */
    void removeObstacle(const int64_t id);

    /**
     * @brief Clear all obstacles belonging to the given layer.
     */
    void clearLayer(const ObstacleLayer layer);

    /**
     * @brief Clear all obstacles
     */
    void clear();
    
    /**
     * @brief Bulk update - apply multiple operations in single copy
     * @param updater Lambda that modifies mutable GridMapData
     * 
     * Example:
     * @code
     *   map.updateSnapshot([](GridMapData& data) {
     *       ObstacleInsertionPolicy::insert(data, 1, obstacle1);
     *       ObstacleInsertionPolicy::insert(data, 2, obstacle2);
     *       ObstacleRemovalPolicy::remove(data, 3);
     *   });
     * @endcode
     */
    template<typename Func>
    void updateSnapshot(Func&& updater);
    
private:
    // RCU snapshot for lock-free reads AND writes
    rcu::AtomicSharedPtr<GridMapData> snapshot_;
    
    // Grid configuration
    Config config_;
};

// ============================================================================
// Template Implementation
// ============================================================================

template<typename Func>
void GridMap::updateSnapshot(Func&& updater) {
    std::shared_ptr<const GridMapData> current = this->snapshot_.load();
    std::shared_ptr<GridMapData>       new_data = std::make_shared<GridMapData>(*current);
    updater(*new_data);
    this->snapshot_.store(new_data);
}

} // namespace geofence
} // namespace rises
