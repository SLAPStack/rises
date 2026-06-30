#include "geofence/gridmap/map/gridmap.hpp"

namespace rises {
namespace geofence {

GridMap::GridMap(const Config& config)
    : snapshot_(std::make_shared<GridMapData>(config)),
      config_(config) {
}

bool GridMap::isOccupied(const double x, const double y) const {
    return this->snapshot_.load()->isOccupied(x, y);
}

bool GridMap::isOccupied(const double x, const double y, const ObstacleLayer mask) const {
    return this->snapshot_.load()->isOccupied(x, y, mask);
}

std::vector<int64_t> GridMap::findObstaclesNear(const double x, const double y, const double radius) const {
    return this->snapshot_.load()->findObstaclesNear(x, y, radius);
}

std::vector<int64_t> GridMap::findObstaclesInSafetyCircle(
    const double center_x, const double center_y, const double radius) const {
    return this->snapshot_.load()->findObstaclesInSafetyCircle(center_x, center_y, radius);
}

bool GridMap::isPathBlocked(const double x1, const double y1, const double x2, const double y2) const {
    return this->snapshot_.load()->isPathBlocked(x1, y1, x2, y2);
}

bool GridMap::isPathBlocked(const double x1, const double y1, const double x2, const double y2,
                            const ObstacleLayer mask) const {
    return this->snapshot_.load()->isPathBlocked(x1, y1, x2, y2, mask);
}

/**
 * Insert/update obstacle using copy-on-write pattern.
 * 
 * RCU Update Pattern:
 * 1. Load current snapshot (lock-free)
 * 2. Create mutable copy
 * 3. Apply modification to copy
 * 4. Atomically publish new snapshot (lock-free)
 * 
 * Concurrent readers continue using old snapshot until they reload.
 * No locks needed - RCU handles memory reclamation safely.
 */
void GridMap::insertObstacle(const int64_t id,
                            const rises_interfaces::msg::Obstacle& obstacle,
                            const float inflation_radius,
                            const ObstacleLayer layer) {
    // Copy current snapshot (lock-free)
    std::shared_ptr<const GridMapData> current = this->snapshot_.load();
    std::shared_ptr<GridMapData> new_data = std::make_shared<GridMapData>(*current);
    
    // Apply insertion with inflation and layer
    ObstacleInsertionPolicy::insert(*new_data, id, obstacle, inflation_radius, layer);
    
    // Atomically publish new snapshot via RCU
    this->snapshot_.store(new_data);
}

void GridMap::removeObstacle(const int64_t id) {
    // Copy current snapshot (lock-free)
    std::shared_ptr<const GridMapData> current = this->snapshot_.load();
    std::shared_ptr<GridMapData> new_data = std::make_shared<GridMapData>(*current);
    
    // Apply removal
    ObstacleRemovalPolicy::remove(*new_data, id);
    
    // Atomically publish new snapshot via RCU
    this->snapshot_.store(new_data);
}

void GridMap::clearLayer(const ObstacleLayer layer) {
    std::shared_ptr<const GridMapData> current = this->snapshot_.load();
    std::shared_ptr<GridMapData> new_data = std::make_shared<GridMapData>(*current);
    new_data->clearLayer(layer);
    this->snapshot_.store(new_data);
}

void GridMap::clear() {
    // Copy current snapshot (lock-free)
    std::shared_ptr<const GridMapData> current = this->snapshot_.load();
    std::shared_ptr<GridMapData> new_data = std::make_shared<GridMapData>(*current);
    
    // Apply clear
    GridClearPolicy::clear(*new_data);
    
    // Atomically publish new snapshot via RCU
    this->snapshot_.store(new_data);
}

} // namespace geofence
} // namespace rises
