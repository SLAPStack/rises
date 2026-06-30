#include "geofence/gridmap/map/gridmap_data.hpp"
#include "geofence/utils/compiler_hints.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace rises {
namespace geofence {

/**
 * Initializes empty grid with all cells free.
 * Pre-computes inverse resolution for fast coordinate conversion.
 * Grid dimensions are rounded up to fully cover the specified world area.
 */
GridMapData::GridMapData(const Config& config)
    : config_(config),
      resolution_(config.resolution),
      inv_resolution_(1.0 / config.resolution),
      origin_x_(config.origin_x),
      origin_y_(config.origin_y) {
    
    this->grid_width_ = static_cast<std::size_t>(std::ceil(config.width_meters / this->resolution_));
    this->grid_height_ = static_cast<std::size_t>(std::ceil(config.height_meters / this->resolution_));
    
    // Allocate byte-per-cell grids: uint8_t for SIMD-friendly reads/writes.
    // Memory footprint: width × height bytes per grid × 4 grids
    // Example: 100m × 100m @ 5cm = 2000 × 2000 = 4M cells = 4MB per grid
    const std::size_t total_cells = this->grid_width_ * this->grid_height_;
    this->grid_.resize(total_cells, 0);
    this->grid_fixed_.resize(total_cells, 0);
    this->grid_static_.resize(total_cells, 0);
    this->grid_dynamic_.resize(total_cells, 0);

    // Per-layer reference counts: zero-initialised, incremented on insert, decremented on remove.
    this->ref_fixed_.resize(total_cells, 0);
    this->ref_static_.resize(total_cells, 0);
    this->ref_dynamic_.resize(total_cells, 0);
}

bool GridMapData::isOccupied(const double x, const double y) const {
    int grid_x = 0;
    int grid_y = 0;
    if (UNLIKELY(!this->worldToGrid(x, y, grid_x, grid_y))) {
        return false;  // Out of bounds = free
    }
    
    return this->grid_[this->gridToIndex(grid_x, grid_y)] != 0;
}

/**
 * Finds all obstacles within a circular radius.
 * 
 * Algorithm:
 * 1. Scan all grid cells within bounding box (radius converted to cells)
 * 2. Collect indices of occupied cells
 * 3. For each obstacle, check if any of its cells are in the collected set
 * 
 * Performance: O(r² + n×m) where r=radius in cells, n=obstacles, m=avg cells per obstacle.
 * Hash set lookup is O(1) average, so the dominant cost is the bounding box scan (O(r²)).
 */
std::vector<int64_t> GridMapData::findObstaclesNear(const double x, const double y, const double radius) const {
    std::unordered_set<int64_t> found_ids;
    
    // Convert radius to grid cells
    const int radius_cells = static_cast<int>(std::ceil(radius * this->inv_resolution_));
    
    int center_x = 0;
    int center_y = 0;
    if (UNLIKELY(!this->worldToGrid(x, y, center_x, center_y))) {
        return std::vector<int64_t>();
    }
    
    // Collect occupied cells in radius into a hash set for O(1) lookup
    std::unordered_set<uint32_t> occupied_set;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int gx = center_x + dx;
            const int gy = center_y + dy;

            if (UNLIKELY(gx < 0 || gx >= static_cast<int>(this->grid_width_) ||
                gy < 0 || gy >= static_cast<int>(this->grid_height_))) {
                continue;
            }

            const uint32_t idx = this->gridToIndex(gx, gy);
            if (this->grid_[idx] != 0) {
                occupied_set.insert(idx);
            }
        }
    }

    // Find which obstacles have cells in this region (O(1) hash lookup per cell)
    for (const std::pair<const int64_t, ObstacleInfo>& obstacle_pair : this->obstacles_) {
        const int64_t id = obstacle_pair.first;
        const ObstacleInfo& info = obstacle_pair.second;
        for (const uint32_t cell_idx : info.occupied_cells) {
            if (occupied_set.count(cell_idx) != 0) {
                found_ids.insert(id);
                break;  // Found this obstacle, move to next
            }
        }
    }
    
    return std::vector<int64_t>(found_ids.begin(), found_ids.end());
}

/**
 * Finds obstacles with at least one cell within a circular safety zone.
 * 
 * Algorithm:
 * 1. Scan all grid cells within bounding box of circle
 * 2. For each cell, check if its center point is within the circle radius
 * 3. If cell is occupied, identify which obstacle(s) occupy it
 * 4. Return unique obstacle IDs
 * 
 * Key behavior: An obstacle is included if ANY of its cells intersect the circle.
 * This ensures even obstacles that are only partially within the circle are detected.
 * 
 * Performance: O(r² + k×m) where r=radius in cells, k=occupied cells in circle, 
 *              m=avg cells per obstacle
 */
std::vector<int64_t> GridMapData::findObstaclesInSafetyCircle(
    const double center_x, const double center_y, const double radius) const {
    
    std::unordered_set<int64_t> found_ids;
    
    // Convert to grid coordinates
    const double inv_res = this->inv_resolution_;
    const int center_gx = static_cast<int>(std::floor((center_x - this->origin_x_) * inv_res));
    const int center_gy = static_cast<int>(std::floor((center_y - this->origin_y_) * inv_res));
    const int radius_cells = static_cast<int>(std::ceil(radius * inv_res));
    
    // Scan bounding box, clipping to grid bounds
    const int min_x = std::max(0, center_gx - radius_cells);
    const int max_x = std::min(static_cast<int>(this->grid_width_) - 1, center_gx + radius_cells);
    const int min_y = std::max(0, center_gy - radius_cells);
    const int max_y = std::min(static_cast<int>(this->grid_height_) - 1, center_gy + radius_cells);
    
    const double radius_sq = radius * radius;
    
    // For each cell in bounding box, check if it's within circle and occupied
    for (int gy = min_y; gy <= max_y; ++gy) {
        for (int gx = min_x; gx <= max_x; ++gx) {
            // Check if cell center is within circle
            const double dx = (gx - center_gx) * this->resolution_;
            const double dy = (gy - center_gy) * this->resolution_;
            const double dist_sq = dx * dx + dy * dy;
            
            if (dist_sq <= radius_sq) {
                // Cell is within circle, check if occupied
                const uint32_t idx = this->gridToIndex(gx, gy);
                if (this->grid_[idx] != 0) {
                    // Find which obstacle(s) occupy this cell
                    for (const std::pair<const int64_t, ObstacleInfo>& obstacle_pair : this->obstacles_) {
                        const int64_t id = obstacle_pair.first;
                        const ObstacleInfo& info = obstacle_pair.second;
                        
                        // Check if this obstacle has this cell
                        if (std::find(info.occupied_cells.begin(), 
                                     info.occupied_cells.end(), idx) != info.occupied_cells.end()) {
                            found_ids.insert(id);
                        }
                    }
                }
            }
        }
    }
    
    return std::vector<int64_t>(found_ids.begin(), found_ids.end());
}

/**
 * Checks if a line segment intersects any occupied cells.
 * 
 * Uses Bresenham's line algorithm for efficient integer-only rasterization.
 * Traces the line through the grid cell-by-cell, returning true on first collision.
 * 
 * @return true if any cell along the path is occupied, false if clear
 */
bool GridMapData::isPathBlocked(const double x1, const double y1, const double x2, const double y2) const {
    // Bresenham's line algorithm: integer-only line rasterization
    int gx1 = 0;
    int gy1 = 0;
    int gx2 = 0;
    int gy2 = 0;
    if (!this->worldToGrid(x1, y1, gx1, gy1) || !this->worldToGrid(x2, y2, gx2, gy2)) {
        return false;  // Path goes out of bounds, consider free
    }
    
    const int dx = std::abs(gx2 - gx1);
    const int dy = std::abs(gy2 - gy1);
    const int sx = gx1 < gx2 ? 1 : -1;
    const int sy = gy1 < gy2 ? 1 : -1;
    int err = dx - dy;
    
    int x = gx1;
    int y = gy1;
    
    while (true) {
        // Check current cell
        if (x >= 0 && x < static_cast<int>(this->grid_width_) &&
            y >= 0 && y < static_cast<int>(this->grid_height_)) {
            if (this->grid_[this->gridToIndex(x, y)] != 0) {
                return true;  // Path blocked
            }
        }
        
        if (x == gx2 && y == gy2) break;
        
        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return false;
}

// ============================================================================
// Layer-aware query methods
// ============================================================================

/**
 * Layer-filtered occupancy check.
 * Queries only the per-layer grids corresponding to the bits set in `mask`.
 * This allows callers to restrict collision checks to specific layers:
 *   - Path planning: FIXED | STATIC  (permanent obstacles; ignore transient dynamics)
 *   - Collision avoidance: ALL       (check everything)
 *   - Robot tracking: DYNAMIC only
 */
bool GridMapData::isOccupied(const double x, const double y, const ObstacleLayer mask) const {
    int grid_x = 0;
    int grid_y = 0;
    if (UNLIKELY(!this->worldToGrid(x, y, grid_x, grid_y))) return false;

    const uint32_t idx = this->gridToIndex(grid_x, grid_y);

    if (hasLayer(mask, ObstacleLayer::FIXED)   && this->grid_fixed_[idx]   != 0) return true;
    if (hasLayer(mask, ObstacleLayer::STATIC)  && this->grid_static_[idx]  != 0) return true;
    if (hasLayer(mask, ObstacleLayer::DYNAMIC) && this->grid_dynamic_[idx] != 0) return true;
    return false;
}

/**
 * Layer-filtered path blocked check using Bresenham's line algorithm.
 * Same as isPathBlocked() but only queries the layers specified by `mask`.
 */
bool GridMapData::isPathBlocked(const double x1, const double y1,
                                const double x2, const double y2,
                                const ObstacleLayer mask) const {
    int gx1 = 0, gy1 = 0, gx2 = 0, gy2 = 0;
    if (!this->worldToGrid(x1, y1, gx1, gy1) || !this->worldToGrid(x2, y2, gx2, gy2))
        return false;

    const int dx = std::abs(gx2 - gx1);
    const int dy = std::abs(gy2 - gy1);
    const int sx = gx1 < gx2 ? 1 : -1;
    const int sy = gy1 < gy2 ? 1 : -1;
    int err = dx - dy;
    int x = gx1, y = gy1;

    while (true) {
        if (x >= 0 && x < static_cast<int>(this->grid_width_) &&
            y >= 0 && y < static_cast<int>(this->grid_height_)) {
            const uint32_t idx = this->gridToIndex(x, y);
            if (hasLayer(mask, ObstacleLayer::FIXED)   && this->grid_fixed_[idx]   != 0) return true;
            if (hasLayer(mask, ObstacleLayer::STATIC)  && this->grid_static_[idx]  != 0) return true;
            if (hasLayer(mask, ObstacleLayer::DYNAMIC) && this->grid_dynamic_[idx] != 0) return true;
        }
        if (x == gx2 && y == gy2) break;
        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
    return false;
}

/**
 * Per-layer grid mutator — dispatches on layer to the corresponding std::vector<uint8_t>.
 * Only used by policy classes during copy-on-write updates.
 */
std::vector<uint8_t>& GridMapData::getMutableLayerGrid(const ObstacleLayer layer) {
    switch (layer) {
        case ObstacleLayer::FIXED:   return this->grid_fixed_;
        case ObstacleLayer::STATIC:  return this->grid_static_;
        case ObstacleLayer::DYNAMIC: return this->grid_dynamic_;
        default:                     return this->grid_static_;   // safe fallback
    }
}

const std::vector<uint8_t>& GridMapData::getLayerGrid(const ObstacleLayer layer) const {
    switch (layer) {
        case ObstacleLayer::FIXED:   return this->grid_fixed_;
        case ObstacleLayer::STATIC:  return this->grid_static_;
        case ObstacleLayer::DYNAMIC: return this->grid_dynamic_;
        default:                     return this->grid_static_;
    }
}

/**
 * Per-layer reference count mutator — used by policy classes to track obstacle cell ownership.
 * Enables O(cells_per_obstacle) removal without scanning other obstacles.
 */
std::vector<uint16_t>& GridMapData::getMutableRefCount(const ObstacleLayer layer) {
    switch (layer) {
        case ObstacleLayer::FIXED:   return this->ref_fixed_;
        case ObstacleLayer::STATIC:  return this->ref_static_;
        case ObstacleLayer::DYNAMIC: return this->ref_dynamic_;
        default:                     return this->ref_static_;
    }
}

/**
 * Clears all obstacles that belong to `layer`.
 * After removal, the merged grid is rebuilt as the bitwise OR of the 3 remaining layer grids.
 * Using uint8_t OR (|) instead of bool short-circuit (||) — auto-vectorisable by the compiler.
 */
void GridMapData::clearLayer(const ObstacleLayer layer) {
    // 1. Clear the per-layer grid and its reference counts
    std::vector<uint8_t>& lg = this->getMutableLayerGrid(layer);
    std::fill(lg.begin(), lg.end(), 0);

    std::vector<uint16_t>& rc = this->getMutableRefCount(layer);
    std::fill(rc.begin(), rc.end(), 0);

    // 2. Erase all obstacles belonging to this layer
    for (std::unordered_map<int64_t, ObstacleInfo>::iterator it = this->obstacles_.begin();
         it != this->obstacles_.end(); ) {
        if (it->second.layer == layer) {
            it = this->obstacles_.erase(it);
        } else {
            ++it;
        }
    }

    // 3. Rebuild merged grid as bitwise OR of the 3 (now updated) layer grids.
    // Scalar loop over uint8_t is auto-vectorisable (compiler emits SIMD OR on -O2+).
    const std::size_t n = this->grid_.size();
    for (std::size_t i = 0; i < n; ++i) {
        this->grid_[i] = this->grid_fixed_[i] | this->grid_static_[i] | this->grid_dynamic_[i];
    }
}

/**
 * Returns the count of obstacles belonging to the specified layer.
 */
std::size_t GridMapData::getObstacleCount(const ObstacleLayer layer) const {
    std::size_t count = 0;
    for (const auto& [id, info] : this->obstacles_) {
        if (info.layer == layer) ++count;
    }
    return count;
}

} // namespace geofence
} // namespace rises

