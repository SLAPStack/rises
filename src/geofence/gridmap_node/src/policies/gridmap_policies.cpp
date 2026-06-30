#include "geofence/gridmap/policies/gridmap_policies.hpp"
#include "geofence/utils/compiler_hints.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>
#include <cmath>

namespace rises {
namespace geofence {

// ============================================================================
// ObstacleInsertionPolicy Implementation
// ============================================================================

/**
 * Inserts or updates an obstacle in the grid.
 * If an obstacle with the same ID exists, it is removed first to avoid stale cells.
 * Dispatches to appropriate rasterization method based on obstacle type.
 */
void ObstacleInsertionPolicy::insert(GridMapData& data, const int64_t id,
                                    const rises_interfaces::msg::Obstacle& obstacle,
                                    const float inflation_radius,
                                    const ObstacleLayer layer) {
    // Remove old version if exists (ensures no stale occupied cells)
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    if (obstacles.count(id)) {
        ObstacleRemovalPolicy::remove(data, id);
    }
    
    // Create obstacle info
    GridMapData::ObstacleInfo info;
    info.obstacle = obstacle;
    info.layer    = layer;
    obstacles[id] = std::move(info);
    
    // Rasterize based on type (with inflation)
    switch (obstacle.type) {
        case rises_interfaces::msg::Obstacle::CIRCLE:
            rasterizeCircle(data, id, obstacle.position.x, obstacle.position.y, obstacle.radius, inflation_radius, layer);
            break;
            
        case rises_interfaces::msg::Obstacle::POLYGON:
        case rises_interfaces::msg::Obstacle::CONVEX_POLYGON:
        case rises_interfaces::msg::Obstacle::FREEFORM: {
            std::vector<Point2D> vertices;
            vertices.reserve(obstacle.vertices.size());
            for (const geometry_msgs::msg::Point& pt : obstacle.vertices) {
                vertices.push_back({pt.x, pt.y});
            }
            rasterizePolygon(data, id, vertices, inflation_radius, layer);
            break;
        }
        
        case rises_interfaces::msg::Obstacle::POINT:
            rasterizePoint(data, id, obstacle.position.x, obstacle.position.y, inflation_radius, layer);
            break;
            
        case rises_interfaces::msg::Obstacle::LINE:
            if (obstacle.vertices.size() >= 2) {
                const geometry_msgs::msg::Point& p1 = obstacle.vertices[0];
                const geometry_msgs::msg::Point& p2 = obstacle.vertices[1];
                rasterizeLine(data, id, p1.x, p1.y, p2.x, p2.y, inflation_radius, layer);
            }
            break;
            
        case rises_interfaces::msg::Obstacle::RECTANGLE:
            rasterizeRectangle(data, id, obstacle.position.x, obstacle.position.y,
                             obstacle.width, obstacle.height, obstacle.orientation, inflation_radius, layer);
            break;
            
        default:
            // Unknown type, ignore
            break;
    }
}

/**
 * Rasterizes a circle into the grid using distance checks.
 * 
 * Scans all cells within the bounding box and marks those within the radius.
 * Uses squared distance comparison to avoid expensive sqrt operations.
 * Handles circles with center outside grid bounds.
 * 
 * Complexity: O(r²) where r is radius in grid cells
 */
void ObstacleInsertionPolicy::rasterizeCircle(GridMapData& data, const int64_t id,
                                              const double cx, const double cy,
                                              const double radius, const float inflation,
                                              const ObstacleLayer layer) {
    // Convert center to grid coordinates (may be outside grid)
    const double inv_res = 1.0 / data.getResolution();
    const int center_x = static_cast<int>(std::floor((cx - data.getOriginX()) * inv_res));
    const int center_y = static_cast<int>(std::floor((cy - data.getOriginY()) * inv_res));

    // Apply inflation to radius
    const double inflated_radius = radius + inflation;
    const double inflated_radius_sq = inflated_radius * inflated_radius;
    const int radius_cells = static_cast<int>(std::ceil(inflated_radius * inv_res));

    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    GridMapData::ObstacleInfo& info = obstacles[id];
    std::vector<uint8_t>& grid        = data.getMutableGrid();
    std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(layer);
    std::vector<uint16_t>& ref_count  = data.getMutableRefCount(layer);

    // Scan bounding box, clipping to grid bounds
    const int min_x = std::max(0, center_x - radius_cells);
    const int max_x = std::min(static_cast<int>(data.getGridWidth()) - 1, center_x + radius_cells);
    const int min_y = std::max(0, center_y - radius_cells);
    const int max_y = std::min(static_cast<int>(data.getGridHeight()) - 1, center_y + radius_cells);
    const double res = data.getResolution();

    for (int gy = min_y; gy <= max_y; ++gy) {
        for (int gx = min_x; gx <= max_x; ++gx) {
            // Check if cell center is within inflated circle
            const double dx = (gx - center_x) * res;
            const double dy = (gy - center_y) * res;

            if (dx * dx + dy * dy <= inflated_radius_sq) {
                const uint32_t idx = data.gridToIndex(gx, gy);
                grid[idx]       = 1;
                layer_grid[idx] = 1;
                ++ref_count[idx];
                info.occupied_cells.push_back(idx);
            }
        }
    }
}

/**
 * Rasterizes an arbitrary polygon using ray-casting point-in-polygon test.
 * 
 * Algorithm:
 * 1. Find axis-aligned bounding box of polygon in world coords
 * 2. Convert to grid coords and clip to grid bounds
 * 3. For each cell center in bounding box:
 *    - Cast horizontal ray to infinity
 *    - Count edge crossings
 *    - Odd count = inside, Even count = outside
 * 
 * Works for both convex and concave polygons.
 * Complexity: O(w×h×n) where w,h=bounding box size, n=vertex count
 */
void ObstacleInsertionPolicy::rasterizePolygon(GridMapData& data, const int64_t id,
                                               const std::vector<Point2D>& vertices,
                                               const float inflation,
                                               const ObstacleLayer layer) {
    if (vertices.empty()) return;
    
    // Find bounding box in world coordinates (including out-of-bounds vertices)
    double min_x_world = vertices[0].x;
    double max_x_world = vertices[0].x;
    double min_y_world = vertices[0].y;
    double max_y_world = vertices[0].y;
    
    for (const Point2D& v : vertices) {
        min_x_world = std::min(min_x_world, v.x);
        max_x_world = std::max(max_x_world, v.x);
        min_y_world = std::min(min_y_world, v.y);
        max_y_world = std::max(max_y_world, v.y);
    }
    
    // Expand bounding box by inflation radius
    min_x_world -= inflation;
    max_x_world += inflation;
    min_y_world -= inflation;
    max_y_world += inflation;
    
    // Convert to grid coordinates and clip to grid bounds
    int min_x = 0;
    int min_y = 0;
    int max_x = 0;
    int max_y = 0;
    data.worldToGrid(min_x_world, min_y_world, min_x, min_y);
    data.worldToGrid(max_x_world, max_y_world, max_x, max_y);
    
    // Clamp to grid bounds
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    max_x = std::min(static_cast<int>(data.getGridWidth()) - 1, max_x);
    max_y = std::min(static_cast<int>(data.getGridHeight()) - 1, max_y);
    
    if (min_x > max_x || min_y > max_y) return;  // Polygon entirely out of bounds
    
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    GridMapData::ObstacleInfo& info = obstacles[id];
    std::vector<uint8_t>& grid        = data.getMutableGrid();
    std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(layer);
    std::vector<uint16_t>& ref_count  = data.getMutableRefCount(layer);
    const double resolution = data.getResolution();
    const GridMapData::Config& config = data.getConfig();
    
    // Scan bounding box and test point-in-polygon OR distance-to-edge
    for (int gy = min_y; gy <= max_y; ++gy) {
        for (int gx = min_x; gx <= max_x; ++gx) {
            // Convert grid cell center to world coordinates
            const double wx = config.origin_x + (gx + 0.5) * resolution;
            const double wy = config.origin_y + (gy + 0.5) * resolution;
            
            bool should_occupy = false;
            
            // Point-in-polygon test (ray casting)
            bool inside = false;
            const std::size_t n = vertices.size();
            for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
                const Point2D& vi = vertices[i];
                const Point2D& vj = vertices[j];
                
                if (((vi.y > wy) != (vj.y > wy)) &&
                    (wx < (vj.x - vi.x) * (wy - vi.y) / (vj.y - vi.y) + vi.x)) {
                    inside = !inside;
                }
            }
            
            should_occupy = inside;
            
            // If not inside but inflation > 0, check distance to edges
            if (!inside && inflation > 0.0f) {
                for (std::size_t i = 0; i < n; ++i) {
                    const Point2D& v1 = vertices[i];
                    const Point2D& v2 = vertices[i + 1 < n ? i + 1 : 0];
                    
                    // Compute distance from point to line segment
                    const double dx = v2.x - v1.x;
                    const double dy = v2.y - v1.y;
                    const double len_sq = dx * dx + dy * dy;
                    
                    if (len_sq < 1e-10) continue;  // Degenerate edge
                    
                    // Project point onto line
                    const double t = std::max(0.0, std::min(1.0, 
                        ((wx - v1.x) * dx + (wy - v1.y) * dy) / len_sq));
                    const double proj_x = v1.x + t * dx;
                    const double proj_y = v1.y + t * dy;
                    
                    const double dist_sq = (wx - proj_x) * (wx - proj_x) + 
                                         (wy - proj_y) * (wy - proj_y);
                    
                    if (dist_sq <= inflation * inflation) {
                        should_occupy = true;
                        break;
                    }
                }
            }
            
            if (should_occupy) {
                const uint32_t idx = data.gridToIndex(gx, gy);
                grid[idx]       = 1;
                layer_grid[idx] = 1;
                ++ref_count[idx];
                info.occupied_cells.push_back(idx);
            }
        }
    }
}

void ObstacleInsertionPolicy::rasterizePoint(GridMapData& data, const int64_t id,
                                            const double x, const double y,
                                            const float inflation,
                                            const ObstacleLayer layer) {
    // Treat point as a small circle with inflation radius
    if (inflation > 0.0f) {
        rasterizeCircle(data, id, x, y, 0.0, inflation, layer);
    } else {
        // No inflation: just mark single cell
        int grid_x = 0;
        int grid_y = 0;
        if (!data.worldToGrid(x, y, grid_x, grid_y)) {
            return;  // Point out of bounds
        }
        
        const uint32_t idx = data.gridToIndex(grid_x, grid_y);
        std::vector<uint8_t>& grid        = data.getMutableGrid();
        std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(layer);
        std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();

        grid[idx]       = 1;
        layer_grid[idx] = 1;
        ++data.getMutableRefCount(layer)[idx];
        obstacles[id].occupied_cells.push_back(idx);
    }
}

/**
 * Rasterizes a line segment using Bresenham's algorithm, with optional inflation.
 * 
 * If inflation > 0, treats the line as a thick line (rectangle swept along path).
 * Integer-only algorithm that traces the ideal line through grid cells.
 * Ensures no gaps in the rasterized line regardless of slope.
 * Handles endpoints outside grid bounds.
 * 
 * Complexity: O(max(dx, dy)) for thin line, O(max(dx, dy) * inflation) for thick line
 */
void ObstacleInsertionPolicy::rasterizeLine(GridMapData& data, const int64_t id,
                                           const double x1, const double y1,
                                           const double x2, const double y2,
                                           const float inflation,
                                           const ObstacleLayer layer) {
    if (inflation > 0.0f) {
        // For inflated line, create a thick line as a polygon
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        const double len = std::sqrt(dx * dx + dy * dy);
        
        if (len < 1e-10) {
            // Degenerate line, treat as point
            rasterizePoint(data, id, x1, y1, inflation, layer);
            return;
        }
        
        // Perpendicular unit vector scaled by inflation
        const double perp_x = -dy / len * inflation;
        const double perp_y = dx / len * inflation;
        
        // Four corners of thick line rectangle
        std::vector<Point2D> corners = {
            {x1 + perp_x, y1 + perp_y},
            {x2 + perp_x, y2 + perp_y},
            {x2 - perp_x, y2 - perp_y},
            {x1 - perp_x, y1 - perp_y}
        };
        
        rasterizePolygon(data, id, corners, 0.0f, layer);  // Already inflated
        return;
    }
    
    // No inflation: use Bresenham's line algorithm
    const double inv_res = 1.0 / data.getResolution();
    const int gx1 = static_cast<int>(std::floor((x1 - data.getOriginX()) * inv_res));
    const int gy1 = static_cast<int>(std::floor((y1 - data.getOriginY()) * inv_res));
    const int gx2 = static_cast<int>(std::floor((x2 - data.getOriginX()) * inv_res));
    const int gy2 = static_cast<int>(std::floor((y2 - data.getOriginY()) * inv_res));
    
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    GridMapData::ObstacleInfo& info = obstacles[id];
    std::vector<uint8_t>& grid        = data.getMutableGrid();
    std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(layer);
    std::vector<uint16_t>& ref_count  = data.getMutableRefCount(layer);

    const int dx = std::abs(gx2 - gx1);
    const int dy = std::abs(gy2 - gy1);
    const int sx = gx1 < gx2 ? 1 : -1;
    const int sy = gy1 < gy2 ? 1 : -1;
    int err = dx - dy;
    
    int x = gx1;
    int y = gy1;
    
    while (true) {
        // Check bounds and set cell
        if (x >= 0 && x < static_cast<int>(data.getGridWidth()) &&
            y >= 0 && y < static_cast<int>(data.getGridHeight())) {
            const uint32_t idx = data.gridToIndex(x, y);
            grid[idx]       = 1;
            layer_grid[idx] = 1;
            ++ref_count[idx];
            info.occupied_cells.push_back(idx);
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
}

/**
 * Rasterizes a rectangle (axis-aligned or rotated), with optional inflation.
 * 
 * Optimization: If not rotated (angle ~0), uses fast bounding box scan.
 * Otherwise, computes rotated corner points and delegates to polygon rasterization.
 * Inflation: Expands width and height by 2×inflation.
 * 
 * Complexity: O(w×h) for axis-aligned, O(w×h×4) for rotated
 */
void ObstacleInsertionPolicy::rasterizeRectangle(GridMapData& data, const int64_t id,
                                                 const double center_x, const double center_y,
                                                 const double width, const double height,
                                                 const double angle_rad, const float inflation,
                                                 const ObstacleLayer layer) {
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    GridMapData::ObstacleInfo& info = obstacles[id];
    std::vector<uint8_t>& grid        = data.getMutableGrid();
    std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(layer);
    std::vector<uint16_t>& ref_count  = data.getMutableRefCount(layer);

    // Apply inflation to dimensions
    const double inflated_width = width + 2.0 * inflation;
    const double inflated_height = height + 2.0 * inflation;
    
    // If no rotation, use simple axis-aligned approach
    if (std::abs(angle_rad) < 1e-6) {
        int center_gx = 0;
        int center_gy = 0;
        if (!data.worldToGrid(center_x, center_y, center_gx, center_gy)) {
            return;  // Center out of bounds
        }
        
        // Convert dimensions to grid cells (half extents)
        const double resolution = data.getResolution();
        const int half_width_cells = static_cast<int>(std::ceil(inflated_width * 0.5 / resolution));
        const int half_height_cells = static_cast<int>(std::ceil(inflated_height * 0.5 / resolution));
        
        // Scan bounding box
        for (int dy = -half_height_cells; dy <= half_height_cells; ++dy) {
            for (int dx = -half_width_cells; dx <= half_width_cells; ++dx) {
                const int gx = center_gx + dx;
                const int gy = center_gy + dy;
                
                // Check bounds
                if (gx < 0 || gx >= static_cast<int>(data.getGridWidth()) ||
                    gy < 0 || gy >= static_cast<int>(data.getGridHeight())) {
                    continue;
                }
                
                const uint32_t idx = data.gridToIndex(gx, gy);
                grid[idx]       = 1;
                layer_grid[idx] = 1;
                ++ref_count[idx];
                info.occupied_cells.push_back(idx);
            }
        }
    } else {
        // Rotated rectangle: compute corners and rasterize as polygon
        const double cos_a = std::cos(angle_rad);
        const double sin_a = std::sin(angle_rad);
        const double hw = inflated_width * 0.5;
        const double hh = inflated_height * 0.5;
        
        std::vector<Point2D> corners = {
            {center_x + hw * cos_a - hh * sin_a, center_y + hw * sin_a + hh * cos_a},
            {center_x - hw * cos_a - hh * sin_a, center_y - hw * sin_a + hh * cos_a},
            {center_x - hw * cos_a + hh * sin_a, center_y - hw * sin_a - hh * cos_a},
            {center_x + hw * cos_a + hh * sin_a, center_y + hw * sin_a - hh * cos_a}
        };
        
        rasterizePolygon(data, id, corners, 0.0f, layer);  // Already inflated
    }
}

// ============================================================================
// ObstacleRemovalPolicy Implementation
// ============================================================================

void ObstacleRemovalPolicy::remove(GridMapData& data, const int64_t id) {
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>::iterator it = obstacles.find(id);
    if (it == obstacles.end()) return;
    
    clearObstacleCells(data, id);
    obstacles.erase(it);
}

void ObstacleRemovalPolicy::clearObstacleCells(GridMapData& data, const int64_t id) {
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>& obstacles = data.getMutableObstacles();
    std::unordered_map<int64_t, GridMapData::ObstacleInfo>::iterator it = obstacles.find(id);
    if (it == obstacles.end()) return;

    const ObstacleLayer my_layer = it->second.layer;
    std::vector<uint8_t>& grid        = data.getMutableGrid();
    std::vector<uint8_t>& layer_grid  = data.getMutableLayerGrid(my_layer);
    std::vector<uint16_t>& ref_layer  = data.getMutableRefCount(my_layer);
    std::vector<uint16_t>& ref_fixed  = data.getMutableRefCount(ObstacleLayer::FIXED);
    std::vector<uint16_t>& ref_static = data.getMutableRefCount(ObstacleLayer::STATIC);
    std::vector<uint16_t>& ref_dyn    = data.getMutableRefCount(ObstacleLayer::DYNAMIC);

    for (const uint32_t idx : it->second.occupied_cells) {
        // Decrement per-layer ref count and clear layer grid if no other obstacle
        // in this layer owns the cell. O(1) — no scan over other obstacles needed.
        if (ref_layer[idx] > 0) { --ref_layer[idx]; }
        if (ref_layer[idx] == 0) { layer_grid[idx] = 0; }

        // Clear merged grid only when all three layer ref counts reach zero.
        if (ref_fixed[idx] == 0 && ref_static[idx] == 0 && ref_dyn[idx] == 0) {
            grid[idx] = 0;
        }
    }
}

// ============================================================================
// GridClearPolicy Implementation
// ============================================================================

void GridClearPolicy::clear(GridMapData& data) {
    std::fill(data.getMutableGrid().begin(), data.getMutableGrid().end(), 0);
    std::fill(data.getMutableLayerGrid(ObstacleLayer::FIXED).begin(),
              data.getMutableLayerGrid(ObstacleLayer::FIXED).end(), 0);
    std::fill(data.getMutableLayerGrid(ObstacleLayer::STATIC).begin(),
              data.getMutableLayerGrid(ObstacleLayer::STATIC).end(), 0);
    std::fill(data.getMutableLayerGrid(ObstacleLayer::DYNAMIC).begin(),
              data.getMutableLayerGrid(ObstacleLayer::DYNAMIC).end(), 0);
    std::fill(data.getMutableRefCount(ObstacleLayer::FIXED).begin(),
              data.getMutableRefCount(ObstacleLayer::FIXED).end(), 0);
    std::fill(data.getMutableRefCount(ObstacleLayer::STATIC).begin(),
              data.getMutableRefCount(ObstacleLayer::STATIC).end(), 0);
    std::fill(data.getMutableRefCount(ObstacleLayer::DYNAMIC).begin(),
              data.getMutableRefCount(ObstacleLayer::DYNAMIC).end(), 0);
    data.getMutableObstacles().clear();
}

} // namespace geofence
} // namespace rises
