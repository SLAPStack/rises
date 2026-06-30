#pragma once

#include "geofence/gridmap/map/gridmap_data.hpp"
#include "geofence/spatial/map/obstacle_layer_type.hpp"
#include <memory>

namespace rises {
namespace geofence {

/**
 * @brief Policy for inserting obstacles into gridmap
 * 
 * Handles rasterization of different geometry types into the occupancy grid.
 * Modifies GridMapData in place during copy-on-write updates.
 */
class ObstacleInsertionPolicy {
public:
    /**
     * @brief Insert obstacle into grid data
     * @param data Grid data to modify (mutable)
     * @param id Unique obstacle identifier
     * @param obstacle ROS obstacle message
     * @param inflation_radius Additional radius to inflate obstacles (in meters)
     * @param layer Obstacle layer classification (FIXED / STATIC / DYNAMIC)
     */
    static void insert(GridMapData& data, const int64_t id,
                      const rises_interfaces::msg::Obstacle& obstacle,
                      float inflation_radius = 0.0f,
                      ObstacleLayer layer = ObstacleLayer::STATIC);

private:
    static void rasterizeCircle(GridMapData& data, const int64_t id,
                               const double cx, const double cy,
                               const double radius, const float inflation,
                               const ObstacleLayer layer);
    static void rasterizePolygon(GridMapData& data, const int64_t id,
                                const std::vector<Point2D>& vertices,
                                const float inflation,
                                const ObstacleLayer layer);
    static void rasterizePoint(GridMapData& data, const int64_t id,
                              const double x, const double y,
                              const float inflation,
                              const ObstacleLayer layer);
    static void rasterizeLine(GridMapData& data, const int64_t id,
                             const double x1, const double y1,
                             const double x2, const double y2,
                             const float inflation,
                             const ObstacleLayer layer);
    static void rasterizeRectangle(GridMapData& data, const int64_t id,
                                  const double center_x, const double center_y,
                                  const double width, const double height,
                                  const double angle_rad, const float inflation,
                                  const ObstacleLayer layer);
};

/**
 * @brief Policy for removing obstacles from gridmap
 * 
 * Clears occupied cells and removes obstacle metadata.
 */
class ObstacleRemovalPolicy {
public:
    /**
     * @brief Remove obstacle from grid data
     * @param data Grid data to modify (mutable)
     * @param id Obstacle identifier to remove
     */
    static void remove(GridMapData& data, const int64_t id);

private:
    static void clearObstacleCells(GridMapData& data, const int64_t id);
};

/**
 * @brief Policy for clearing entire gridmap
 */
class GridClearPolicy {
public:
    /**
     * @brief Clear all obstacles from grid data
     * @param data Grid data to clear (mutable)
     */
    static void clear(GridMapData& data);
};

} // namespace geofence
} // namespace rises
