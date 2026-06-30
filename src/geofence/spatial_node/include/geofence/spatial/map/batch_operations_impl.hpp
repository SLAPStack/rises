#pragma once

/**
 * @file batch_operations_impl.hpp
 * @brief Template implementations for batch update operations
 */

#include "batch_operations.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace rises::geofence {

// ============================================================================
// Bulk Insertion Implementation
// ============================================================================

inline std::size_t BatchOperations::insertBatch(
    GeofenceMap& map,
    const std::vector<ObstaclePair>& obstacles) {

    if (obstacles.empty()) {
        return 0;
    }

    // Validate all obstacles first (fail-fast)
    for (const auto& [id, geom] : obstacles) {
        if (!validateObstacle(id, geom)) {
            throw std::invalid_argument(
                "Invalid obstacle in batch: id=" + std::to_string(id));
        }
    }

    // CRITICAL: Single updateSnapshot call for ALL insertions
    // This is the key optimization - amortizes copy-on-write cost
    std::size_t inserted_count = 0;

    map.updateSnapshot([&obstacles, &inserted_count](
        GeofenceMap::Snapshot& snap) {

        // Reserve space to minimize reallocations
        snap.obstacles.reserve(snap.obstacles.size() + obstacles.size());

        for (const auto& [id, geom] : obstacles) {
            // Check if obstacle already exists (remove old entry first)
            auto existing = snap.obstacles.find(id);
            if (existing != snap.obstacles.end()) {
                const BoundingBox old_bbox{
                    static_cast<float>(existing->second.bbox.min.x),
                    static_cast<float>(existing->second.bbox.min.y),
                    static_cast<float>(existing->second.bbox.max.x),
                    static_cast<float>(existing->second.bbox.max.y)
                };
                snap.obstacle_tree->remove(id, old_bbox);
                snap.obstacles.erase(existing);
            }

            // Create new entry
            GeometryEntry entry = createEntry(id, geom);

            // Insert into spatial index
            const BoundingBox bbox{
                static_cast<float>(entry.bbox.min.x),
                static_cast<float>(entry.bbox.min.y),
                static_cast<float>(entry.bbox.max.x),
                static_cast<float>(entry.bbox.max.y)
            };
            snap.obstacle_tree->insert(id, bbox);

            // Insert into obstacle map
            snap.obstacles.emplace(id, std::move(entry));
            inserted_count++;
        }
    });

    return inserted_count;
}

inline std::size_t BatchOperations::insertBatch(
    GeofenceMap& map,
    std::vector<ObstaclePair>&& obstacles) {

    if (obstacles.empty()) {
        return 0;
    }

    // Validate all obstacles
    for (const auto& [id, geom] : obstacles) {
        if (!validateObstacle(id, geom)) {
            throw std::invalid_argument(
                "Invalid obstacle in batch: id=" + std::to_string(id));
        }
    }

    std::size_t inserted_count = 0;

    map.updateSnapshot([obstacles = std::move(obstacles), &inserted_count](
        GeofenceMap::Snapshot& snap) mutable {

        snap.obstacles.reserve(snap.obstacles.size() + obstacles.size());

        for (auto& [id, geom] : obstacles) {
            // Check for existing
            auto existing = snap.obstacles.find(id);
            if (existing != snap.obstacles.end()) {
                const BoundingBox old_bbox{
                    static_cast<float>(existing->second.bbox.min.x),
                    static_cast<float>(existing->second.bbox.min.y),
                    static_cast<float>(existing->second.bbox.max.x),
                    static_cast<float>(existing->second.bbox.max.y)
                };
                snap.obstacle_tree->remove(id, old_bbox);
                snap.obstacles.erase(existing);
            }

            // Move geometry (zero-copy)
            GeometryEntry entry(id, std::move(geom));

            const BoundingBox bbox{
                static_cast<float>(entry.bbox.min.x),
                static_cast<float>(entry.bbox.min.y),
                static_cast<float>(entry.bbox.max.x),
                static_cast<float>(entry.bbox.max.y)
            };
            snap.obstacle_tree->insert(id, bbox);
            snap.obstacles.emplace(id, std::move(entry));
            inserted_count++;
        }
    });

    return inserted_count;
}

// ============================================================================
// Bulk Removal Implementation
// ============================================================================

inline std::size_t BatchOperations::removeBatch(
    GeofenceMap& map,
    const std::vector<int64_t>& ids) {

    if (ids.empty()) {
        return 0;
    }

    std::size_t removed_count = 0;

    map.updateSnapshot([&ids, &removed_count](
        GeofenceMap::Snapshot& snap) {

        for (int64_t id : ids) {
            auto it = snap.obstacles.find(id);
            if (it != snap.obstacles.end()) {
                const BoundingBox bbox{
                    static_cast<float>(it->second.bbox.min.x),
                    static_cast<float>(it->second.bbox.min.y),
                    static_cast<float>(it->second.bbox.max.x),
                    static_cast<float>(it->second.bbox.max.y)
                };
                snap.obstacle_tree->remove(id, bbox);
                snap.obstacles.erase(it);
                removed_count++;
            }
        }
    });

    return removed_count;
}

// ============================================================================
// Bulk Area Registration
// ============================================================================

inline std::size_t BatchOperations::registerAreasBatch(
    GeofenceMap& map,
    const std::vector<AreaPair>& areas) {

    if (areas.empty()) {
        return 0;
    }

    std::size_t registered_count = 0;

    map.updateSnapshot([&areas, &registered_count](
        GeofenceMap::Snapshot& snap) {

        snap.areas.reserve(snap.areas.size() + areas.size());

        for (const auto& [id, area] : areas) {
            // Validate area
            if (area.min.x > area.max.x || area.min.y > area.max.y) {
                throw std::invalid_argument("Invalid area: min > max");
            }

            // Insert into spatial index
            const BoundingBox bbox{
                static_cast<float>(area.min.x),
                static_cast<float>(area.min.y),
                static_cast<float>(area.max.x),
                static_cast<float>(area.max.y)
            };
            snap.area_tree->insert(id, bbox);
            snap.areas[id] = area;
            registered_count++;
        }
    });

    return registered_count;
}

// ============================================================================
// Combined Update
// ============================================================================

inline std::pair<std::size_t, std::size_t> BatchOperations::updateBatch(
    GeofenceMap& map,
    const std::vector<ObstaclePair>& to_insert,
    const std::vector<int64_t>& to_remove) {

    if (to_insert.empty() && to_remove.empty()) {
        return {0, 0};
    }

    std::size_t inserted_count = 0;
    std::size_t removed_count = 0;

    map.updateSnapshot([&](GeofenceMap::Snapshot& snap) {
        // Process removals first
        for (int64_t id : to_remove) {
            auto it = snap.obstacles.find(id);
            if (it != snap.obstacles.end()) {
                const BoundingBox bbox{
                    static_cast<float>(it->second.bbox.min.x),
                    static_cast<float>(it->second.bbox.min.y),
                    static_cast<float>(it->second.bbox.max.x),
                    static_cast<float>(it->second.bbox.max.y)
                };
                snap.obstacle_tree->remove(id, bbox);
                snap.obstacles.erase(it);
                removed_count++;
            }
        }

        // Then process insertions
        for (const auto& [id, geom] : to_insert) {
            if (!validateObstacle(id, geom)) continue;

            GeometryEntry entry = createEntry(id, geom);
            const BoundingBox bbox{
                static_cast<float>(entry.bbox.min.x),
                static_cast<float>(entry.bbox.min.y),
                static_cast<float>(entry.bbox.max.x),
                static_cast<float>(entry.bbox.max.y)
            };
            snap.obstacle_tree->insert(id, bbox);
            snap.obstacles.emplace(id, std::move(entry));
            inserted_count++;
        }
    });

    return {inserted_count, removed_count};
}

// ============================================================================
// ROS Message Processing
// ============================================================================

inline std::pair<std::size_t, std::size_t> BatchOperations::processROSUpdates(
    GeofenceMap& map,
    const rises_interfaces::msg::ObstacleUpdateArray& updates) {

    if (updates.updates.empty()) {
        return {0, 0};
    }

    // Separate inserts and removes
    std::vector<ObstaclePair> to_insert;
    std::vector<int64_t> to_remove;

    to_insert.reserve(updates.updates.size());
    to_remove.reserve(updates.updates.size());

    for (const auto& update : updates.updates) {
        if (update.operation == rises_interfaces::msg::ObstacleUpdate::OP_INSERT) {
            Geometry geom = rises::geofence::fromObstacleMsg(update.obstacle);
            to_insert.emplace_back(update.obstacle.id, std::move(geom));
        } else if (update.operation == rises_interfaces::msg::ObstacleUpdate::OP_DELETE) {
            to_remove.push_back(update.obstacle.id);
        }
    }

    // Single atomic update
    return updateBatch(map, to_insert, to_remove);
}

// ============================================================================
// Internal Helpers
// ============================================================================

inline bool BatchOperations::validateObstacle(
    int64_t id, const Geometry& geom) {

    if (id < 0) {
        return false;
    }

    // Validate bounding box
    const BoundingBox bbox = getBoundingBox(geom);
    if (!bbox.isValid()) {
        return false;
    }

    // Check for NaN/infinity
    if (!std::isfinite(bbox.min_x) || !std::isfinite(bbox.min_y) ||
        !std::isfinite(bbox.max_x) || !std::isfinite(bbox.max_y)) {
        return false;
    }

    return true;
}

inline GeometryEntry BatchOperations::createEntry(
    int64_t id, Geometry geom) {

    return GeometryEntry(id, std::move(geom));
}

// ============================================================================
// Batch Point Checking Implementation
// ============================================================================

namespace detail {

// Helper: Check if point collides with geometry (within buffer distance)
inline bool pointCollidesWithGeometry(const Point2D& point, const Geometry& geom, const float buffer) {
    const float dist_sq = distanceSquaredToPoint(geom, point);
    return dist_sq <= (buffer * buffer);
}

} // namespace detail

// ============================================================================
// Batch Point Collision Checking (Sequential and Parallel)
// ============================================================================

template<ExecutionPolicy Policy, ObstacleLayer Layer>
ObstacleResult BatchOperations::checkBatch(
    const GeofenceMap& map,
    const std::vector<Point2D>& points,
    const Point2D& robot_pos,
    const float buffer_distance)
{
    ObstacleResult result;
    result.has_collision = false;
    
    if (points.empty()) {
        return result;
    }
    
    // Compute bounding box for spatial query
    float min_x = static_cast<float>(robot_pos.x);
    float max_x = static_cast<float>(robot_pos.x);
    float min_y = static_cast<float>(robot_pos.y);
    float max_y = static_cast<float>(robot_pos.y);

    for (const Point2D& point : points) {
        min_x = std::min(min_x, static_cast<float>(point.x));
        max_x = std::max(max_x, static_cast<float>(point.x));
        min_y = std::min(min_y, static_cast<float>(point.y));
        max_y = std::max(max_y, static_cast<float>(point.y));
    }
    
    // Expand by buffer distance
    const BoundingBox query_box(
        min_x - buffer_distance,
        min_y - buffer_distance,
        max_x + buffer_distance,
        max_y + buffer_distance
    );
    
    // Sequential execution path
    if constexpr (Policy == ExecutionPolicy::seq) {
        // Check static layer if requested
        if constexpr (Layer == ObstacleLayer::STATIC || Layer == ObstacleLayer::ALL) {
            map.forEachObstacleInRegion(query_box, ObstacleLayer::STATIC, [&](const GeometryEntry& entry) {
                for (std::size_t i = 0; i < points.size(); ++i) {
                    if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                        result.has_collision = true;
                        result.unmatched_vertices_per_obstacle[entry.id].insert(i);
                    }
                }
            });
        }
        
        // Check dynamic layer if requested (100% lock-free with embedded geometry)
        if constexpr (Layer == ObstacleLayer::DYNAMIC || Layer == ObstacleLayer::ALL) {
            map.forEachObstacleInRegion(query_box, ObstacleLayer::DYNAMIC, [&](const GeometryEntry& entry) {
                for (std::size_t i = 0; i < points.size(); ++i) {
                    if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                        result.has_collision = true;
                        result.unmatched_vertices_per_obstacle[entry.id].insert(i);
                    }
                }
            });
        }
    }
    // Parallel execution path (split work across 2 threads)
    else if constexpr (Policy == ExecutionPolicy::par) {
        const std::size_t mid = points.size() / 2;
        
        // Only parallelize if we have enough work to justify thread overhead
        if (mid > 0 && points.size() >= 20) {
            // Results for parallel threads (thread-local to avoid contention)
            ObstacleResult result_first_half;
            result_first_half.has_collision = false;
            
            ObstacleResult result_second_half;
            result_second_half.has_collision = false;
            
            // Process first half in parallel thread
            std::future<void> future = GeofenceThreadPool::instance().enqueue([&]() {
                // Static layer (first half)
                if constexpr (Layer == ObstacleLayer::STATIC || Layer == ObstacleLayer::ALL) {
                    map.forEachObstacleInRegion(query_box, ObstacleLayer::STATIC, [&](const GeometryEntry& entry) {
                        for (std::size_t i = 0; i < mid; ++i) {
                            if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                                result_first_half.has_collision = true;
                                result_first_half.unmatched_vertices_per_obstacle[entry.id].insert(i);
                            }
                        }
                    });
                }
                
                // Dynamic layer (first half)
                if constexpr (Layer == ObstacleLayer::DYNAMIC || Layer == ObstacleLayer::ALL) {
                    map.forEachObstacleInRegion(query_box, ObstacleLayer::DYNAMIC, [&](const GeometryEntry& entry) {
                        for (std::size_t i = 0; i < mid; ++i) {
                            if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                                result_first_half.has_collision = true;
                                result_first_half.unmatched_vertices_per_obstacle[entry.id].insert(i);
                            }
                        }
                    });
                }
            });
            
            // Process second half in current thread
            // Static layer (second half)
            if constexpr (Layer == ObstacleLayer::STATIC || Layer == ObstacleLayer::ALL) {
                map.forEachObstacleInRegion(query_box, ObstacleLayer::STATIC, [&](const GeometryEntry& entry) {
                    for (std::size_t i = mid; i < points.size(); ++i) {
                        if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                            result_second_half.has_collision = true;
                            result_second_half.unmatched_vertices_per_obstacle[entry.id].insert(i);
                        }
                    }
                });
            }
            
            // Dynamic layer (second half)
            if constexpr (Layer == ObstacleLayer::DYNAMIC || Layer == ObstacleLayer::ALL) {
                map.forEachObstacleInRegion(query_box, ObstacleLayer::DYNAMIC, [&](const GeometryEntry& entry) {
                    for (std::size_t i = mid; i < points.size(); ++i) {
                        if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                            result_second_half.has_collision = true;
                            result_second_half.unmatched_vertices_per_obstacle[entry.id].insert(i);
                        }
                    }
                });
            }
            
            // Wait for parallel thread to complete
            future.wait();
            
            // Merge results
            result.has_collision = result_first_half.has_collision || result_second_half.has_collision;
            
            // Merge unmatched vertices
            for (const auto& [obstacle_id, vertex_set] : result_first_half.unmatched_vertices_per_obstacle) {
                result.unmatched_vertices_per_obstacle[obstacle_id].insert(vertex_set.begin(), vertex_set.end());
            }
            for (const auto& [obstacle_id, vertex_set] : result_second_half.unmatched_vertices_per_obstacle) {
                result.unmatched_vertices_per_obstacle[obstacle_id].insert(vertex_set.begin(), vertex_set.end());
            }
        } else {
            // Too few points - fall back to sequential execution
            // Check static layer if requested
            if constexpr (Layer == ObstacleLayer::STATIC || Layer == ObstacleLayer::ALL) {
                map.forEachObstacleInRegion(query_box, ObstacleLayer::STATIC, [&](const GeometryEntry& entry) {
                    for (std::size_t i = 0; i < points.size(); ++i) {
                        if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                            result.has_collision = true;
                            result.unmatched_vertices_per_obstacle[entry.id].insert(i);
                        }
                    }
                });
            }
            
            // Check dynamic layer if requested
            if constexpr (Layer == ObstacleLayer::DYNAMIC || Layer == ObstacleLayer::ALL) {
                map.forEachObstacleInRegion(query_box, ObstacleLayer::DYNAMIC, [&](const GeometryEntry& entry) {
                    for (std::size_t i = 0; i < points.size(); ++i) {
                        if (detail::pointCollidesWithGeometry(points[i], entry.geometry, buffer_distance)) {
                            result.has_collision = true;
                            result.unmatched_vertices_per_obstacle[entry.id].insert(i);
                        }
                    }
                });
            }
        }
    }

    return result;
}

} // namespace rises::geofence
