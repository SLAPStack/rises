/**
 * @file geofence_map_impl.hpp
 * @brief Template implementations for GeofenceMap query operations
 * 
 * Contains header-only implementations of template methods for zero-cost
 * iteration and optimized spatial queries. These methods are inlined for
 * maximum performance and support arbitrary visitor functors.
 * 
 * Key Features:
 * - Zero-copy iteration via visitor pattern
 * - Expanding radius search for distance queries
 * - Lock-free reads via RCU snapshots
 * - Backend-agnostic collision detection
 */

#pragma once

#include "geofence_map.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "geofence/spatial/common/bounding_box.hpp"
#include <algorithm>
#include <limits>

namespace rises {
namespace geofence {

// Note: This file uses Point2D from geometry_types.hpp, not backend-specific Point types

// ============================================================================
// Template Implementations for Zero-Cost Iteration
// ============================================================================

/**
 * @brief Iterates over all obstacles with zero-copy visitor pattern
 * 
 * Provides read-only access to obstacle entries without copying. The visitor
 * is called with const references, ensuring thread-safe read operations via
 * the RCU snapshot mechanism.
 * 
 * @tparam Visitor Callable type: void(const GeometryEntry&)
 * @param visitor Function/lambda to call for each obstacle
 * 
 * @note Thread-safe: Multiple readers can iterate concurrently
 * @note Sees consistent snapshot - no partial updates visible
 */
template<typename Visitor, typename>
void GeofenceMap::forEachObstacle(Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, const GeometryEntry&>,
                  "Visitor must be callable with (const GeometryEntry&)");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    for (const std::pair<const int64_t, GeometryEntry>& obstacle_entry : snap->obstacles) {
        visitor(obstacle_entry.second);
    }
}

/**
 * @brief Iterates over all navigation areas with zero-copy access
 * 
 * Provides read-only access to area definitions. Visitor receives both
 * the area ID and its rectangle bounds.
 * 
 * @tparam Visitor Callable type: void(int64_t, const ::Rectangle&)
 * @param visitor Function/lambda to call for each area
 *
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Visitor, typename>
void GeofenceMap::forEachArea(Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, int64_t, const ::Rectangle&>,
                  "Visitor must be callable with (int64_t, const ::Rectangle&)");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    for (const std::pair<const int64_t, ::Rectangle>& area_entry : snap->areas) {
        visitor(area_entry.first, area_entry.second);
    }
}

/**
 * @brief Iterates over obstacles within a bounding box with zero-copy access
 * 
 * Combines spatial query with iteration for efficiency. Only visits obstacles
 * whose bounding boxes intersect the query region.
 * 
 * @tparam Visitor Callable type: void(const GeometryEntry&)
 * @param bbox Bounding box to filter obstacles
 * @param visitor Function/lambda to call for each obstacle in region
 * 
 * @note More efficient than queryNearby() + manual getObstacle() loop
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Visitor, typename>
void GeofenceMap::forEachObstacleInRegion(
    const BoundingBox& bbox, Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, const GeometryEntry&>,
                  "Visitor must be callable with (const GeometryEntry&)");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    const std::vector<int64_t> obstacle_ids = snap->obstacle_tree->query(bbox);
    
    for (const int64_t id : obstacle_ids) {
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = 
            snap->obstacles.find(id);
        if (it != snap->obstacles.end()) {
            visitor(it->second);
        }
    }
}

/**
 * @brief Iterates over obstacles in region filtered by layer mask
 * 
 * Combines spatial query with layer filtering for efficiency. Only visits
 * obstacles whose layers match the specified mask. Eliminates need for manual
 * layer filtering in visitor lambdas.
 * 
 * @tparam Visitor Callable type: void(const GeometryEntry&)
 * @param bbox Bounding box to filter obstacles
 * @param layers Layer mask (e.g., FIXED | STATIC)
 * @param visitor Function/lambda to call for each matching obstacle
 * 
 * @note More efficient than manual hasLayer() checks in visitor
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Visitor, typename>
void GeofenceMap::forEachObstacleInRegion(
    const BoundingBox& bbox, const ObstacleLayer layers, Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, const GeometryEntry&>,
                  "Visitor must be callable with (const GeometryEntry&)");
    
    // Query static layer (FIXED + STATIC) if requested
    if (hasLayer(layers, ObstacleLayer::FIXED) || hasLayer(layers, ObstacleLayer::STATIC)) {
        const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
        const std::vector<int64_t> static_ids = snap->obstacle_tree->query(bbox);
        
        for (const int64_t id : static_ids) {
            const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = 
                snap->obstacles.find(id);
            if (it != snap->obstacles.end()) {
                if (hasLayer(layers, it->second.layer)) {
                    visitor(it->second);
                }
            }
        }
    }
    
    // Query dynamic layer if requested (fully lock-free)
    if (hasLayer(layers, ObstacleLayer::DYNAMIC)) {
        this->dynamic_quadtree_->queryGeometry(bbox, visitor);
    }
}

/**
 * @brief Finds first obstacle in region matching a predicate (early termination)
 * 
 * Iterates through obstacles in the bounding box until the predicate returns true.
 * This is more efficient than forEachObstacleInRegion when you only need to find
 * the first match and can stop early.
 * 
 * @tparam Predicate Callable type: bool(const GeometryEntry&)
 * @param bbox Bounding box to search within
 * @param predicate Function/lambda that returns true to stop, false to continue
 * @return true if completed all obstacles, false if stopped early by predicate
 * 
 * @note Predicate returns true to STOP iteration (match found)
 * @note Return value: false means "found something", true means "nothing found"
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Predicate, typename>
bool GeofenceMap::findFirstObstacleInRegion(
    const BoundingBox& bbox, Predicate&& predicate) const {
    static_assert(std::is_invocable_r_v<bool, Predicate, const GeometryEntry&>,
                  "Predicate must be callable with (const GeometryEntry&) and return bool");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    // Query spatial index for obstacle IDs in region
    const std::vector<int64_t> obstacle_ids = snap->obstacle_tree->query(bbox);
    
    // Visit each obstacle until predicate returns true (early exit)
    for (const int64_t id : obstacle_ids) {
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = 
            snap->obstacles.find(id);
        if (it != snap->obstacles.end()) {
            if (predicate(it->second)) {
                return false;  // Stopped early - predicate signaled to stop
            }
        }
    }
    
    return true;  // Completed iteration - no early exit
}

/**
 * @brief Iterates over obstacles near a point with zero-copy access
 * 
 * Convenience wrapper that converts point+radius to bounding box and
 * calls forEachObstacleInRegion.
 * 
 * @tparam Visitor Callable type: void(const GeometryEntry&)
 * @param center Center point of search region
 * @param radius Search radius
 * @param visitor Function/lambda to call for each nearby obstacle
 * 
 * @note Circular region approximated by square bounding box
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Visitor, typename>
void GeofenceMap::forEachObstacleNearby(
    const Point2D& center, const float radius, Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, const GeometryEntry&>,
                  "Visitor must be callable with (const GeometryEntry&)");
    
    // Convert point+radius to bounding box
    const BoundingBox bbox{
        static_cast<float>(center.x - radius),
        static_cast<float>(center.y - radius),
        static_cast<float>(center.x + radius),
        static_cast<float>(center.y + radius)
    };
    
    this->forEachObstacleInRegion(bbox, std::forward<Visitor>(visitor));
}

/**
 * @brief Iterates over areas within a bounding box with zero-copy access
 * 
 * Combines spatial query with iteration for efficiency. Only visits areas
 * whose rectangles intersect the query region.
 * 
 * @tparam Visitor Callable type: void(int64_t, const ::Rectangle&)
 * @param bbox Bounding box to filter areas
 * @param visitor Function/lambda to call for each area in region
 *
 * @note More efficient than manual spatial query + getArea() loop
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Visitor, typename>
void GeofenceMap::forEachAreaInRegion(
    const BoundingBox& bbox, Visitor&& visitor) const {
    static_assert(std::is_invocable_v<Visitor, int64_t, const ::Rectangle&>,
                  "Visitor must be callable with (int64_t, const ::Rectangle&)");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    // Query spatial index for area IDs in region
    const std::vector<int64_t> area_ids = snap->area_tree->query(bbox);
    
    // Visit each area (zero-copy)
    for (const int64_t id : area_ids) {
        const std::unordered_map<int64_t, ::Rectangle>::const_iterator it = 
            snap->areas.find(id);
        if (it != snap->areas.end()) {
            visitor(id, it->second);
        }
    }
}

/**
 * @brief Finds first area in region matching a predicate (early termination)
 * 
 * Iterates through areas in the bounding box until the predicate returns true.
 * This is more efficient than forEachAreaInRegion when you only need to find
 * the first match and can stop early.
 * 
 * @tparam Predicate Callable type: bool(int64_t, const Rectangle&)
 * @param bbox Bounding box to search within  
 * @param predicate Function/lambda that returns true to stop, false to continue
 * @return true if completed all areas, false if stopped early by predicate
 * 
 * @note Predicate returns true to STOP iteration (match found)
 * @note Return value: false means "found something", true means "nothing found"
 * @note Thread-safe concurrent reads via RCU snapshots
 */
template<typename Predicate, typename>
bool GeofenceMap::findFirstAreaInRegion(
    const BoundingBox& bbox, Predicate&& predicate) const {
    static_assert(std::is_invocable_r_v<bool, Predicate, int64_t, const ::Rectangle&>,
                  "Predicate must be callable with (int64_t, const ::Rectangle&) and return bool");
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    // Query spatial index for area IDs in region
    const std::vector<int64_t> area_ids = snap->area_tree->query(bbox);
    
    // Visit each area until predicate returns true (early exit)
    for (const int64_t id : area_ids) {
        const std::unordered_map<int64_t, ::Rectangle>::const_iterator it = 
            snap->areas.find(id);
        if (it != snap->areas.end()) {
            if (predicate(id, it->second)) {
                return false;  // Stopped early - predicate signaled to stop
            }
        }
    }
    
    return true;  // Completed iteration - no early exit
}

// ============================================================================
// Optimized Spatial Query Implementations
// ============================================================================

/**
 * @brief Checks if a line segment intersects any obstacle
 * 
 * Two-phase algorithm for efficiency:
 * 1. Spatial index query: Find candidate obstacles via bounding box overlap
 * 2. Precise collision: Test each candidate using backend-specific collision
 * 
 * This approach is much faster than checking all obstacles, especially for
 * large maps where most obstacles are far from the query segment.
 * 
 * @param p1 First endpoint of the line segment
 * @param p2 Second endpoint of the line segment
 * @return true if segment intersects any obstacle geometry
 * 
 * @note Backend-agnostic: Uses variant-based collision detection
 * @note Thread-safe: Reads from immutable snapshot
 */
inline bool GeofenceMap::segmentIntersectsObstacle(
    const Point2D& p1, const Point2D& p2) const {
    
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    if (snap->obstacles.empty()) {
        return false;
    }
    
    // Compute bounding box of line segment for spatial query (with validation)
    const BoundingBox query_box = createValidatedBBox(p1, p2);
    
    // Phase 1: Query spatial index for candidates
    const std::vector<int64_t> candidates = snap->obstacle_tree->query(query_box);
    
    // Phase 2: Precise collision detection on candidates
    for (const int64_t id : candidates) {
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = snap->obstacles.find(id);
        if (it != snap->obstacles.end()) {
            const Geometry& geom = it->second.geometry;
            
            // Create line segment geometry using float coordinates
            // (Avoids constructor ambiguity between backends)
            const rises::geofence::Line query_line = rises::geofence::makeLine(
                static_cast<float>(p1.x), static_cast<float>(p1.y),
                static_cast<float>(p2.x), static_cast<float>(p2.y)
            );
            
            // Use variant-based collision detection with explicit namespace
            const bool overlaps = std::visit([&query_line](const auto& obstacle) {
                return rises::geofence::collision(query_line, obstacle);
            }, geom);
            
            if (overlaps) {
                return true;
            }
        }
    }
    
    return false;
}

/**
 * @brief Finds distance to the nearest obstacle using expanding search\n * \n * Implements an efficient expanding radius search algorithm:
 * 1. Start with small search radius (0.5m)
 * 2. Query spatial index for nearby obstacles
 * 3. If found, compute actual distances; if not, double radius and retry
 * 4. Continue until obstacles found or max radius (100m) reached
 * \n * This approach is much faster than computing distance to every obstacle,
 * especially when the nearest obstacle is close (common case).
 * \n * @param p Query point
 * @return Distance to nearest obstacle in meters, or infinity if none within 100m
 * \n * @note Uses bounding box center for distance (approximation)
 * @note Thread-safe via immutable snapshot reads
 * @note Performance: O(log N) average case due to spatial indexing
 */
inline float GeofenceMap::distanceToNearestObstacle(const Point2D& p) const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    if (snap->obstacles.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    
    // Expanding search: start small, double radius until we find something
    float search_radius = 0.5f;  // Start with 0.5m
    const float max_radius = 100.0f;  // Don't search beyond 100m
    
    while (search_radius <= max_radius) {
        const Point2D p1{p.x - search_radius, p.y - search_radius};
        const Point2D p2{p.x + search_radius, p.y + search_radius};
        const BoundingBox query_box = createValidatedBBox(p1, p2);
        
        const std::vector<int64_t> candidates = snap->obstacle_tree->query(query_box);
        
        if (!candidates.empty()) {
            // Found candidates - compute actual squared distances (optimization: single sqrt at end)
            float min_dist_sq = std::numeric_limits<float>::infinity();
            
            for (const int64_t id : candidates) {
                const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = snap->obstacles.find(id);
                if (it != snap->obstacles.end()) {
                    const Geometry& geom = it->second.geometry;
                    
                    // Use squared distance to avoid sqrt in loop (performance optimization)
                    const float dist_sq = distanceSquaredToPoint(geom, p);
                    
                    min_dist_sq = std::min(min_dist_sq, dist_sq);
                }
            }
            
            return std::sqrt(min_dist_sq);  // Single sqrt after finding minimum
        }
        
        // No results in this radius, expand search
        search_radius *= 2.0f;
    }
    
    return std::numeric_limits<float>::infinity();
}

// ============================================================================
// Dynamic Obstacle Layer Operations
// ============================================================================

inline bool GeofenceMap::insertDynamicObstacle(
    int64_t id, Geometry geometry, std::chrono::milliseconds ttl)
{
    (void)ttl;  // TTL not yet implemented for Quad-tree approach
    // Create geometry entry with DYNAMIC layer marker
    GeometryEntry entry(id, geometry, ObstacleLayer::DYNAMIC);
    
    // 100% lock-free insert with embedded geometry
    this->dynamic_quadtree_->insert(entry);
    
    return true;
}

inline bool GeofenceMap::removeDynamicObstacle(int64_t id) {
    // 100% lock-free removal with geometry search
    return this->dynamic_quadtree_->removeGeometry(id);
}

inline std::size_t GeofenceMap::removeExpiredDynamicObstacles() {
    // TTL-based expiration is not active for the Quad-tree approach.
    // Dynamic obstacles are managed explicitly via removeDynamicObstacle() and clearDynamicObstacles().
    return 0;
}

inline void GeofenceMap::clearDynamicObstacles() {
    // 100% lock-free clear operation
    this->dynamic_quadtree_->clear();
}

inline std::size_t GeofenceMap::getDynamicObstacleCount() const {
    // Lock-free atomic read from Quad-tree
    return this->dynamic_quadtree_->size();
}

// Note: Batch query optimizations and other complex algorithms should be
// implemented in separate policy classes (like PathSafetyChecker) rather than
// in this data structure class.

// ============================================================================
// Snapshot Update (must be in header for member-template instantiation)
// ============================================================================

/**
 * @brief Atomically updates the snapshot using copy-on-write (RCU pattern)
 *
 * @tparam Modifier Callable type: void(Snapshot&)
 * @param modifier Function that modifies the snapshot in-place
 *
 * @note Thread-safe: Multiple concurrent writers are serialized via mutex
 * @note Readers never block and always see consistent state
 */
template<typename Modifier, typename>
void GeofenceMap::updateSnapshot(Modifier&& modifier) {
    static_assert(std::is_invocable_v<Modifier, Snapshot&>,
                  "Modifier must be callable with (Snapshot&)");

    std::lock_guard<std::mutex> lock(this->static_mutex_);

    const std::shared_ptr<const Snapshot> old_snap = this->snapshot_.load();

    // Clone the spatial indices for the new snapshot
    const std::shared_ptr<SpatialIndex> cloned_obstacle_tree = old_snap->obstacle_tree ?
        std::static_pointer_cast<SpatialIndex>(old_snap->obstacle_tree->clone()) : nullptr;
    const std::shared_ptr<SpatialIndex> cloned_area_tree = old_snap->area_tree ?
        std::static_pointer_cast<SpatialIndex>(old_snap->area_tree->clone()) : nullptr;

    // Acquire snapshot from pool (30-50% faster than heap allocation)
    const std::shared_ptr<Snapshot> new_snap = this->snapshot_pool_.acquire();
    new_snap->obstacle_tree = cloned_obstacle_tree;
    new_snap->obstacles = old_snap->obstacles;
    new_snap->area_tree = cloned_area_tree;
    new_snap->areas = old_snap->areas;
    new_snap->locked_areas = old_snap->locked_areas;
    new_snap->contours = old_snap->contours;

    // Apply modifications
    std::forward<Modifier>(modifier)(*new_snap);

    // Build spatial indices BEFORE publishing.  NanoflannAdapter's ensureBuilt()
    // is not thread-safe (modifies mutable members); multiple concurrent reader
    // threads calling it on the same snapshot races on kdtree_ construction and
    // causes heap corruption.  Building here, in the single writer thread while
    // the snapshot is still private, eliminates the race entirely.
    if (cloned_obstacle_tree) {
        cloned_obstacle_tree->ensureBuilt();
    }
    if (cloned_area_tree) {
        cloned_area_tree->ensureBuilt();
    }

    // Atomic update - readers switch to new snapshot atomically
    this->snapshot_.store(new_snap);
}

} // namespace geofence
} // namespace rises
