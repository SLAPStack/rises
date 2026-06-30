/**
 * @file geofence_map_variant.cpp
 * @brief Core implementation of the variant-based geofence map
 * 
 * This file implements the GeofenceMap class, which provides a
 * thread-safe, backend-agnostic spatial data structure for obstacles,
 * navigation areas, and warehouse boundaries.
 * 
 * Key Architecture Patterns:
 * - RCU (Read-Copy-Update): Lock-free reads, copy-on-write updates
 * - Variant-based polymorphism: Zero-cost type-safe geometry handling
 * - Compile-time backend selection: Eigen, Boost, or CUDA geometry
 * - Spatial indexing: Pluggable spatial index backends (Nanoflann, Boost, libspatialindex)
 * 
 * Thread Safety:
 * - Reads: Lock-free, multiple concurrent readers
 * - Writes: Copy-on-write with atomic snapshot swap
 * - External synchronization required for concurrent writes
 * 
 * @see GeofenceMap class for detailed API documentation
 */

#include <functional>
#include <stdexcept>
#include "geofence/spatial/map/geofence_map.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include "geofence/spatial/shape/contour.hpp"

namespace rises {
namespace geofence {

// ============================================================================
// GeofenceThreadPool Static Member Initialization
// ============================================================================

std::atomic<bool> GeofenceThreadPool::configured_(false);
std::size_t GeofenceThreadPool::configured_thread_count_ = 0;

// ============================================================================
// Constructor
// ============================================================================

/**
 * @brief Constructs an empty geofence map with specified spatial index factory
 * 
 * Initializes the map with empty obstacle and area collections, creating
 * spatial indices using the provided factory. The initial snapshot contains
 * no warehouse contours.
 * 
 * @param factory Function that creates spatial index instances
 * @throws std::runtime_error if factory returns nullptr
 * 
 * @note The factory will be called multiple times (for obstacle and area trees)
 * @note All spatial indices must implement SpatialIndexInterface
 */
GeofenceMap::GeofenceMap(
    const std::function<std::shared_ptr<SpatialIndex>()>& factory)
{
    // Initialize with empty snapshot
    std::shared_ptr<SpatialIndex> obs_tree = factory();
    std::shared_ptr<SpatialIndex> area_tree = factory();
    
    // Validate factory produced valid indices
    if (!obs_tree || !area_tree) {
        throw std::runtime_error("Spatial index factory returned nullptr");
    }
    
    // Acquire snapshot from pool (30-50% faster than heap allocation)
    std::shared_ptr<Snapshot> initial_snapshot = this->snapshot_pool_.acquire();
    initial_snapshot->obstacle_tree = obs_tree;
    initial_snapshot->obstacles.clear();
    initial_snapshot->area_tree = area_tree;
    initial_snapshot->areas.clear();
    initial_snapshot->locked_areas.clear();
    initial_snapshot->contours = nullptr;
    
    this->snapshot_.store(initial_snapshot);
    
    // Initialize lock-free dynamic obstacle Quad-tree
    // World bounds: -100m to +100m (adjust as needed for your environment)
    const BoundingBox world_bounds{-100.0f, -100.0f, 100.0f, 100.0f};
    this->dynamic_quadtree_ = std::make_shared<SimpleQuadTree>(world_bounds, 16);
}

// ============================================================================
// Snapshot Implementation
// ============================================================================

/**
 * @brief Constructs an immutable snapshot of the geofence map state
 * 
 * A snapshot captures a consistent view of all map data at a single point
 * in time. Once created, snapshots are immutable and can be safely shared
 * across multiple reader threads.
 * 
 * @param obs_tree Spatial index for obstacle queries
 * @param obs Map of obstacle IDs to geometry entries
 * @param area_tree Spatial index for area queries
 * @param areas_map Map of area IDs to rectangle definitions
 * @param locked Set of locked area IDs
 * @param cont Warehouse boundary contours (may be null)
 */
GeofenceMap::Snapshot::Snapshot(
    std::shared_ptr<SpatialIndex> obs_tree,
    std::unordered_map<int64_t, GeometryEntry> obs,
    std::shared_ptr<SpatialIndex> area_tree,
    std::unordered_map<int64_t, ::Rectangle> areas_map,
    std::unordered_set<int64_t> locked,
    std::shared_ptr<shape::MapBoundaryContours> cont)
    : obstacle_tree(obs_tree)
    , obstacles(std::move(obs))
    , area_tree(area_tree)
    , areas(std::move(areas_map))
    , locked_areas(std::move(locked))
    , contours(cont)
{}

// ============================================================================
// Query Methods - Non-template implementations
// ============================================================================

/**
 * @brief Queries obstacles within a circular region around a point
 * 
 * Uses spatial index for efficient candidate selection, then filters by
 * actual Euclidean distance to obstacle bounding box centers.
 * 
 * @param point Center of the query region
 * @param radius Search radius in meters
 * @return Vector of obstacle IDs within the specified radius
 * 
 * @note Uses bounding box centers for distance calculation (approximation)
 * @note Returns empty vector if no obstacles exist
 * @note Thread-safe via immutable snapshot reads
 */
std::vector<int64_t> GeofenceMap::queryNearby(
    const Point2D& point, const float radius) const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    
    if (snap->obstacles.empty()) {
        return {};
    }
    
    // Create query box around point (validated)
    const Point2D p1{point.x - radius, point.y - radius};
    const Point2D p2{point.x + radius, point.y + radius};
    const BoundingBox query_box = createValidatedBBox(p1, p2);
    
    // Query spatial index for candidates (conservative, uses AABB)
    const std::vector<int64_t> candidates = snap->obstacle_tree->query(query_box);
    
    // Filter by actual distance (bbox query is conservative)
    std::vector<int64_t> results;
    const float radius_sq = radius * radius;
    
    for (const int64_t id : candidates) {
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = snap->obstacles.find(id);
        if (it != snap->obstacles.end()) {
            const ::Rectangle& bbox = it->second.bbox;
            
            // Check distance from point to bbox center (approximate)
            const float cx = (bbox.min.x + bbox.max.x) * 0.5f;
            const float cy = (bbox.min.y + bbox.max.y) * 0.5f;
            const float dx = cx - static_cast<float>(point.x);
            const float dy = cy - static_cast<float>(point.y);
            const float dist_sq = dx*dx + dy*dy;
            
            if (dist_sq <= radius_sq) {
                results.push_back(id);
            }
        }
    }
    
    return results;
}

/**
 * @brief Retrieves obstacle geometry by ID
 * 
 * @param id Obstacle identifier
 * @return Pointer to geometry variant, or nullptr if not found
 * 
 * @note Returned pointer is valid only while the current snapshot is alive
 * @note Thread-safe via immutable snapshot reads
 */
const Geometry* GeofenceMap::getObstacle(const int64_t id) const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = snap->obstacles.find(id);
    if (it != snap->obstacles.end()) {
        return &(it->second.geometry);
    }
    return nullptr;
}

/**
 * @brief Retrieves navigation area rectangle by ID
 * 
 * @param id Area identifier
 * @return Pointer to area rectangle, or nullptr if not found
 * 
 * @note Returned pointer is valid only while the current snapshot is alive
 * @note Thread-safe via immutable snapshot reads
 */
const ::Rectangle* GeofenceMap::getArea(const int64_t id) const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    const std::unordered_map<int64_t, ::Rectangle>::const_iterator it = snap->areas.find(id);
    if (it != snap->areas.end()) {
        return &(it->second);
    }
    return nullptr;
}

/**
 * @brief Checks if a navigation area is locked
 * 
 * Locked areas are temporarily inaccessible, typically due to other
 * vehicle occupancy or maintenance.
 * 
 * @param id Area identifier
 * @return true if area is locked
 * 
 * @note Thread-safe via immutable snapshot reads
 */
bool GeofenceMap::isAreaLocked(const int64_t id) const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    return snap->locked_areas.count(id) > 0;
}

/**
 * @brief Retrieves all obstacle IDs currently in the map
 * 
 * @return Vector of all obstacle identifiers
 * 
 * @note Thread-safe via immutable snapshot reads
 * @note Order is unspecified (depends on unordered_map iteration)
 */
std::vector<int64_t> GeofenceMap::getAllObstacleIds() const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    std::vector<int64_t> ids;
    ids.reserve(snap->obstacles.size());
    for (const std::pair<const int64_t, GeometryEntry>& obstacle_entry : snap->obstacles) {
        ids.push_back(obstacle_entry.first);
    }
    return ids;
}

// ============================================================================
// Mutation Methods
// ============================================================================

/**
 * @brief Inserts a new obstacle into the map
 * 
 * Creates a copy-on-write snapshot with the new obstacle added to both
 * the spatial index and the geometry storage.
 * 
 * @param id Unique obstacle identifier
 * @param geom Geometry variant (Point, Line, Rectangle, or Polygon)
 * 
 * @note If ID already exists, it will be replaced
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 * @note Readers see atomic update via snapshot swap
 */
void GeofenceMap::insertObstacle(const int64_t id, Geometry geom) {
    // Input validation
    if (id < 0) {
        throw std::invalid_argument("Obstacle ID must be non-negative");
    }
    
    // Validate geometry has valid bounding box
    const BoundingBox test_bbox = getBoundingBox(geom);
    if (!test_bbox.isValid()) {
        throw std::invalid_argument("Invalid geometry: bounding box has min > max");
    }
    
    // Check for NaN or infinity in bounding box
    if (!std::isfinite(test_bbox.min_x) || !std::isfinite(test_bbox.min_y) ||
        !std::isfinite(test_bbox.max_x) || !std::isfinite(test_bbox.max_y)) {
        throw std::invalid_argument("Invalid geometry: bounding box contains non-finite values");
    }
    
    this->updateSnapshot([id, geom = std::move(geom)](Snapshot& snap) mutable {
        // If obstacle with this ID exists, remove it first
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator existing = snap.obstacles.find(id);
        if (existing != snap.obstacles.end()) {
            // Get the old bounding box for removal from spatial index
            const BoundingBox old_bbox{
                static_cast<float>(existing->second.bbox.min.x),
                static_cast<float>(existing->second.bbox.min.y),
                static_cast<float>(existing->second.bbox.max.x),
                static_cast<float>(existing->second.bbox.max.y)
            };
            snap.obstacle_tree->remove(id, old_bbox);
            snap.obstacles.erase(existing);
        }
        
        GeometryEntry entry(id, std::move(geom));
        
        // Convert ::Rectangle bbox to BoundingBox for spatial index
        const BoundingBox bbox{
            static_cast<float>(entry.bbox.min.x),
            static_cast<float>(entry.bbox.min.y),
            static_cast<float>(entry.bbox.max.x),
            static_cast<float>(entry.bbox.max.y)
        };
        
        // Insert into spatial index
        snap.obstacle_tree->insert(id, bbox);

        snap.obstacles.emplace(id, std::move(entry));
    });

    // Visualization handled in node;
}

/**
 * @brief Removes an obstacle from the map
 * 
 * Creates a copy-on-write snapshot with the obstacle removed from both
 * the spatial index and the geometry storage. No-op if ID doesn't exist.
 * 
 * @param id Obstacle identifier to remove
 * 
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 * @note Readers see atomic update via snapshot swap
 */
void GeofenceMap::removeObstacle(const int64_t id) {
    // Input validation
    if (id < 0) {
        throw std::invalid_argument("Obstacle ID must be non-negative");
    }
    
    this->updateSnapshot([id](Snapshot& snap) {
        const std::unordered_map<int64_t, GeometryEntry>::const_iterator it = snap.obstacles.find(id);
        if (it != snap.obstacles.end()) {
            // Convert ::Rectangle bbox to BoundingBox
            const BoundingBox bbox{
                static_cast<float>(it->second.bbox.min.x),
                static_cast<float>(it->second.bbox.min.y),
                static_cast<float>(it->second.bbox.max.x),
                static_cast<float>(it->second.bbox.max.y)
            };
            // Remove from spatial index
            snap.obstacle_tree->remove(id, bbox);
            snap.obstacles.erase(it);
        }
    });

    // Visualization handled in node;
}

/**
 * @brief Registers a navigation area in the map
 * 
 * Navigation areas define valid regions for robot movement. Areas can
 * be locked/unlocked dynamically.
 * 
 * @param id Unique area identifier
 * @param area Rectangle defining the area boundaries
 * 
 * @note If ID already exists, it will be replaced
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 * @note Readers see atomic update via snapshot swap
 */
void GeofenceMap::registerArea(const int64_t id, const ::Rectangle area) {
    // Input validation
    if (id < 0) {
        throw std::invalid_argument("Area ID must be non-negative");
    }
    
    // Validate area rectangle
    if (area.min.x > area.max.x || area.min.y > area.max.y) {
        throw std::invalid_argument("Invalid area: min > max");
    }
    
    if (!std::isfinite(area.min.x) || !std::isfinite(area.min.y) ||
        !std::isfinite(area.max.x) || !std::isfinite(area.max.y)) {
        throw std::invalid_argument("Invalid area: contains non-finite values");
    }
    
    this->updateSnapshot([id, area](Snapshot& snap) {
        // Create bounding box for spatial index
        const BoundingBox bbox{
            static_cast<float>(area.min.x), static_cast<float>(area.min.y),
            static_cast<float>(area.max.x), static_cast<float>(area.max.y)
        };
        // Insert into spatial index
        snap.area_tree->insert(id, bbox);
        snap.areas[id] = area;
    });

    // Visualization handled in node;
}

/**
 * @brief Locks a navigation area, making it temporarily inaccessible
 * 
 * Locked areas are excluded from path planning and occupancy checks.
 * Typically used when another vehicle occupies the area.
 * 
 * @param id Area identifier to lock
 * 
 * @note No-op if area doesn't exist or is already locked
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 */
void GeofenceMap::lockArea(const int64_t id) {
    // Input validation
    if (id < 0) {
        throw std::invalid_argument("Area ID must be non-negative");
    }
    
    this->updateSnapshot([id](Snapshot& snap) {
        snap.locked_areas.insert(id);
    });

    // Visualization handled in node;
}

/**
 * @brief Unlocks a navigation area, making it accessible again
 * 
 * @param id Area identifier to unlock
 * 
 * @note No-op if area doesn't exist or is not locked
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 */
void GeofenceMap::unlockArea(const int64_t id) {
    // Input validation
    if (id < 0) {
        throw std::invalid_argument("Area ID must be non-negative");
    }
    
    this->updateSnapshot([id](Snapshot& snap) {
        snap.locked_areas.erase(id);
    });

    // Visualization handled in node;
}

/**
 * @brief Sets the map boundary contours
 * 
 * Map contours define the valid operating area. Points outside
 * these contours are considered out-of-bounds.
 * 
 * @param contours Map boundary polygon(s)
 * 
 * @note Not thread-safe with concurrent writes (requires external synchronization)
 * @note Readers see atomic update via snapshot swap
 */
void GeofenceMap::setMapContours(
    const shape::MapBoundaryContours& contours) {
    this->updateSnapshot([contours](Snapshot& snap) {
        snap.contours = std::make_shared<shape::MapBoundaryContours>(contours);
    });
}

/**
 * @brief Retrieves the map boundary contours
 * 
 * @return Pointer to map contours, or nullptr if not set
 * 
 * @note Returned pointer is valid only while the current snapshot is alive
 * @note Thread-safe via immutable snapshot reads
 */
const shape::MapBoundaryContours* GeofenceMap::getMapContours() const {
    const std::shared_ptr<const Snapshot> snap = this->snapshot_.load();
    return snap->contours.get();
}

// updateSnapshot is defined in geofence_map_impl.hpp (header) because it is a
// member template that must be instantiable for arbitrary Modifier types
// (e.g. lambdas from BatchOperations).

} // namespace geofence
} // namespace rises

