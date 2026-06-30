#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <future>
#include <atomic>
#include <algorithm>
#include <cstdlib>
#include "geofence/common/geometry/variant_geometry.hpp"
#include "geometry_types.hpp"
#include "geofence/utils/rcu.hpp"
#include "geofence/spatial/map/config.hpp"
#include "geofence/spatial/map/hierarchical_spatial_index.hpp"
#include "geofence/spatial/map/snapshot_memory_pool.hpp"
#include "spatial_index_selection.hpp"

namespace rises {
namespace shape { class MapBoundaryContours; }

namespace geofence {

// ============================================================================
// Thread Pool (Lazy Initialization - Only for Parallel Execution)
// ============================================================================

/**
 * @brief Static thread pool for parallel batch operations
 * 
 * This pool is only instantiated when ExecutionPolicy::par is actually used.
 * Lazy initialization ensures zero overhead for sequential-only workloads.
 * 
 * Thread count is configurable via:
 * 1. Environment variable: GEOFENCE_THREAD_COUNT (checked at first use)
 * 2. Explicit call to configure() before first ExecutionPolicy::par use
 * 3. Default: std::thread::hardware_concurrency() / 2 (at least 1)
 * 
 * @note Singleton pattern with automatic cleanup on program exit
 * @note Thread count cannot be changed after first use (lazy init is final)
 */
class GeofenceThreadPool {
public:
    static GeofenceThreadPool& instance() {
        static GeofenceThreadPool pool(getThreadCount());
        return pool;
    }
    
    /**
     * @brief Configure thread count BEFORE first use
     * 
     * This must be called before the first ExecutionPolicy::par usage,
     * otherwise it has no effect (singleton already initialized).
     * 
     * @param num_threads Number of worker threads (0 = auto-detect)
     * 
     * @note Thread-safe: Uses atomic flag to ensure one-time configuration
     */
    static void configure(std::size_t num_threads) {
        bool expected = false;
        if (configured_.compare_exchange_strong(expected, true)) {
            configured_thread_count_ = (num_threads > 0) ? num_threads : getDefaultThreadCount();
        }
    }
    
    template<typename Func>
    std::future<void> enqueue(Func&& f) {
        std::packaged_task<void()> task(std::forward<Func>(f));
        std::future<void> result = task.get_future();
        
        {
            std::unique_lock<std::mutex> lock(this->queue_mutex_);
            this->tasks_.emplace_back(std::move(task));
        }
        this->condition_.notify_one();
        return result;
    }
    
    // Deleted copy/move constructors
    GeofenceThreadPool(const GeofenceThreadPool&) = delete;
    GeofenceThreadPool& operator=(const GeofenceThreadPool&) = delete;
    GeofenceThreadPool(GeofenceThreadPool&&) = delete;
    GeofenceThreadPool& operator=(GeofenceThreadPool&&) = delete;
    
private:
    static std::size_t getDefaultThreadCount() {
        const unsigned int hw_threads = std::thread::hardware_concurrency();
        // Use half of available cores (leave headroom for other processes)
        return (hw_threads > 0) ? std::max(1u, hw_threads / 2) : 2;
    }
    
    static std::size_t getThreadCount() {
        // Priority 1: Explicit configuration
        if (configured_.load()) {
            return configured_thread_count_;
        }
        
        // Priority 2: Environment variable
        const char* env_threads = std::getenv("GEOFENCE_THREAD_COUNT");
        if (env_threads != nullptr) {
            const int count = std::atoi(env_threads);
            if (count > 0) {
                return static_cast<std::size_t>(count);
            }
        }
        
        // Priority 3: Auto-detect
        return getDefaultThreadCount();
    }
    
    explicit GeofenceThreadPool(std::size_t num_threads) : stop_(false) {
        for (std::size_t i = 0; i < num_threads; ++i) {
            this->workers_.emplace_back([this] {
                while (true) {
                    std::packaged_task<void()> task;
                    
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex_);
                        this->condition_.wait(lock, [this] {
                            return this->stop_ || !this->tasks_.empty();
                        });
                        
                        if (this->stop_ && this->tasks_.empty()) {
                            return;
                        }
                        
                        task = std::move(this->tasks_.front());
                        this->tasks_.pop_front();
                    }
                    
                    task();
                }
            });
        }
    }
    
    ~GeofenceThreadPool() {
        {
            std::unique_lock<std::mutex> lock(this->queue_mutex_);
            this->stop_ = true;
        }
        this->condition_.notify_all();
        for (std::thread& worker : this->workers_) {
            worker.join();
        }
    }
    
    std::vector<std::thread> workers_;
    std::deque<std::packaged_task<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_;
    
    // Static configuration (must be initialized in .cpp)
    static std::atomic<bool> configured_;
    static std::size_t configured_thread_count_;
};

// ============================================================================
// GeofenceMap: Dual-Layer Spatial Data Structure
// ============================================================================

/**
 * @brief Spatial data structure with static and dynamic obstacle layers
 * 
 * Provides high-performance collision detection and spatial queries for
 * robotic navigation with dual-layer architecture:
 * - Static layer: Persistent map obstacles (R-tree spatial index)
 * - Dynamic layer: Time-limited obstacles (lock-free RCU circular buffer)
 * 
 * This class ONLY provides primitive geometric facts:
 * - What shapes exist
 * - Where they are
 * - Spatial relationships (intersects, contains, distance)
 * 
 * It does NOT make high-level decisions like "is this path safe?"
 * Those belong in separate checker classes.
 * 
 * 
 * @note Thread Safety:
 *   - Read operations (queries, iteration) are lock-free and never block
 *   - Multiple readers can operate concurrently without synchronization
 *   - Static layer: Uses RCU pattern for copy-on-write updates
 *   - Dynamic layer: Uses RCU pattern with lock-free circular buffer
 *   - Concurrent writes require external synchronization
 *   - Each reader sees a consistent immutable snapshot
 * 
 * @note Performance:
 *   - Static layer queries: O(log N + K) spatial index query
 *   - Dynamic layer queries: O(log N + K) Quad-tree query
 *   - Mutations: O(N) due to copy-on-write snapshot
 *   - Use updateSnapshot() for bulk operations to amortize copy cost
 */
// Forward declaration for friend access
class BatchOperations;

class GeofenceMap {
public:

    using SpatialIndexFactory = std::function<std::shared_ptr<SpatialIndex>>();
    
    explicit GeofenceMap(const std::function<std::shared_ptr<SpatialIndex>()>& factory);
    
    // ============================================================================
    // Primitive Queries (Mechanism - what IS true about space)
    // ============================================================================
    
    /**
     * @brief Check if line segment intersects any obstacle
     * @param start Start point
     * @param end End point
     * @return true if segment intersects any shape
     */
    [[nodiscard]] bool segmentIntersectsObstacle(const Point2D& start, const Point2D& end) const;
    
    /**
     * @brief Get distance to nearest obstacle from point
     * @param point Query point
     * @return Distance to nearest obstacle (positive)
     */
    [[nodiscard]] float distanceToNearestObstacle(const Point2D& point) const;
    
    /**
     * @brief Query obstacles near a point
     * @param point Center point
     * @param radius Search radius
     * @return Span of geometry IDs within radius
     */
    [[nodiscard]] std::vector<int64_t> queryNearby(const Point2D& point, float radius) const;
    
    /**
     * @brief Get obstacle geometry by ID
     * @param id Obstacle identifier
     * @return Pointer to geometry, or nullptr if not found
     */
    [[nodiscard]] const Geometry* getObstacle(int64_t id) const;
    
    /**
     * @brief Get area rectangle by ID
     * @param id Area identifier  
     * @return Pointer to area rectangle, or nullptr if not found
     */
    [[nodiscard]] const ::Rectangle* getArea(int64_t id) const;
    
    /**
     * @brief Check if area is locked
     * @param id Area identifier
     * @return true if area is locked
     */
    [[nodiscard]] bool isAreaLocked(int64_t id) const;
    
    // ============================================================================
    // Zero-Cost Iteration (Read-only access)
    // ============================================================================
    
    /**
     * @brief Visit all obstacles (zero-copy callback)
     * @param visitor Callable (const GeometryEntry&) -> void
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, const GeometryEntry&>>>
    void forEachObstacle(Visitor&& visitor) const;
    
    /**
     * @brief Visit obstacles within a bounding box (zero-copy callback)
     * @param bbox Bounding box to filter obstacles
     * @param visitor Callable (const GeometryEntry&) -> void
     * 
     * @note More efficient than manual queryNearby + getObstacle loop
     * @note Only visits obstacles whose bounding boxes intersect the query box
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, const GeometryEntry&>>>
    void forEachObstacleInRegion(const BoundingBox& bbox, Visitor&& visitor) const;
    
    /**
     * @brief Visit obstacles in region filtered by layer
     * @param bbox Bounding box to filter obstacles
     * @param layers Layer mask (e.g., ObstacleLayer::FIXED | ObstacleLayer::STATIC)
     * @param visitor Callable (const GeometryEntry&) -> void
     * 
     * @note More efficient than manual layer filtering in lambda
     * @note Visits only obstacles matching specified layers
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, const GeometryEntry&>>>
    void forEachObstacleInRegion(const BoundingBox& bbox, ObstacleLayer layers, Visitor&& visitor) const;
    
    /**
     * @brief Visit obstacles in region with early termination support
     * @param bbox Bounding box to filter obstacles
     * @param predicate Callable (const GeometryEntry&) -> bool
     * @return true if iteration completed, false if stopped early
     * 
     * @note Predicate should return true to stop iteration, false to continue
     * @note Returns true if all obstacles were visited, false if stopped early
     * @note Useful for "find first" patterns where you stop at first match
     */
    template<typename Predicate,
             typename = std::enable_if_t<std::is_invocable_r_v<bool, Predicate, const GeometryEntry&>>>
    [[nodiscard]] bool findFirstObstacleInRegion(const BoundingBox& bbox, Predicate&& predicate) const;
    
    /**
     * @brief Visit obstacles near a point (zero-copy callback)
     * @param center Center point of search region
     * @param radius Search radius
     * @param visitor Callable (const GeometryEntry&) -> void
     * 
     * @note Convenience wrapper that converts point+radius to bounding box
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, const GeometryEntry&>>>
    void forEachObstacleNearby(const Point2D& center, float radius, Visitor&& visitor) const;
    
    /**
     * @brief Visit all areas (zero-copy callback)
     * @param visitor Callable (int64_t id, const ::Rectangle& area) -> void
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, int64_t, const ::Rectangle&>>>
    void forEachArea(Visitor&& visitor) const;

    /**
     * @brief Visit areas within a bounding box (zero-copy callback)
     * @param bbox Bounding box to filter areas
     * @param visitor Callable (int64_t id, const ::Rectangle& area) -> void
     *
     * @note More efficient than manual spatial query + getArea loop
     */
    template<typename Visitor,
             typename = std::enable_if_t<std::is_invocable_v<Visitor, int64_t, const ::Rectangle&>>>
    void forEachAreaInRegion(const BoundingBox& bbox, Visitor&& visitor) const;

    /**
     * @brief Visit areas in region with early termination support
     * @param bbox Bounding box to filter areas
     * @param predicate Callable (int64_t id, const ::Rectangle& area) -> bool
     * @return true if iteration completed, false if stopped early
     * 
     * @note Predicate should return true to stop iteration, false to continue
     * @note Returns true if all areas were visited, false if stopped early
     * @note Useful for "find first" patterns like finding first locked area
     */
    template<typename Predicate,
             typename = std::enable_if_t<std::is_invocable_r_v<bool, Predicate, int64_t, const ::Rectangle&>>>
    [[nodiscard]] bool findFirstAreaInRegion(const BoundingBox& bbox, Predicate&& predicate) const;
    
    /**
     * @brief Get all obstacle IDs (for batched queries)
     * @return Span of obstacle IDs
     */
    [[nodiscard]] std::vector<int64_t> getAllObstacleIds() const;
    
    // ============================================================================
    // Mutation (Thread-safe via RCU)
    // ============================================================================
    
    /**
     * @brief Insert new obstacle
     * @param id Unique obstacle identifier
     * @param geom Geometry variant
     */
    void insertObstacle(int64_t id, Geometry geom);
    
    /**
     * @brief Remove obstacle
     * @param id Obstacle identifier
     */
    void removeObstacle(int64_t id);
    
    /**
     * @brief Register navigation area
     * @param id Area identifier
     * @param area Area bounds
     */
    void registerArea(int64_t id, ::Rectangle area);
    
    /**
     * @brief Lock area (prevent traversal)
     * @param id Area identifier
     */
    void lockArea(int64_t id);
    
    /**
     * @brief Unlock area
     * @param id Area identifier
     */
    void unlockArea(int64_t id);
    
    /**
     * @brief Set map boundary contours
     * @param contours Map boundary contour shapes
     */
    void setMapContours(const shape::MapBoundaryContours& contours);
    
    /**
     * @brief Get map boundary contours
     * @return Pointer to map boundary contours, or nullptr if not set
     */
    [[nodiscard]] const shape::MapBoundaryContours* getMapContours() const;
    
    // ============================================================================
    // Dynamic Obstacle Layer Operations
    // ============================================================================
    
    /**
     * @brief Insert or update dynamic obstacle with TTL
     * 
     * Dynamic obstacles are time-limited and automatically expire.
     * Useful for temporary obstacles like detected objects.
     * 
     * @param id Unique obstacle identifier
     * @param geometry Obstacle shape
     * @param ttl Time-to-live duration (default: 5 seconds)
     * @return true if inserted, false if buffer full
     * 
     * @note Lock-free for single writer
     * @note Automatically evicts expired obstacles if buffer full
     */
    bool insertDynamicObstacle(int64_t id, Geometry geometry, 
                               std::chrono::milliseconds ttl = std::chrono::seconds(5));
    
    /**
     * @brief Remove dynamic obstacle by ID
     * 
     * @param id Obstacle identifier
     * @return true if removed, false if not found
     * 
     * @note Lock-free for single writer
     */
    bool removeDynamicObstacle(int64_t id);
    
    /**
     * @brief Remove all expired dynamic obstacles
     * 
     * TTL-based expiration for automatic cleanup of stale obstacles.
     * Currently returns 0 as TTL tracking is not yet active.
     * 
     * @return Number of obstacles removed
     */
    std::size_t removeExpiredDynamicObstacles();
    
    /**
     * @brief Clear all dynamic obstacles (lock-free)
     * 
     * Replaces entire Quad-tree with empty tree.
     */
    void clearDynamicObstacles();
    
    /**
     * @brief Get number of active dynamic obstacles (lock-free)
     * 
     * @return Number of obstacles in Quad-tree
     * 
     * @note Lock-free atomic read
     */
    [[nodiscard]] std::size_t getDynamicObstacleCount() const;
    
    // BatchOperations needs access to Snapshot and updateSnapshot
    friend class BatchOperations;

private:
    struct Snapshot {
        std::shared_ptr<SpatialIndex> obstacle_tree;
        std::unordered_map<int64_t, GeometryEntry> obstacles;
        
        std::shared_ptr<SpatialIndex> area_tree;
        std::unordered_map<int64_t, ::Rectangle> areas;
        
        std::unordered_set<int64_t> locked_areas;
        std::shared_ptr<shape::MapBoundaryContours> contours;
        
        // Default constructor for memory pool pre-allocation
        Snapshot() = default;
        
        Snapshot(std::shared_ptr<SpatialIndex> obs_tree,
                std::unordered_map<int64_t, GeometryEntry> obs,
                std::shared_ptr<SpatialIndex> area_tree,
                std::unordered_map<int64_t, ::Rectangle> areas_map,
                std::unordered_set<int64_t> locked,
                std::shared_ptr<shape::MapBoundaryContours> cont);
    };
    
    // Static obstacle layer (RCU-protected snapshot)
    rcu::AtomicSharedPtr<Snapshot> snapshot_;
    mutable std::mutex static_mutex_;  // Serializes snapshot updates (RCU write path)
    mutable SnapshotMemoryPool<Snapshot, 4> snapshot_pool_;  // Pre-allocated snapshot pool
    
    // Dynamic obstacle layer (fully lock-free with embedded geometry)
    std::shared_ptr<SimpleQuadTree> dynamic_quadtree_;
    
    // Helper: Create new snapshot with modifications
    template<typename Modifier,
             typename = std::enable_if_t<std::is_invocable_v<Modifier, Snapshot&>>>
    void updateSnapshot(Modifier&& modifier);
};

} // namespace geofence
} // namespace rises

// Include template implementations
#include "geofence_map_impl.hpp"
