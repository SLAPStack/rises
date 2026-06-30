#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <functional>
#include <atomic>
#include "geofence/spatial/common/bounding_box.hpp"
#include "node_pool.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"

namespace rises {
namespace geofence {

/**
 * @brief Hierarchical spatial index combining R-tree (static) and Quad-tree (dynamic)
 * 
 * Optimal data structure for dual-layer geofence architecture:
 * - R-tree: Optimized for static obstacles (read-heavy, infrequent updates)
 * - Quad-tree: Optimized for dynamic obstacles (write-heavy, frequent updates)
 * 
 * Benefits:
 * - Best of both worlds: Fast static queries + fast dynamic updates
 * - Reduced rebuild overhead (only quad-tree updates, R-tree stable)
 * - Better locality: Dynamic obstacles clustered in quad-tree nodes
 * 
 * Trade-offs:
 * - Slightly higher memory (two indexes)
 * - More complex query logic (query both structures)
 * 
 * Performance:
 * - Static insert: O(log N) (R-tree rebuild, infrequent)
 * - Dynamic insert: O(log N) (Quad-tree, frequent)
 * - Query: O(log N_static + log N_dynamic + K)
 * 
 * @tparam RTreeBackend R-tree implementation for static obstacles
 * @tparam QuadTreeBackend Quad-tree implementation for dynamic obstacles
 */
template<typename RTreeBackend, typename QuadTreeBackend>
class HierarchicalSpatialIndex {
public:
    /**
     * @brief Construct hierarchical index with world bounds
     * 
     * @param world_bounds World bounding box for quad-tree partitioning
     * @param rtree_factory Factory for creating R-tree instances
     * @param quadtree_capacity Max elements per quad-tree node (default: 16)
     */
    explicit HierarchicalSpatialIndex(
        const BoundingBox& world_bounds,
        std::function<std::shared_ptr<RTreeBackend>()> rtree_factory,
        std::size_t quadtree_capacity = 16);
    
    // ========================================================================
    // Static Layer Operations (R-tree backend)
    // ========================================================================
    
    /**
     * @brief Insert static obstacle into R-tree
     * 
     * @param id Obstacle identifier
     * @param bbox Bounding box
     * 
     * @note Expensive operation - rebuilds spatial index
     * @note Use batch insertions when possible
     */
    void insertStatic(int64_t id, const BoundingBox& bbox);
    
    /**
     * @brief Remove static obstacle from R-tree
     * 
     * @param id Obstacle identifier
     * @param bbox Bounding box (needed for removal)
     */
    void removeStatic(int64_t id, const BoundingBox& bbox);
    
    /**
     * @brief Query static obstacles in region
     * 
     * @param bbox Query bounding box
     * @return Vector of obstacle IDs
     */
    [[nodiscard]] std::vector<int64_t> queryStatic(const BoundingBox& bbox) const;
    
    // ========================================================================
    // Dynamic Layer Operations (Quad-tree backend)
    // ========================================================================
    
    /**
     * @brief Insert dynamic obstacle into quad-tree
     * 
     * @param id Obstacle identifier
     * @param bbox Bounding box
     * 
     * @note Fast operation - O(log N) tree traversal
     * @note Optimized for frequent updates
     */
    void insertDynamic(int64_t id, const BoundingBox& bbox);
    
    /**
     * @brief Remove dynamic obstacle from quad-tree
     * 
     * @param id Obstacle identifier
     * @param bbox Bounding box
     */
    void removeDynamic(int64_t id, const BoundingBox& bbox);
    
    /**
     * @brief Query dynamic obstacles in region
     * 
     * @param bbox Query bounding box
     * @return Vector of obstacle IDs
     */
    [[nodiscard]] std::vector<int64_t> queryDynamic(const BoundingBox& bbox) const;
    
    // ========================================================================
    // Combined Operations (Both layers)
    // ========================================================================
    
    /**
     * @brief Query all obstacles (static + dynamic) in region
     * 
     * @param bbox Query bounding box
     * @return Vector of obstacle IDs from both layers
     * 
     * @note Queries both R-tree and quad-tree, merges results
     */
    [[nodiscard]] std::vector<int64_t> queryAll(const BoundingBox& bbox) const;
    
    /**
     * @brief Clear all obstacles from both layers
     */
    void clear();
    
    /**
     * @brief Get statistics for monitoring
     */
    struct Stats {
        std::size_t static_count;
        std::size_t dynamic_count;
        std::size_t quadtree_depth;
        std::size_t quadtree_nodes;
    };
    
    [[nodiscard]] Stats getStats() const;
    
private:
    // R-tree for static obstacles (read-optimized)
    std::shared_ptr<RTreeBackend> static_rtree_;
    
    // Quad-tree for dynamic obstacles (write-optimized)
    std::shared_ptr<QuadTreeBackend> dynamic_quadtree_;
    
    BoundingBox world_bounds_;
    std::size_t quadtree_capacity_;
};

/**
 * @brief Lock-free quad-tree using path-copying RCU for dynamic obstacles
 * 
 * Recursively partitions 2D space into quadrants until node capacity reached.
 * Uses path-copying technique for lock-free updates:
 * - Reads: Lock-free via atomic shared_ptr load
 * - Writes: Copy only affected path (root to leaf), atomic pointer swap
 * - Memory: Copies ~4-6 nodes (~100-200 bytes) instead of full structure
 * 
 * Performance:
 * - Insert/Remove: O(log N) + path copy (typically 4-6 nodes)
 * - Query: O(log N + K) lock-free traversal
 * - Memory overhead: ~200 bytes per update vs ~30KB for full RCU
 * 
 * Thread Safety:
 * - Multiple concurrent readers: Safe (lock-free)
 * - Single writer: Safe (external synchronization for multiple writers)
 * - Read-write: Safe (RCU pattern ensures consistency)
 */
class SimpleQuadTree {
public:
    explicit SimpleQuadTree(const BoundingBox& bounds, std::size_t capacity = 16);
    
    void insert(int64_t id, const BoundingBox& bbox);
    void insert(const GeometryEntry& entry);
    void remove(int64_t id, const BoundingBox& bbox);
    bool removeGeometry(int64_t id);  // Remove by ID only (searches tree)
    [[nodiscard]] std::vector<int64_t> query(const BoundingBox& bbox) const;
    
    template<typename Visitor>
    void queryGeometry(const BoundingBox& bbox, Visitor&& visitor) const;
    
    void clear();
    
    [[nodiscard]] std::size_t depth() const;
    [[nodiscard]] std::size_t nodeCount() const;
    [[nodiscard]] std::size_t size() const;
    
private:
    struct Node {
        BoundingBox bounds;
        std::vector<std::pair<int64_t, BoundingBox>> elements;
        std::vector<GeometryEntry> geometry_elements;  // Full geometry storage for lock-free queries
        std::shared_ptr<Node> children[4];  // NW, NE, SW, SE (shared for RCU)
        bool is_leaf;
        
        explicit Node(const BoundingBox& b) : bounds(b), is_leaf(true) {}
        
        // Copy constructor for path copying
        Node(const Node& other)
            : bounds(other.bounds)
            , elements(other.elements)
            , geometry_elements(other.geometry_elements)
            , is_leaf(other.is_leaf)
        {
            for (std::size_t i = 0; i < 4; ++i) {
                this->children[i] = other.children[i];  // Share unchanged children
            }
        }
    };
    
    std::shared_ptr<Node> root_;
    std::atomic<std::size_t> size_;
    std::size_t capacity_;
    
    // Path-copying insert: Returns new root with modifications
    std::shared_ptr<Node> insertRecursive(const std::shared_ptr<Node>& node, int64_t id, const BoundingBox& bbox, bool& inserted);
    std::shared_ptr<Node> insertGeometryRecursive(const std::shared_ptr<Node>& node, const GeometryEntry& entry, bool& inserted);
    std::shared_ptr<Node> subdivide(const std::shared_ptr<Node>& node);
    std::shared_ptr<Node> subdivideWithGeometry(const std::shared_ptr<Node>& node);
    
    // Path-copying remove: Returns new root with modifications
    std::shared_ptr<Node> removeRecursive(const std::shared_ptr<Node>& node, int64_t id, const BoundingBox& bbox, bool& removed);
    std::shared_ptr<Node> removeGeometryRecursive(const std::shared_ptr<Node>& node, int64_t id, bool& removed);
    
    void queryRecursive(const Node* node, const BoundingBox& bbox, std::vector<int64_t>& results) const;
    
    template<typename Visitor>
    void queryGeometryRecursive(const Node* node, const BoundingBox& bbox, Visitor&& visitor) const;
    
    std::size_t depthRecursive(const Node* node) const;
    std::size_t nodeCountRecursive(const Node* node) const;
    std::size_t sizeRecursive(const Node* node) const;
};

} // namespace geofence
} // namespace rises

#include "hierarchical_spatial_index_impl.hpp"
