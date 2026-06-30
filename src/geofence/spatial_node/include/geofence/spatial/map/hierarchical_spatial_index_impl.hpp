#pragma once

#include "hierarchical_spatial_index.hpp"
#include <algorithm>

namespace rises {
namespace geofence {

// ============================================================================
// HierarchicalSpatialIndex Implementation
// ============================================================================

template<typename RTreeBackend, typename QuadTreeBackend>
HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::HierarchicalSpatialIndex(
    const BoundingBox& world_bounds,
    std::function<std::shared_ptr<RTreeBackend>()> rtree_factory,
    std::size_t quadtree_capacity)
    : world_bounds_(world_bounds)
    , quadtree_capacity_(quadtree_capacity)
{
    static_rtree_ = rtree_factory();
    dynamic_quadtree_ = std::make_shared<QuadTreeBackend>(world_bounds, quadtree_capacity);
}

template<typename RTreeBackend, typename QuadTreeBackend>
void HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::insertStatic(
    int64_t id, 
    const BoundingBox& bbox)
{
    static_rtree_->insert(id, bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
void HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::removeStatic(
    int64_t id, 
    const BoundingBox& bbox)
{
    static_rtree_->remove(id, bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
std::vector<int64_t> HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::queryStatic(
    const BoundingBox& bbox) const
{
    return static_rtree_->query(bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
void HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::insertDynamic(
    int64_t id, 
    const BoundingBox& bbox)
{
    dynamic_quadtree_->insert(id, bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
void HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::removeDynamic(
    int64_t id, 
    const BoundingBox& bbox)
{
    dynamic_quadtree_->remove(id, bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
std::vector<int64_t> HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::queryDynamic(
    const BoundingBox& bbox) const
{
    return dynamic_quadtree_->query(bbox);
}

template<typename RTreeBackend, typename QuadTreeBackend>
std::vector<int64_t> HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::queryAll(
    const BoundingBox& bbox) const
{
    // Query both layers
    std::vector<int64_t> static_results = queryStatic(bbox);
    std::vector<int64_t> dynamic_results = queryDynamic(bbox);
    
    // Merge results
    std::vector<int64_t> combined;
    combined.reserve(static_results.size() + dynamic_results.size());
    combined.insert(combined.end(), static_results.begin(), static_results.end());
    combined.insert(combined.end(), dynamic_results.begin(), dynamic_results.end());
    
    return combined;
}

template<typename RTreeBackend, typename QuadTreeBackend>
void HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::clear()
{
    static_rtree_->clear();
    dynamic_quadtree_->clear();
}

template<typename RTreeBackend, typename QuadTreeBackend>
typename HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::Stats 
HierarchicalSpatialIndex<RTreeBackend, QuadTreeBackend>::getStats() const
{
    return Stats{
        static_rtree_->size(),
        dynamic_quadtree_->size(),
        dynamic_quadtree_->depth(),
        dynamic_quadtree_->nodeCount()
    };
}

// ============================================================================
// SimpleQuadTree Implementation
// ============================================================================

inline SimpleQuadTree::SimpleQuadTree(const BoundingBox& bounds, std::size_t capacity)
    : root_(makeSharedNode<Node>(bounds))
    , size_(0)
    , capacity_(capacity)
{
}

inline void SimpleQuadTree::insert(int64_t id, const BoundingBox& bbox)
{
    bool inserted = false;
    this->root_ = this->insertRecursive(this->root_, id, bbox, inserted);
    if (inserted) {
        this->size_.fetch_add(1, std::memory_order_relaxed);
    }
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::insertRecursive(
    const std::shared_ptr<Node>& node, 
    int64_t id, 
    const BoundingBox& bbox,
    bool& inserted)
{
    // Check if bbox intersects node bounds
    if (!node->bounds.intersects(bbox)) {
        return node;  // No change needed
    }
    
    // Create a copy of this node for path copying
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    // If leaf and not full, add element
    if (new_node->is_leaf) {
        // Check for duplicate ID (update instead of insert)
        auto it = std::find_if(
            new_node->elements.begin(),
            new_node->elements.end(),
            [id](const auto& pair) { return pair.first == id; }
        );
        
        if (it != new_node->elements.end()) {
            // Update existing element
            it->second = bbox;
            return new_node;
        }
        
        // Insert new element
        new_node->elements.emplace_back(id, bbox);
        inserted = true;
        
        // Subdivide if capacity exceeded
        if (new_node->elements.size() > this->capacity_) {
            new_node = this->subdivide(new_node);
        }
        return new_node;
    }
    
    // Not a leaf, insert into children (path copying)
    for (std::size_t i = 0; i < 4; ++i) {
        if (new_node->children[i] && new_node->children[i]->bounds.intersects(bbox)) {
            // Recursively insert into child (creates new path)
            new_node->children[i] = this->insertRecursive(new_node->children[i], id, bbox, inserted);
        }
    }
    
    return new_node;
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::subdivide(const std::shared_ptr<Node>& node)
{
    if (!node->is_leaf) {
        return node;
    }
    
    // Create new node (copy)
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    const auto& bounds = new_node->bounds;
    const float mid_x = (bounds.min_x + bounds.max_x) * 0.5f;
    const float mid_y = (bounds.min_y + bounds.max_y) * 0.5f;
    
    // Create quadrants: NW, NE, SW, SE
    new_node->children[0] = makeSharedNode<Node>(BoundingBox{
        bounds.min_x, mid_x, mid_y, bounds.max_y});  // NW
    new_node->children[1] = makeSharedNode<Node>(BoundingBox{
        mid_x, bounds.max_x, mid_y, bounds.max_y});  // NE
    new_node->children[2] = makeSharedNode<Node>(BoundingBox{
        bounds.min_x, mid_x, bounds.min_y, mid_y});  // SW
    new_node->children[3] = makeSharedNode<Node>(BoundingBox{
        mid_x, bounds.max_x, bounds.min_y, mid_y});  // SE
    
    // Redistribute elements to children
    for (const auto& [elem_id, elem_bbox] : new_node->elements) {
        for (auto& child : new_node->children) {
            if (child->bounds.intersects(elem_bbox)) {
                child->elements.emplace_back(elem_id, elem_bbox);
            }
        }
    }
    
    // Clear parent elements and mark as non-leaf
    new_node->elements.clear();
    new_node->is_leaf = false;
    
    return new_node;
}

inline void SimpleQuadTree::remove(int64_t id, const BoundingBox& bbox)
{
    bool removed = false;
    this->root_ = this->removeRecursive(this->root_, id, bbox, removed);
    if (removed) {
        this->size_.fetch_sub(1, std::memory_order_relaxed);
    }
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::removeRecursive(
    const std::shared_ptr<Node>& node,
    int64_t id,
    const BoundingBox& bbox,
    bool& removed)
{
    if (!node || !node->bounds.intersects(bbox)) {
        return node;
    }
    
    // Create a copy of this node for path copying
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    if (new_node->is_leaf) {
        auto it = std::remove_if(
            new_node->elements.begin(), 
            new_node->elements.end(),
            [id](const auto& pair) { return pair.first == id; }
        );
        
        if (it != new_node->elements.end()) {
            new_node->elements.erase(it, new_node->elements.end());
            removed = true;
        }
        return new_node;
    }
    
    // Try removing from children (path copying)
    for (std::size_t i = 0; i < 4; ++i) {
        if (new_node->children[i]) {
            new_node->children[i] = this->removeRecursive(new_node->children[i], id, bbox, removed);
        }
    }
    
    return new_node;
}

inline bool SimpleQuadTree::removeGeometry(int64_t id)
{
    bool removed = false;
    this->root_ = this->removeGeometryRecursive(this->root_, id, removed);
    if (removed) {
        this->size_.fetch_sub(1, std::memory_order_relaxed);
    }
    return removed;
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::removeGeometryRecursive(
    const std::shared_ptr<Node>& node,
    int64_t id,
    bool& removed)
{
    if (!node) {
        return node;
    }
    
    // Create a copy of this node for path copying
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    if (new_node->is_leaf) {
        auto it = std::remove_if(
            new_node->geometry_elements.begin(), 
            new_node->geometry_elements.end(),
            [id](const GeometryEntry& entry) { return entry.id == id; }
        );
        
        if (it != new_node->geometry_elements.end()) {
            new_node->geometry_elements.erase(it, new_node->geometry_elements.end());
            removed = true;
        }
        return new_node;
    }
    
    // Try removing from all children (path copying)
    for (std::size_t i = 0; i < 4; ++i) {
        if (new_node->children[i]) {
            new_node->children[i] = this->removeGeometryRecursive(new_node->children[i], id, removed);
            if (removed) {
                break;  // Found and removed, stop searching
            }
        }
    }
    
    return new_node;
}

inline std::vector<int64_t> SimpleQuadTree::query(const BoundingBox& bbox) const
{
    std::vector<int64_t> results;
    queryRecursive(root_.get(), bbox, results);
    return results;
}

inline void SimpleQuadTree::queryRecursive(
    const Node* node, 
    const BoundingBox& bbox, 
    std::vector<int64_t>& results) const
{
    if (!node || !node->bounds.intersects(bbox)) {
        return;
    }
    
    // If leaf, check elements
    if (node->is_leaf) {
        for (const auto& [id, elem_bbox] : node->elements) {
            if (elem_bbox.intersects(bbox)) {
                results.push_back(id);
            }
        }
        return;
    }
    
    // Recurse to children
    for (const auto& child : node->children) {
        if (child) {
            queryRecursive(child.get(), bbox, results);
        }
    }
}

inline void SimpleQuadTree::clear()
{
    this->root_ = makeSharedNode<Node>(this->root_->bounds);
    this->size_.store(0, std::memory_order_relaxed);
}

inline std::size_t SimpleQuadTree::depth() const
{
    return this->depthRecursive(this->root_.get());
}

inline std::size_t SimpleQuadTree::depthRecursive(const Node* node) const
{
    if (!node || node->is_leaf) {
        return 1;
    }
    
    std::size_t max_depth = 0;
    for (const auto& child : node->children) {
        if (child) {
            max_depth = std::max(max_depth, this->depthRecursive(child.get()));
        }
    }
    return max_depth + 1;
}

inline std::size_t SimpleQuadTree::nodeCount() const
{
    return this->nodeCountRecursive(this->root_.get());
}

inline std::size_t SimpleQuadTree::size() const
{
    return this->size_.load(std::memory_order_relaxed);
}

inline std::size_t SimpleQuadTree::nodeCountRecursive(const Node* node) const
{
    if (!node) {
        return 0;
    }
    
    std::size_t count = 1;  // Count this node
    for (const auto& child : node->children) {
        if (child) {
            count += this->nodeCountRecursive(child.get());
        }
    }
    return count;
}

inline std::size_t SimpleQuadTree::sizeRecursive(const Node* node) const
{
    if (!node) {
        return 0;
    }
    
    if (node->is_leaf) {
        return node->elements.size();
    }
    
    std::size_t total = 0;
    for (const auto& child : node->children) {
        if (child) {
            total += this->sizeRecursive(child.get());
        }
    }
    return total;
}

// ============================================================================
// Geometry-Aware Operations (Fully Lock-Free)
// ============================================================================

inline void SimpleQuadTree::insert(const GeometryEntry& entry)
{
    bool inserted = false;
    this->root_ = this->insertGeometryRecursive(this->root_, entry, inserted);
    if (inserted) {
        this->size_.fetch_add(1, std::memory_order_relaxed);
    }
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::insertGeometryRecursive(
    const std::shared_ptr<Node>& node, const GeometryEntry& entry, bool& inserted)
{
    if (!node) {
        return node;
    }
    
    const BoundingBox bbox{
        static_cast<float>(entry.bbox.min.x),
        static_cast<float>(entry.bbox.min.y),
        static_cast<float>(entry.bbox.max.x),
        static_cast<float>(entry.bbox.max.y)
    };
    
    if (!node->bounds.intersects(bbox)) {
        return node;  // No change needed
    }
    
    // Create a copy of this node for path copying
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    // If leaf and not full, add element
    if (new_node->is_leaf) {
        // Check for duplicate ID (update instead of insert)
        auto it = std::find_if(
            new_node->geometry_elements.begin(),
            new_node->geometry_elements.end(),
            [id = entry.id](const GeometryEntry& e) { return e.id == id; }
        );
        
        if (it != new_node->geometry_elements.end()) {
            // Update existing element
            *it = entry;
            return new_node;
        }
        
        // Insert new element
        new_node->geometry_elements.push_back(entry);
        inserted = true;
        
        // Subdivide if capacity exceeded
        if (new_node->geometry_elements.size() > this->capacity_) {
            new_node = this->subdivideWithGeometry(new_node);
        }
        return new_node;
    }
    
    // Not a leaf, insert into children (path copying)
    for (std::size_t i = 0; i < 4; ++i) {
        if (new_node->children[i] && new_node->children[i]->bounds.intersects(bbox)) {
            new_node->children[i] = this->insertGeometryRecursive(new_node->children[i], entry, inserted);
        }
    }
    
    return new_node;
}

inline std::shared_ptr<SimpleQuadTree::Node> SimpleQuadTree::subdivideWithGeometry(const std::shared_ptr<Node>& node)
{
    if (!node->is_leaf) {
        return node;
    }
    
    // Create new node (copy)
    std::shared_ptr<Node> new_node = makeSharedNode<Node>(*node);
    
    const auto& bounds = new_node->bounds;
    const float mid_x = (bounds.min_x + bounds.max_x) * 0.5f;
    const float mid_y = (bounds.min_y + bounds.max_y) * 0.5f;
    
    // Create quadrants: NW, NE, SW, SE
    new_node->children[0] = makeSharedNode<Node>(BoundingBox{
        bounds.min_x, mid_x, mid_y, bounds.max_y});  // NW
    new_node->children[1] = makeSharedNode<Node>(BoundingBox{
        mid_x, bounds.max_x, mid_y, bounds.max_y});  // NE
    new_node->children[2] = makeSharedNode<Node>(BoundingBox{
        bounds.min_x, mid_x, bounds.min_y, mid_y});  // SW
    new_node->children[3] = makeSharedNode<Node>(BoundingBox{
        mid_x, bounds.max_x, bounds.min_y, mid_y});  // SE
    
    // Redistribute geometry elements to children
    for (const auto& entry : new_node->geometry_elements) {
        const BoundingBox bbox{
            static_cast<float>(entry.bbox.min.x),
            static_cast<float>(entry.bbox.min.y),
            static_cast<float>(entry.bbox.max.x),
            static_cast<float>(entry.bbox.max.y)
        };
        
        for (auto& child : new_node->children) {
            if (child->bounds.intersects(bbox)) {
                child->geometry_elements.push_back(entry);
            }
        }
    }
    
    // Clear parent elements and mark as non-leaf
    new_node->geometry_elements.clear();
    new_node->is_leaf = false;
    
    return new_node;
}

template<typename Visitor>
inline void SimpleQuadTree::queryGeometry(const BoundingBox& bbox, Visitor&& visitor) const
{
    this->queryGeometryRecursive(this->root_.get(), bbox, std::forward<Visitor>(visitor));
}

template<typename Visitor>
inline void SimpleQuadTree::queryGeometryRecursive(const Node* node, const BoundingBox& bbox, Visitor&& visitor) const
{
    if (!node || !node->bounds.intersects(bbox)) {
        return;
    }
    
    // If leaf, iterate geometry elements
    if (node->is_leaf) {
        for (const GeometryEntry& entry : node->geometry_elements) {
            const BoundingBox elem_bbox{
                static_cast<float>(entry.bbox.min.x),
                static_cast<float>(entry.bbox.min.y),
                static_cast<float>(entry.bbox.max.x),
                static_cast<float>(entry.bbox.max.y)
            };
            
            if (elem_bbox.intersects(bbox)) {
                visitor(entry);
            }
        }
        return;
    }
    
    // Recurse to children
    for (const auto& child : node->children) {
        if (child) {
            this->queryGeometryRecursive(child.get(), bbox, std::forward<Visitor>(visitor));
        }
    }
}

} // namespace geofence
} // namespace rises
