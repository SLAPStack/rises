#pragma once

#include "geofence/spatial/common/bounding_box.hpp"
#include "geofence/common/geometry/variant_geometry.hpp"
#include <vector>
#include <cstdint>
#include <memory>

namespace rises {
namespace geofence {

// ============================================================================
// Common Result Types
// ============================================================================

struct SpatialQueryResult {
    int64_t id;
    float distance;
    
    bool operator<(const SpatialQueryResult& other) const {
        return distance < other.distance;
    }
};

// ============================================================================
// Unified Interface (Adapter Pattern)
// ============================================================================
// This is the "common interface" that all backends must implement
// Each backend adapter translates this to its native API

class SpatialIndexInterface {
public:
    virtual ~SpatialIndexInterface() = default;
    
    // Core operations - all backends must support these
    // Note: Uses common BoundingBox type, not Rectangle (geometry shape)
    virtual void insert(int64_t id, const BoundingBox& bbox) = 0;
    virtual void remove(int64_t id, const BoundingBox& bbox) = 0;
    virtual std::vector<int64_t> query(const BoundingBox& bbox) const = 0;
    
    // Advanced operations - may have efficient or fallback implementations
    virtual std::vector<SpatialQueryResult> knn(const Point2D& pt, size_t k) const = 0;
    virtual std::vector<SpatialQueryResult> withinRadius(const Point2D& center, float radius) const = 0;
    
    // Management
    virtual void clear() = 0;
    virtual size_t size() const = 0;
    
    // Polymorphic clone for snapshotting
    virtual std::shared_ptr<SpatialIndexInterface> clone() const = 0;

    /**
     * @brief Eagerly build any deferred internal index (e.g. kd-tree).
     *
     * Must be called by the WRITER thread in updateSnapshot() after all
     * modifications and before snapshot_.store() so that concurrent reader
     * threads never race on lazy index construction.
     *
     * Default implementation is a no-op for backends that build eagerly
     * (i.e. ones that update their index on every insert/remove rather than
     * deferring to a batch build).
     */
    virtual void ensureBuilt() {}
    
    // Generic insert for any geometry type
    // Extracts bounding box and delegates to adapter
    template<typename Shape>
    void insertShape(int64_t id, const Shape& shape) {
        BoundingBox bbox = getBoundingBox(Geometry{shape});
        insert(id, bbox);  // Adapter converts BoundingBox to backend-specific type
    }
    
    template<typename Shape>
    void removeShape(int64_t id, const Shape& shape) {
        BoundingBox bbox = getBoundingBox(Geometry{shape});
        remove(id, bbox);
    }
};

} // namespace geofence
} // namespace rises
