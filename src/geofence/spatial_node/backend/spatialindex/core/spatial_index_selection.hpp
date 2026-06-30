#pragma once

#include "nanoflann_adapter.hpp"

namespace rises {
namespace geofence {

// ============================================================================
// Spatial Index Selection
// ============================================================================
// Nanoflann (header-only KD-tree) is the spatial index. It is the most
// performant choice for this workload (2D, in-memory, mostly-static map of a
// few thousand obstacles, per-scan kNN/range queries) and has zero external
// dependencies. The pluggable Boost-rtree / libspatialindex / CUDA-BVH
// adapters were removed: none were built or tested, and none beat a KD-tree at
// 2D in-memory queries (CUDA only wins at millions of primitives). The
// SpatialIndexInterface (spatial_index_interface.hpp) remains the seam through
// which an alternative backend could be reintroduced.
//
// The type alias names the concrete backend for the composition root
// (geofencing_node.cpp), which is the only place that should depend on it.
using SpatialIndex = NanoflannAdapter;
#define SPATIAL_BACKEND_NAME "nanoflann"

} // namespace geofence
} // namespace rises
