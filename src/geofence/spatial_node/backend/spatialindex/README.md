# Spatial Index

Fast proximity queries for the geofence map.

## Backend: Nanoflann (KD-tree)

[`nanoflann/`](nanoflann/) — a header-only KD-tree with excellent cache
performance. It is the sole spatial index:

- **Best for**: 2D point/range/nearest-neighbour queries over an in-memory,
  mostly-static map of a few thousand obstacles — exactly this system's
  workload.
- **Dependencies**: none (header-only).
- **Query time**: O(log n + k), cache-friendly.

Earlier versions carried pluggable Boost-rtree, libspatialindex, and CUDA-BVH
adapters behind a `SPATIAL_BACKEND` CMake option. They were removed: none were
built or tested in deployment, and none beat a KD-tree for 2D in-memory queries
(an R-tree's strength is large/disk-backed/box-heavy workloads; CUDA only pays
off at millions of primitives, where host↔device transfer would otherwise
dominate). See git history to recover them.

## Architecture

`core/spatial_index_interface.hpp` defines `SpatialIndexInterface`, the contract
every backend implements (NanoflannAdapter does). `core/spatial_index_selection.hpp`
aliases `SpatialIndex = NanoflannAdapter` for the composition root. The interface
is the seam through which an alternative index could be reintroduced (e.g. as
part of a future type-erasure of the map) without touching query call sites.
Plain geometry value types (`Point2D`, `Rectangle`) live in
`backend/geometry/core/geometry_types.hpp` and carry no index dependency.

```
GeofenceMap
    └─ SpatialIndex (= NanoflannAdapter)  →  nanoflann KD-tree
```
