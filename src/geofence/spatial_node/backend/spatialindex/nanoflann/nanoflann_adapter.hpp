#pragma once

#include "../core/spatial_index_interface.hpp"
#include <algorithm>
#include <cmath>
#include <nanoflann.hpp>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

// nanoflann >= 0x150 renamed SearchParams -> SearchParameters and added a
// dedicated ResultItem<Index, Distance> struct. Older releases (Ubuntu Jammy
// ships 0x142) expose the radius-search results as a std::vector of
// std::pair<index, distance> with the older SearchParams name. Provide a
// version-gated compatibility shim so the same call sites compile against
// both APIs without an #ifdef at every use.
#if defined(NANOFLANN_VERSION) && (NANOFLANN_VERSION >= 0x150)
namespace rises {
namespace geofence {
namespace nanoflann_compat {
using SearchParameters = ::nanoflann::SearchParameters;
template <typename Index, typename Distance>
using ResultItem = ::nanoflann::ResultItem<Index, Distance>;
template <typename T> inline auto resultIndex(const T &m) -> decltype(m.first) {
  return m.first;
}
template <typename T>
inline auto resultDistance(const T &m) -> decltype(m.second) {
  return m.second;
}
} // namespace nanoflann_compat
} // namespace geofence
} // namespace rises
#else
namespace rises {
namespace geofence {
namespace nanoflann_compat {
using SearchParameters = ::nanoflann::SearchParams;
template <typename Index, typename Distance>
using ResultItem = std::pair<Index, Distance>;
template <typename Index, typename Distance>
inline Index resultIndex(const std::pair<Index, Distance> &m) {
  return m.first;
}
template <typename Index, typename Distance>
inline Distance resultDistance(const std::pair<Index, Distance> &m) {
  return m.second;
}
} // namespace nanoflann_compat
} // namespace geofence
} // namespace rises
#endif

namespace rises {
namespace geofence {

// ============================================================================
// Nanoflann Adapter - Translates nanoflann's API to our interface
// ============================================================================

class NanoflannAdapter : public SpatialIndexInterface {
public:
  struct PointCloud {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<int64_t> ids;
    std::vector<float> half_diag;

    inline size_t kdtree_get_point_count() const { return x.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
      return dim == 0 ? x[idx] : y[idx];
    }

    template <class BBOX> bool kdtree_get_bbox(BBOX &) const { return false; }
  };

  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 2>;

  NanoflannAdapter() : kdtree_(nullptr), needs_rebuild_(false) {}

  // Adapt our interface to nanoflann
  void insert(int64_t id, const BoundingBox &bbox) override {
    const float cx = (bbox.min_x + bbox.max_x) * 0.5f;
    const float cy = (bbox.min_y + bbox.max_y) * 0.5f;
    // Track the obstacle's own half-diagonal so query() can inflate its search
    // radius by the largest stored extent. Indexing by centroid alone would
    // otherwise miss a stored obstacle whose footprint overlaps the query box
    // but whose centroid lies outside the (centroid-only) search circle.
    const float dx = bbox.max_x - bbox.min_x;
    const float dy = bbox.max_y - bbox.min_y;
    const float half_diag = std::sqrt(dx * dx + dy * dy) * 0.5f;
    this->id_to_index_[id] = points_.ids.size();
    points_.x.push_back(cx);
    points_.y.push_back(cy);
    points_.ids.push_back(id);
    points_.half_diag.push_back(half_diag);
    max_half_diag_ = std::max(max_half_diag_, half_diag);
    needs_rebuild_ = true;
  }

  // O(1) remove: lookup index, swap with last element, pop_back
  void remove(int64_t id, const BoundingBox &) override {
    auto it = this->id_to_index_.find(id);
    if (it == this->id_to_index_.end())
      return;

    const size_t idx = it->second;
    const size_t last = points_.ids.size() - 1;

    // If we are removing the (a) largest obstacle, the cached max is now stale
    // and must be recomputed after the element is gone.
    const bool removing_max = (points_.half_diag[idx] >= max_half_diag_);

    if (idx != last) {
      // Move last element into the vacated slot
      points_.x[idx] = points_.x[last];
      points_.y[idx] = points_.y[last];
      points_.ids[idx] = points_.ids[last];
      points_.half_diag[idx] = points_.half_diag[last];
      // Update displaced element's index
      this->id_to_index_[points_.ids[idx]] = idx;
    }

    points_.x.pop_back();
    points_.y.pop_back();
    points_.ids.pop_back();
    points_.half_diag.pop_back();
    this->id_to_index_.erase(it);

    // O(1) amortized: only the rare removal of the single largest obstacle
    // forces an O(n) recompute of the cached maximum.
    if (removing_max) {
      max_half_diag_ = points_.half_diag.empty()
                           ? 0.0f
                           : *std::max_element(points_.half_diag.begin(),
                                               points_.half_diag.end());
    }
    needs_rebuild_ = true;
  }

  std::vector<int64_t> query(const BoundingBox &bbox) const override {
    ensureBuilt();

    std::vector<int64_t> results;
    if (!kdtree_) {
      // Empty map (no pallets loaded yet) — return empty, not an error.
      return results;
    }

    float cx = (bbox.min_x + bbox.max_x) * 0.5f;
    float cy = (bbox.min_y + bbox.max_y) * 0.5f;
    // Inflate by the largest stored obstacle's half-diagonal. For any stored
    // obstacle whose bbox truly overlaps the query bbox, the centroid distance
    // is bounded by (query half-diagonal + obstacle half-diagonal) <=
    // (query half-diagonal + max_half_diag_), so it can never be missed.
    float radius =
        std::sqrt((bbox.max_x - bbox.min_x) * (bbox.max_x - bbox.min_x) +
                  (bbox.max_y - bbox.min_y) * (bbox.max_y - bbox.min_y)) *
            0.5f +
        max_half_diag_;

    // Translate to nanoflann's API
    const float query_pt[2] = {static_cast<float>(cx), static_cast<float>(cy)};
    std::vector<nanoflann_compat::ResultItem<unsigned int, float>> ret_matches;
    nanoflann_compat::SearchParameters params;
    kdtree_->radiusSearch(query_pt, radius * radius, ret_matches, params);

    results.reserve(ret_matches.size());
    for (const auto &match : ret_matches) {
      results.push_back(points_.ids[nanoflann_compat::resultIndex(match)]);
    }

    return results;
  }

  std::vector<SpatialQueryResult> knn(const Point2D &pt,
                                      size_t k) const override {
    ensureBuilt();

    std::vector<SpatialQueryResult> results;
    if (!kdtree_)
      return results;

    // Translate to nanoflann's API
    const float query_pt[2] = {static_cast<float>(pt.x),
                               static_cast<float>(pt.y)};
    std::vector<unsigned int> ret_indices(k);
    std::vector<float> ret_distances_sq(k);

    size_t num_results = kdtree_->knnSearch(query_pt, k, ret_indices.data(),
                                            ret_distances_sq.data());

    results.reserve(num_results);
    for (size_t i = 0; i < num_results; ++i) {
      results.push_back(
          {points_.ids[ret_indices[i]], std::sqrt(ret_distances_sq[i])});
    }

    return results;
  }

  std::vector<SpatialQueryResult> withinRadius(const Point2D &center,
                                               float radius) const override {
    ensureBuilt();

    std::vector<SpatialQueryResult> results;
    if (!kdtree_)
      return results;

    // Translate to nanoflann's API
    const float query_pt[2] = {static_cast<float>(center.x),
                               static_cast<float>(center.y)};
    std::vector<nanoflann_compat::ResultItem<unsigned int, float>> ret_matches;
    nanoflann_compat::SearchParameters params;
    kdtree_->radiusSearch(query_pt, radius * radius, ret_matches, params);

    results.reserve(ret_matches.size());
    for (const auto &match : ret_matches) {
      results.push_back({points_.ids[nanoflann_compat::resultIndex(match)],
                         std::sqrt(nanoflann_compat::resultDistance(match))});
    }

    return results;
  }

  void clear() override {
    points_.x.clear();
    points_.y.clear();
    points_.ids.clear();
    points_.half_diag.clear();
    max_half_diag_ = 0.0f;
    this->id_to_index_.clear();
    kdtree_.reset();
    needs_rebuild_ = false;
  }

  size_t size() const override { return points_.ids.size(); }

  // Polymorphic clone
  std::shared_ptr<SpatialIndexInterface> clone() const override {
    std::shared_ptr<NanoflannAdapter> cloned =
        std::make_shared<NanoflannAdapter>();
    cloned->points_ = points_;
    cloned->id_to_index_ = this->id_to_index_;
    // max_half_diag_ is a member of NanoflannAdapter, NOT inside PointCloud, so
    // it is not carried by the points_ copy above and must be copied explicitly.
    cloned->max_half_diag_ = this->max_half_diag_;
    cloned->needs_rebuild_ =
        true; // Rebuild eagerly by caller via ensureBuilt()
    return cloned;
  }

  /**
   * @brief Build (or rebuild) the kdtree from the current point set.
   *
   * Must be called by the WRITER thread after all modifications and before
   * the snapshot is published via RCU store().  This guarantees that
   * concurrent reader threads never race on the mutable kdtree_ member.
   *
   * Do NOT call from query/knn/withinRadius – those paths are read-only and
   * must find the tree already built.
   */
  void ensureBuilt() override {
    if (!needs_rebuild_)
      return;
    if (!points_.ids.empty()) {
      kdtree_ = std::make_unique<KDTree>(
          2, points_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }
    // Always clear the flag so the const read-side path never fires.
    needs_rebuild_ = false;
  }

private:
  mutable PointCloud points_;
  mutable std::unique_ptr<KDTree> kdtree_;
  mutable bool needs_rebuild_;
  // Largest half-diagonal of any currently-stored obstacle's bbox; used to
  // inflate query() radii so footprint-overlapping obstacles are never missed.
  float max_half_diag_ = 0.0f;
  std::unordered_map<int64_t, size_t> id_to_index_;

  // Read-side fallback: logs a warning instead of racing on mutable state.
  // Under normal operation this should never trigger because the writer
  // calls the public ensureBuilt() before publishing the snapshot.
  void ensureBuilt() const {
    if (needs_rebuild_) {
      RCLCPP_WARN(
          rclcpp::get_logger("nanoflann_adapter"),
          "[NanoflannAdapter] ensureBuilt() called on const path – "
          "writer forgot to call ensureBuilt() before snapshot_.store(). "
          "points=%zu",
          points_.ids.size());
    }
  }
};

} // namespace geofence
} // namespace rises
