#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>

namespace rises {

/**
 * @brief Axis-aligned bounding box for a segment.
 */
struct SegmentAABB {
    float min_x;
    float min_y;
    float max_x;
    float max_y;
};

/**
 * @class ErrorSegmentTracker
 * @brief Assigns persistent segment IDs to error segments across scan cycles.
 *
 * Uses a flat sorted array with two-buffer swap for cache-friendly matching.
 * Matching combines centroid proximity and AABB overlap to handle segments
 * that shift by one or more scan points between frames.
 *
 * Each tracked segment stores its origin (first-detection centroid/AABB).
 * Frame-to-frame matching uses the latest centroid/AABB for scoring, but
 * rejects matches where the current observation has drifted beyond
 * max_drift from the origin. This prevents a slowly moving obstacle from
 * keeping the same ID indefinitely.
 *
 * Eviction is implicit: only entries seen (via getOrAssignId) in the
 * current cycle survive into the next.
 */
class ErrorSegmentTracker {
public:
    /**
     * @param cell_size   Grid cell size for spatial lookup (meters).
     * @param max_drift   Maximum allowed drift from origin before a new ID
     *                    is assigned (meters). 0 = unlimited drift.
     */
    explicit ErrorSegmentTracker(const float cell_size = 0.3f,
                                 const float max_drift = 1.0f)
        : cell_size_(cell_size),
          inv_cell_size_(1.0f / cell_size),
          max_dist_sq_(cell_size * cell_size * 2.25f),
          max_drift_sq_(max_drift > 0.0f ? max_drift * max_drift : 0.0f)
    {}

    void beginScanCycle()
    {
        this->current_.clear();
        this->current_.reserve(this->previous_.size());
        this->claimed_ids_.clear();
    }

    /**
     * @brief Look up or assign a persistent segment ID.
     *
     * Matching scores previous-cycle entries by centroid proximity and AABB
     * overlap. A match is rejected if the current centroid has drifted beyond
     * max_drift from the entry's origin (first-detection position).
     *
     * @param cx   Centroid X in meters.
     * @param cy   Centroid Y in meters.
     * @param aabb Bounding box of the segment's vertices.
     * @return Persistent segment ID. The ID is stable across scans for a
     *         stationary feature; a moving obstacle drifts past max_drift and is
     *         re-assigned a new ID, so ID recurrence indicates a static feature.
     */
    int32_t getOrAssignId(const float cx, const float cy, const SegmentAABB& aabb)
    {
        const int32_t gx = this->toGrid(cx);
        const int32_t gy = this->toGrid(cy);

        const int32_t aabb_gx_min = this->toGrid(aabb.min_x);
        const int32_t aabb_gx_max = this->toGrid(aabb.max_x);
        const int32_t search_col_min = std::min(gx - 1, aabb_gx_min - 1);
        const int32_t search_col_max = std::max(gx + 1, aabb_gx_max + 1);

        const int32_t aabb_gy_min = this->toGrid(aabb.min_y);
        const int32_t aabb_gy_max = this->toGrid(aabb.max_y);
        const int32_t search_row_min = std::min(gy - 1, aabb_gy_min - 1);
        const int32_t search_row_max = std::max(gy + 1, aabb_gy_max + 1);

        float best_score = -1.0f;
        int32_t best_id = -1;
        float best_origin_cx = 0.0f;
        float best_origin_cy = 0.0f;
        SegmentAABB best_origin_aabb{};

        for (int32_t col = search_col_min; col <= search_col_max; ++col) {
            Entry probe{};
            probe.cell_x = col;
            probe.cell_y = search_row_min - 1;

            std::vector<Entry>::const_iterator lo = std::lower_bound(
                this->previous_.cbegin(), this->previous_.cend(), probe, EntryCmp{});

            for (std::vector<Entry>::const_iterator it = lo; it != this->previous_.cend(); ++it) {
                if (it->cell_x != col) break;
                if (it->cell_y > search_row_max) break;
                if (it->cell_y < search_row_min) continue;

                if (this->isIdClaimed(it->id)) continue;

                // Reject if drifted too far from origin (first detection position)
                if (this->max_drift_sq_ > 0.0f) {
                    const float odx = cx - it->origin_cx;
                    const float ody = cy - it->origin_cy;
                    if (odx * odx + ody * ody > this->max_drift_sq_) continue;
                }

                // Score by centroid proximity to last observation
                const float ddx = it->cx - cx;
                const float ddy = it->cy - cy;
                const float dist_sq = ddx * ddx + ddy * ddy;
                float score = 0.0f;
                if (dist_sq < this->max_dist_sq_) {
                    score = 1.0f - (dist_sq / this->max_dist_sq_);
                }

                // Score by AABB overlap with last observation
                const float overlap_x = std::max(0.0f,
                    std::min(aabb.max_x, it->aabb.max_x) - std::max(aabb.min_x, it->aabb.min_x));
                const float overlap_y = std::max(0.0f,
                    std::min(aabb.max_y, it->aabb.max_y) - std::max(aabb.min_y, it->aabb.min_y));
                const float overlap_area = overlap_x * overlap_y;

                if (overlap_area > 0.0f) {
                    const float area_prev =
                        (it->aabb.max_x - it->aabb.min_x) * (it->aabb.max_y - it->aabb.min_y);
                    const float area_curr =
                        (aabb.max_x - aabb.min_x) * (aabb.max_y - aabb.min_y);
                    const float min_area = std::max(std::min(area_prev, area_curr), 1e-8f);
                    const float overlap_ratio = overlap_area / min_area;
                    score = std::max(score, 0.5f + 0.5f * overlap_ratio);
                }

                if (score > best_score) {
                    best_score = score;
                    best_id = it->id;
                    best_origin_cx = it->origin_cx;
                    best_origin_cy = it->origin_cy;
                    best_origin_aabb = it->origin_aabb;
                }
            }
        }

        const bool is_new = (best_id < 0);
        const int32_t result_id = is_new ? this->next_id_++ : best_id;

        this->claimed_ids_.push_back(result_id);

        Entry entry{};
        entry.cell_x = gx;
        entry.cell_y = gy;
        entry.cx = cx;
        entry.cy = cy;
        entry.aabb = aabb;
        entry.id = result_id;
        // Preserve origin from first detection; new obstacles use current as origin
        entry.origin_cx = is_new ? cx : best_origin_cx;
        entry.origin_cy = is_new ? cy : best_origin_cy;
        entry.origin_aabb = is_new ? aabb : best_origin_aabb;
        this->current_.push_back(entry);

        return result_id;
    }

    void endScanCycle()
    {
        std::sort(this->current_.begin(), this->current_.end(), EntryCmp{});
        this->previous_.swap(this->current_);
        this->current_.clear();
        this->claimed_ids_.clear();
    }

    /** @brief Number of tracked segments from the previous cycle. */
    [[nodiscard]] std::size_t size() const { return this->previous_.size(); }

    /** @brief Remove all tracked segments and reset the ID counter. */
    void clear()
    {
        this->previous_.clear();
        this->current_.clear();
        this->claimed_ids_.clear();
        this->next_id_ = 1;
    }

private:
    struct Entry {
        int32_t cell_x;
        int32_t cell_y;
        float cx;              // latest centroid (for frame-to-frame matching)
        float cy;
        SegmentAABB aabb;      // latest bounding box (for overlap matching)
        float origin_cx;       // first-detection centroid (drift anchor)
        float origin_cy;
        SegmentAABB origin_aabb; // first-detection bounding box
        int32_t id;
    };

    struct EntryCmp {
        bool operator()(const Entry& a, const Entry& b) const
        {
            if (a.cell_x != b.cell_x) return a.cell_x < b.cell_x;
            return a.cell_y < b.cell_y;
        }
    };

    int32_t toGrid(const float v) const
    {
        return static_cast<int32_t>(std::floor(v * this->inv_cell_size_));
    }

    bool isIdClaimed(const int32_t id) const
    {
        for (const int32_t claimed : this->claimed_ids_) {
            if (claimed == id) return true;
        }
        return false;
    }

    float cell_size_;
    float inv_cell_size_;
    float max_dist_sq_;   // (cell_size * 1.5)^2 — frame-to-frame centroid threshold
    float max_drift_sq_;  // max allowed drift from origin (0 = unlimited)
    int32_t next_id_ = 1;

    std::vector<Entry> previous_;
    std::vector<Entry> current_;
    std::vector<int32_t> claimed_ids_;
};

} // namespace rises
