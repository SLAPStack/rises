#pragma once

#include <vector>
#include <cstdint>
#include "geofence/common/geometry/variant_geometry.hpp"

namespace rises {
namespace geofence {

/**
 * @brief Lock-free batch update accumulator for dynamic obstacles
 * 
 * Collects multiple insertions/removals and applies them atomically via a single
 * RCU snapshot. This eliminates per-operation allocation overhead and ensures
 * atomic visibility (readers see all updates together, never partial state).
 * 
 * Benefits:
 * - Reduces RCU snapshot allocations from N to 1 for N updates
 * - Single atomic pointer swap instead of N swaps (better cache locality)
 * - Atomic consistency: All updates visible together to concurrent readers
 * - Optimal for map updates from perception/multi-robot systems
 * 
 * Usage:
 * @code
 *   DynamicObstacleBatch batch;
 *   // Batch updates from perception system
 *   for (const auto& detected_obstacle : perception_output) {
 *       batch.insert(detected_obstacle.id, detected_obstacle.geometry, detected_obstacle.ttl);
 *   }
 *   geofence_map.applyBatch(std::move(batch));  // Single atomic operation
 * @endcode
 * 
 * Memory Efficiency:
 * - Pre-reserve capacity via constructor: batch.reserve(100)
 * - Move semantics: applyBatch(std::move(batch)) avoids copies
 * - Reuse batches: batch.clear() + refill for subsequent updates
 */
class DynamicObstacleBatch {
public:
    struct Insertion {
        int64_t id;
        GeometryEntry geometry;
        std::chrono::nanoseconds ttl;
    };
    
    struct Removal {
        int64_t id;
    };
    
    /**
     * @brief Construct empty batch
     * 
     * @param reserve_insertions Pre-allocate space for insertions (optional)
     * @param reserve_removals Pre-allocate space for removals (optional)
     */
    explicit DynamicObstacleBatch(
        std::size_t reserve_insertions = 0,
        std::size_t reserve_removals = 0)
    {
        if (reserve_insertions > 0) {
            insertions_.reserve(reserve_insertions);
        }
        if (reserve_removals > 0) {
            removals_.reserve(reserve_removals);
        }
    }
    
    /**
     * @brief Add obstacle insertion to batch
     * 
     * @param id Obstacle identifier
     * @param geometry Obstacle geometry
     * @param ttl Time-to-live duration
     */
    void insert(int64_t id, const GeometryEntry& geometry, std::chrono::nanoseconds ttl) {
        insertions_.push_back({id, geometry, ttl});
    }
    
    /**
     * @brief Add obstacle insertion to batch (move semantics)
     */
    void insert(int64_t id, GeometryEntry&& geometry, std::chrono::nanoseconds ttl) {
        insertions_.push_back({id, std::move(geometry), ttl});
    }
    
    /**
     * @brief Add obstacle removal to batch
     * 
     * @param id Obstacle identifier to remove
     */
    void remove(int64_t id) {
        removals_.push_back({id});
    }
    
    /**
     * @brief Get insertions (const accessor)
     */
    [[nodiscard]] const std::vector<Insertion>& insertions() const noexcept {
        return insertions_;
    }
    
    /**
     * @brief Get removals (const accessor)
     */
    [[nodiscard]] const std::vector<Removal>& removals() const noexcept {
        return removals_;
    }
    
    /**
     * @brief Get insertions (move accessor for zero-copy application)
     */
    [[nodiscard]] std::vector<Insertion>&& moveInsertions() noexcept {
        return std::move(insertions_);
    }
    
    /**
     * @brief Get removals (move accessor for zero-copy application)
     */
    [[nodiscard]] std::vector<Removal>&& moveRemovals() noexcept {
        return std::move(removals_);
    }
    
    /**
     * @brief Check if batch is empty
     */
    [[nodiscard]] bool empty() const noexcept {
        return insertions_.empty() && removals_.empty();
    }
    
    /**
     * @brief Get total update count
     */
    [[nodiscard]] std::size_t size() const noexcept {
        return insertions_.size() + removals_.size();
    }
    
    /**
     * @brief Clear batch for reuse
     * 
     * @note Preserves allocated capacity for efficiency
     */
    void clear() noexcept {
        insertions_.clear();
        removals_.clear();
    }
    
    /**
     * @brief Reserve capacity for future insertions
     */
    void reserve(std::size_t insertions, std::size_t removals = 0) {
        insertions_.reserve(insertions);
        if (removals > 0) {
            removals_.reserve(removals);
        }
    }
    
private:
    std::vector<Insertion> insertions_;
    std::vector<Removal> removals_;
};

/**
 * @brief Statistics for batch update monitoring
 */
struct BatchUpdateStats {
    std::size_t total_batches{0};
    std::size_t total_insertions{0};
    std::size_t total_removals{0};
    double avg_batch_size{0.0};
    std::size_t max_batch_size{0};
    
    void recordBatch(const DynamicObstacleBatch& batch) {
        ++total_batches;
        total_insertions += batch.insertions().size();
        total_removals += batch.removals().size();
        
        std::size_t batch_size = batch.size();
        max_batch_size = std::max(max_batch_size, batch_size);
        
        // Running average
        avg_batch_size = (avg_batch_size * (total_batches - 1) + batch_size) / total_batches;
    }
    
    void reset() {
        total_batches = 0;
        total_insertions = 0;
        total_removals = 0;
        avg_batch_size = 0.0;
        max_batch_size = 0;
    }
};

} // namespace geofence
} // namespace rises
