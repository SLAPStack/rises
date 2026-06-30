#include "laserscan_preprocessor/spatial/spatial_indexer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <numeric>

#ifdef USE_SIMD
#include <xsimd/xsimd.hpp>
#endif

namespace rises::spatial {

// ------------ SpatialIndexer Implementation ------------

void SpatialIndexer::buildIndex(const std::vector<Eigen::Vector2f>& points) {
    if (points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("SpatialIndexer"),
                   "[buildIndex] Cannot build KDTree index: empty points vector");
        kdtree_.reset();
        return;
    }
    
    // Copy points to internal storage
    point_cloud_.pts = points;
    
    // Build the KD-tree index
    kdtree_ = std::make_unique<KDTree>(2, point_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree_->buildIndex();
    
    RCLCPP_DEBUG(rclcpp::get_logger("SpatialIndexer"),
                "Built KDTree index for %zu points", points.size());
}

std::vector<std::size_t> SpatialIndexer::findNeighbors(const Eigen::Vector2f& point, float radius) const {
    if (!kdtree_ || radius <= 0.0f) {
        return {};
    }
    
    std::vector<nanoflann::ResultItem<std::size_t, float>> ret_matches;
    nanoflann::SearchParameters params;
    
    float query_pt[2] = {point.x(), point.y()};
    
    std::size_t n_matches = kdtree_->radiusSearch(&query_pt[0], radius * radius, ret_matches, params);
    
    std::vector<std::size_t> indices;
    indices.reserve(n_matches);
    
    for (const auto& match : ret_matches) {
        indices.push_back(match.first);
    }
    
    return indices;
}

std::vector<std::pair<std::size_t, float>> SpatialIndexer::findKNearestNeighbors(
    const Eigen::Vector2f& point, std::size_t k) const {

    if (!kdtree_ || k == 0 || k > point_cloud_.pts.size()) {
        return {};
    }

    std::vector<std::size_t> ret_indices(k);
    std::vector<float> out_dists_sqr(k);

    float query_pt[2] = {point.x(), point.y()};

    std::size_t n_results = kdtree_->knnSearch(&query_pt[0], k, &ret_indices[0], &out_dists_sqr[0]);

    std::vector<std::pair<std::size_t, float>> result;
    result.reserve(n_results);

    for (std::size_t i = 0; i < n_results; ++i) {
        result.emplace_back(ret_indices[i], out_dists_sqr[i]);
    }

    return result;
}

// ------------ PointCloudFilter Implementation ------------

std::vector<Eigen::Vector2f> PointCloudFilter::removeOutliers(
    const std::vector<Eigen::Vector2f>& points,
    float factor,
    SpatialIndexer& indexer) {
    
    if (points.empty()) {
        return {};
    }
    
    // Build index if not ready
    if (!indexer.isReady()) {
        indexer.buildIndex(points);
    }
    
    const std::size_t k_neighbors = std::min(static_cast<std::size_t>(8), points.size());
    
    // Compute distances to k nearest neighbors for each point
    std::vector<float> mean_distances;
    mean_distances.reserve(points.size());
    
    for (const auto& point : points) {
        auto neighbors = indexer.findKNearestNeighbors(point, k_neighbors);
        
        float sum_dist = 0.0f;
        std::size_t valid_neighbors = 0;
        
        for (const auto& neighbor : neighbors) {
            float dist = std::sqrt(neighbor.second);
            if (dist > 1e-6f) { // Skip self
                sum_dist += dist;
                valid_neighbors++;
            }
        }
        
        float mean_dist = (valid_neighbors > 0) ? (sum_dist / valid_neighbors) : 0.0f;
        mean_distances.push_back(mean_dist);
    }
    
    // Compute global statistics
    float sum = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0f);
    float mean = sum / mean_distances.size();
    
    float variance = 0.0f;
#ifdef USE_SIMD
    {
        using simd_t = xsimd::batch<float>;
        constexpr std::size_t batch_size = simd_t::size;
        const std::size_t m = mean_distances.size();
        const simd_t mean_v(mean);
        simd_t var_acc(0.0f);
        const std::size_t simd_end = m - (m % batch_size);
        for (std::size_t i = 0; i < simd_end; i += batch_size) {
            const simd_t d = xsimd::load_unaligned(&mean_distances[i]) - mean_v;
            var_acc = xsimd::fma(d, d, var_acc);
        }
        float tmp[batch_size];
        xsimd::store_unaligned(tmp, var_acc);
        for (std::size_t j = 0; j < batch_size; ++j) {
            variance += tmp[j];
        }
        for (std::size_t i = simd_end; i < m; ++i) {
            const float d = mean_distances[i] - mean;
            variance += d * d;
        }
    }
#else
    for (float dist : mean_distances) {
        variance += (dist - mean) * (dist - mean);
    }
#endif
    variance /= mean_distances.size();
    float stddev = std::sqrt(variance);
    
    // Filter outliers
    float threshold = mean + factor * stddev;
    
    std::vector<Eigen::Vector2f> filtered_points;
    filtered_points.reserve(points.size());
    
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (mean_distances[i] <= threshold) {
            filtered_points.push_back(points[i]);
        }
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("PointCloudFilter"),
                "Filtered %zu points to %zu points (threshold: %.3f)",
                points.size(), filtered_points.size(), threshold);
    
    return filtered_points;
}

float PointCloudFilter::computeAdaptiveThreshold(
    const std::vector<Eigen::Vector2f>& points,
    float base_threshold) {
    
    if (points.empty()) {
        return base_threshold;
    }
    
    // Estimate local density
    SpatialIndexer indexer;
    indexer.buildIndex(points);
    
    const float search_radius = base_threshold * 2.0f;
    float density_sum = 0.0f;
    std::size_t valid_points = 0;
    
    for (const auto& point : points) {
        auto neighbors = indexer.findNeighbors(point, search_radius);
        if (neighbors.size() > 1) { // Exclude self
            float density = static_cast<float>(neighbors.size() - 1) / (M_PI * search_radius * search_radius);
            density_sum += density;
            valid_points++;
        }
    }
    
    if (valid_points == 0) {
        return base_threshold;
    }
    
    float avg_density = density_sum / valid_points;
    
    // Adaptive scaling: higher density -> lower threshold
    float adaptive_factor = std::max(0.5f, std::min(2.0f, 1.0f / (avg_density + 0.1f)));
    
    return base_threshold * adaptive_factor;
}

} // namespace rises::spatial