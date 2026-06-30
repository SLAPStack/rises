#include "laserscan_preprocessor/segmentation/segmentation_strategy.hpp"
#include "laserscan_preprocessor/spatial/spatial_indexer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <queue>

namespace rises::segmentation {

// Static configuration
SegmentationConfig SegmentationStrategy::config_{};

void SegmentationStrategy::initialize(const SegmentationConfig& config) {
    config_ = config;
    RCLCPP_INFO(rclcpp::get_logger("SegmentationStrategy"),
                "Initialized segmentation: dbscan_eps=%.3f, min_points=%zu, distance_threshold=%.3f, angle_threshold=%.3f rad",
                config_.dbscan_eps, config_.dbscan_min_points, config_.distance_threshold, config_.angle_threshold);
}

const SegmentationConfig& SegmentationStrategy::getConfig() {
    return config_;
}

// ---------------- DBSCAN Implementation ----------------

std::vector<int> SegmentationStrategy::dbscan(const std::vector<Eigen::Vector2f>& points) {
    if (points.empty()) return {};
    
    RCLCPP_DEBUG(rclcpp::get_logger("SegmentationStrategy"), 
                "Starting DBSCAN with %zu points, eps=%f, min_points=%zu", 
                points.size(), config_.dbscan_eps, config_.dbscan_min_points);
    
    // Use spatial indexer for efficient neighbor queries
    rises::spatial::SpatialIndexer indexer;
    indexer.buildIndex(points);
    
    std::vector<int> cluster_labels(points.size(), -2); // -2: unvisited, -1: noise, >=0: cluster
    int cluster_id = 0;
    
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (cluster_labels[i] != -2) continue; // Already processed
        
        const std::vector<std::size_t> neighbors = indexer.findNeighbors(points[i], config_.dbscan_eps);
        
        if (neighbors.size() < config_.dbscan_min_points) {
            cluster_labels[i] = -1; // Mark as noise
            continue;
        }
        
        // Start new cluster
        cluster_labels[i] = cluster_id;
        std::queue<std::size_t> seed_set;
        for (const std::size_t neighbor : neighbors) {
            if (neighbor != i) {
                seed_set.push(neighbor);
            }
        }
        
        // Expand cluster
        while (!seed_set.empty()) {
            const std::size_t current = seed_set.front();
            seed_set.pop();
            
            if (cluster_labels[current] == -1) {
                cluster_labels[current] = cluster_id; // Convert noise to border point
            }
            if (cluster_labels[current] != -2) continue; // Already processed
            
            cluster_labels[current] = cluster_id;
            
            const std::vector<std::size_t> current_neighbors = indexer.findNeighbors(points[current], config_.dbscan_eps);
            
            if (current_neighbors.size() >= config_.dbscan_min_points) {
                for (const std::size_t neighbor : current_neighbors) {
                    if (cluster_labels[neighbor] == -2) {
                        seed_set.push(neighbor);
                    }
                }
            }
        }
        
        cluster_id++;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("SegmentationStrategy"), 
                "DBSCAN completed: found %d clusters", cluster_id);
    return cluster_labels;
}

// ---------------- Region Growing Implementation ----------------

std::vector<int> SegmentationStrategy::regionGrow(const std::vector<Eigen::Vector2f>& points) {
    if (points.empty()) return {};
    
    RCLCPP_DEBUG(rclcpp::get_logger("SegmentationStrategy"), 
                "Starting region growing with %zu points", points.size());
    
    // Use spatial indexer for efficient neighbor queries
    rises::spatial::SpatialIndexer indexer;
    indexer.buildIndex(points);
    
    std::vector<int> region_labels(points.size(), -1);
    std::vector<bool> visited(points.size(), false);
    int region_id = 0;
    
    for (std::size_t seed = 0; seed < points.size(); ++seed) {
        if (visited[seed]) continue;
        
        std::queue<std::size_t> region_queue;
        std::vector<std::size_t> current_region;
        
        region_queue.push(seed);
        visited[seed] = true;
        
        while (!region_queue.empty()) {
            const std::size_t current = region_queue.front();
            region_queue.pop();
            current_region.push_back(current);
            
            // Find neighbors within threshold using spatial indexer
            const std::vector<std::size_t> neighbors = indexer.findNeighbors(points[current], config_.region_grow_threshold);
            for (const std::size_t neighbor_idx : neighbors) {
                if (!visited[neighbor_idx]) {
                    visited[neighbor_idx] = true;
                    region_queue.push(neighbor_idx);
                }
            }
        }
        
        // Assign region ID if region is large enough
        if (current_region.size() >= config_.min_segment_size) {
            for (const std::size_t idx : current_region) {
                region_labels[idx] = region_id;
            }
            region_id++;
        }
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("SegmentationStrategy"), 
                "Region growing completed: found %d regions", region_id);
    return region_labels;
}

// ---------------- Distance Segmentation Implementation ----------------

std::vector<int> SegmentationStrategy::distanceSegment(const std::vector<Eigen::Vector2f>& points) {
    if (points.empty()) return {};
    
    std::vector<int> segment_labels(points.size());
    int segment_id = 0;
    
    // First point starts first segment
    segment_labels[0] = segment_id;
    
    Eigen::Vector2f prev_point = points[0];
    Eigen::Vector2f prev_direction(0, 0);
    
    for (std::size_t i = 1; i < points.size(); ++i) {
        const Eigen::Vector2f current_point = points[i];
        Eigen::Vector2f direction = current_point - prev_point;
        const float distance = direction.norm();
        
        // Adaptive distance threshold based on range
        const float range = prev_point.norm();
        const float adaptive_distance_threshold = config_.distance_threshold * (1.0f + range * 0.1f);
        
        bool split_segment = false;
        
        // Check distance threshold
        if (distance > adaptive_distance_threshold) {
            split_segment = true;
        }
        
        // Check angle threshold (if we have previous direction)
        if (i > 1 && prev_direction.norm() > 1e-6f) {
            direction.normalize();
            prev_direction.normalize();
            
            float dot_product = direction.dot(prev_direction);
            dot_product = std::clamp(dot_product, -1.0f, 1.0f);
            const float angle = std::acos(dot_product);
            
            if (angle > config_.angle_threshold) {
                split_segment = true;
            }
        }
        
        if (split_segment) {
            segment_id++;
        }
        
        segment_labels[i] = segment_id;
        
        prev_point = current_point;
        if (direction.norm() > 1e-6f) {
            prev_direction = direction.normalized();
        }
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("SegmentationStrategy"), 
                "Distance segmentation completed: found %d segments", segment_id + 1);
    
    return segment_labels;
}

} // namespace rises::segmentation
