#pragma once

#include <vector>
#include <Eigen/Dense>

namespace rises::segmentation {

/**
 * @brief Configuration for segmentation algorithms
 */
struct SegmentationConfig {
    float dbscan_eps{0.15f};
    std::size_t dbscan_min_points{3};
    float region_grow_threshold{0.1f};
    std::size_t min_segment_size{3};
    float distance_threshold{0.2f};
    float angle_threshold{0.5236f};  // 30 degrees in radians
};

/**
 * @brief Static segmentation strategies (geofence pattern)
 * 
 * All strategies are static and initialized once with configuration.
 * No polymorphism or virtual dispatch overhead.
 */
class SegmentationStrategy {
public:
    /**
     * @brief Initialize segmentation with configuration
     */
    static void initialize(const SegmentationConfig& config);
    
    /**
     * @brief DBSCAN clustering
     * @param points Input 2D points
     * @return Vector of cluster IDs (-1 for noise, >=0 for clusters)
     */
    static std::vector<int> dbscan(const std::vector<Eigen::Vector2f>& points);
    
    /**
     * @brief Region growing segmentation
     * @param points Input 2D points
     * @return Vector of cluster IDs (-1 for noise, >=0 for clusters)
     */
    static std::vector<int> regionGrow(const std::vector<Eigen::Vector2f>& points);
    
    /**
     * @brief Distance-based segmentation
     * @param points Input 2D points
     * @return Vector of cluster IDs (-1 for noise, >=0 for clusters)
     */
    static std::vector<int> distanceSegment(const std::vector<Eigen::Vector2f>& points);
    
    /**
     * @brief Get current configuration
     */
    static const SegmentationConfig& getConfig();

private:
    static SegmentationConfig config_;
};

} // namespace rises::segmentation