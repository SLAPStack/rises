#pragma once

#include <vector>
#include <memory>
#include <nanoflann.hpp>
#include <Eigen/Dense>

namespace rises::spatial {

/**
 * @brief Spatial indexing for efficient neighbor queries
 */
class SpatialIndexer {
public:
    SpatialIndexer() = default;
    
    /**
     * @brief Build spatial index from points
     * @param points Input 2D points
     */
    void buildIndex(const std::vector<Eigen::Vector2f>& points);
    
    /**
     * @brief Find neighbors within radius
     * @param point Query point
     * @param radius Search radius
     * @return Indices of neighboring points
     */
    std::vector<std::size_t> findNeighbors(const Eigen::Vector2f& point, float radius) const;
    
    /**
     * @brief Find k nearest neighbors
     * @param point Query point
     * @param k Number of neighbors
     * @return Indices and squared distances of neighbors
     */
    std::vector<std::pair<std::size_t, float>> findKNearestNeighbors(
        const Eigen::Vector2f& point, std::size_t k) const;
    
    /**
     * @brief Check if index is built and ready
     */
    [[nodiscard]] bool isReady() const { return kdtree_ != nullptr; }
    
    /**
     * @brief Get number of indexed points
     */
    [[nodiscard]] std::size_t size() const { return point_cloud_.pts.size(); }

private:
    // nanoflann point cloud adapter for Eigen vectors
    struct PointCloud {
        std::vector<Eigen::Vector2f> pts;
        
        inline std::size_t kdtree_get_point_count() const { return pts.size(); }
        
        inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
            return dim == 0 ? pts[idx].x() : pts[idx].y();
        }
        
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud>,
        PointCloud,
        2, /* dimensions */
        std::size_t /* index type */
    >;
    
    PointCloud point_cloud_;
    std::unique_ptr<KDTree> kdtree_;
};

/**
 * @brief Point cloud filtering and preprocessing utilities
 */
class PointCloudFilter {
public:
    /**
     * @brief Remove outliers using statistical analysis
     * @param points Input points
     * @param factor Standard deviation factor for outlier detection
     * @param indexer Spatial indexer (will be built if not ready)
     * @return Filtered points
     */
    static std::vector<Eigen::Vector2f> removeOutliers(
        const std::vector<Eigen::Vector2f>& points,
        float factor,
        SpatialIndexer& indexer);
    
    /**
     * @brief Compute adaptive threshold based on local point density
     * @param points Input points
     * @param base_threshold Base threshold value
     * @return Adaptive threshold value
     */
    static float computeAdaptiveThreshold(
        const std::vector<Eigen::Vector2f>& points,
        float base_threshold);
    
    /**
     * @brief Downsample points using voxel grid
     * @param points Input points
     * @param voxel_size Size of voxel grid cells
     * @return Downsampled points
     */
    static std::vector<Eigen::Vector2f> voxelGridFilter(
        const std::vector<Eigen::Vector2f>& points,
        float voxel_size);
};

} // namespace rises::spatial