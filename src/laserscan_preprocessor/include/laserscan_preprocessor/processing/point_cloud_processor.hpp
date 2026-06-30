#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace rises::processing {

/**
 * @brief Configuration for a single laser sensor
 */
struct LaserConfig {
    std::string frame_id;
    float height;
};

/**
 * @brief Point cloud conversion and transformation utilities
 */
class PointCloudProcessor {
public:
    explicit PointCloudProcessor(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : tf_buffer_(tf_buffer) {}
    
    /**
     * @brief Convert multiple LaserScan messages to PointCloud2
     * @param scans Vector of synchronized LaserScan messages
     * @param laser_configs Configuration for each laser
     * @param global_frame Target frame for output
     * @return Combined PointCloud2 message
     */
    [[nodiscard]] sensor_msgs::msg::PointCloud2 convertToPointCloud2(
        const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>& scans,
        const std::vector<LaserConfig>& laser_configs,
        const std::string& global_frame) const;
    
    /**
     * @brief Convert LaserScan messages to Eigen point cloud
     * @param scans Vector of synchronized LaserScan messages
     * @param laser_configs Configuration for each laser
     * @return 3xN Eigen matrix of points
     */
    Eigen::Matrix<float, 3, Eigen::Dynamic> convertToEigenCloud(
        const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>& scans,
        const std::vector<LaserConfig>& laser_configs) const;
    
    /**
     * @brief Extract 2D points from PointCloud2 message
     * @param cloud Input PointCloud2
     * @return Vector of 2D points
     */
    static std::vector<Eigen::Vector2f> extractPoints2D(
        const sensor_msgs::msg::PointCloud2& cloud);
    
    /**
     * @brief Segment point cloud using distance and angle thresholds
     * @param cloud_in Input PointCloud2
     * @param base_distance_threshold Distance threshold for segmentation
     * @param angle_threshold_rad Angle threshold (radians) for segmentation
     * @return PointCloud2 with additional "segment_id" field
     */
    static sensor_msgs::msg::PointCloud2 segmentPointCloud(
        const sensor_msgs::msg::PointCloud2& cloud_in,
        float base_distance_threshold,
        float angle_threshold_rad);
    
    /**
     * @brief Transform point cloud to target frame if needed
     * @param cloud Input/output PointCloud2
     * @param target_frame Desired target frame
     * @return True if transformation successful
     */
    [[nodiscard]] bool transformToFrame(
        sensor_msgs::msg::PointCloud2& cloud,
        const std::string& target_frame) const;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace rises::processing