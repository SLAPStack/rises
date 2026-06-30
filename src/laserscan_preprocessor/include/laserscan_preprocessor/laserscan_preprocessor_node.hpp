#pragma once

#include <rises_interfaces/msg/obstacle_array.hpp>
#include <atomic>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Diagnostics
#include "diagnostic_updater/diagnostic_updater.hpp"

// New modular components
#include "laserscan_preprocessor/processing/point_cloud_processor.hpp"
#include "laserscan_preprocessor/segmentation/segmentation_strategy.hpp"
#include "laserscan_preprocessor/shapes/shape_fitter.hpp"
#include "laserscan_preprocessor/spatial/spatial_indexer.hpp"
#include "laserscan_preprocessor/sync/laser_synchronizer.hpp"

namespace rises {

/**
 * @brief Configuration for laser preprocessing pipeline
 */
struct PreprocessingConfig {
  // Segmentation parameters
  float distance_threshold{0.2f};
  float angle_threshold{0.5236f}; // 30 degrees in radians

  // Advanced segmentation parameters
  float dbscan_eps{0.15f};
  std::size_t dbscan_min_points{3};
  float region_grow_threshold{0.1f};
  std::size_t min_segment_size{3};
  float outlier_removal_factor{1.5f};
  bool use_adaptive_thresholding{true};
  bool publish_points_only{false};

  // Shape fitting parameters
  int ransac_max_iterations{100};
  float ransac_inlier_threshold{0.05f};
  std::size_t ransac_min_inliers{3};
};

/**
 * @brief Main laser preprocessing node using modular architecture
 *
 * This node follows a clean architectural pattern:
 * - Strategy Pattern for segmentation algorithms
 * - Factory Pattern for shape fitting
 * - Template-based compile-time laser synchronization
 * - Modular processors for different responsibilities
 */
class LaserPreprocessorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LaserPreprocessorNode(const rclcpp::NodeOptions &options);

  // ---------------- Lifecycle Hooks ----------------
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override;

  // Auto-activation helper
  void auto_transition();

private:
  // ---------------- Core Components ----------------
  static constexpr std::size_t LASER_COUNT_CONST = LASER_COUNT;

#ifdef SCAN_POINT_COUNT
  // Compile-time known scan point count for optimization
  static constexpr std::size_t SCAN_POINT_COUNT_CONST = SCAN_POINT_COUNT;
  static constexpr bool HAS_COMPILE_TIME_POINT_COUNT = true;
#else
  // Runtime point count determination
  static constexpr bool HAS_COMPILE_TIME_POINT_COUNT = false;
#endif

  // Modular components (strategies are static, no pointers needed)
  std::unique_ptr<processing::PointCloudProcessor> point_cloud_processor_;
  std::unique_ptr<spatial::SpatialIndexer> spatial_indexer_;

  // Configuration
  PreprocessingConfig config_;
  std::vector<processing::LaserConfig> laser_configs_;

  // Individual laser scan subscribers (no synchronization needed for detection)
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      laser_subs_;

  // Core parameters
  std::vector<std::string> laser_frame_ids_;
  std::vector<std::string> laser_topics_;

  // ---------------- ROS Publishers ----------------
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      world_scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      processed_scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      rises_interfaces::msg::ObstacleArray>::SharedPtr obstacles_pub_;

  // ---------------- TF ----------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;
  std::string tf_prefix_;
  std::string global_frame_;

  // ---------------- Diagnostics ----------------
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::atomic<int64_t> last_scan_ns_{0};
  std::atomic<int64_t> scan_count_{0};
  // Counts scans whose processing threw and produced no obstacle output.
  // Read by produceDiagnostics; exposed so downstream cannot mistake the
  // absence of an ObstacleArray for a "no obstacles present" report.
  std::atomic<int64_t> failed_scans_{0};
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // ---------------- Lidar metadata ----------------
  float last_angle_increment_ = 0.0f; // cached from most recent LaserScan

  // ---------------- Auto-activation ----------------
  rclcpp::TimerBase::SharedPtr auto_transition_timer_;
  uint8_t last_transition_state_ = 0;

  // ---------------- Core Processing Pipeline ----------------

  /**
   * @brief Callback for individual laser scan (no synchronization needed)
   * @param scan Single LaserScan message
   * @param laser_idx Index of the laser that produced this scan
   */
  void onLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                   std::size_t laser_idx);

  /**
   * @brief Process point cloud through segmentation and shape fitting
   * @param cloud Input PointCloud2 message
   */
  void processPointCloud(const sensor_msgs::msg::PointCloud2 &cloud);

  /**
   * @brief Extract and fit obstacles from segmented point cloud
   * @param segmented_cloud PointCloud2 with segment_id field
   * @param frame_id Frame for obstacle messages
   */
  void extractAndPublishObstacles(
      const sensor_msgs::msg::PointCloud2 &segmented_cloud,
      const std::string &frame_id);

  // ---------------- Configuration and Setup ----------------

  /**
   * @brief Load configuration from ROS parameters
   */
  void loadConfiguration();

  /**
   * @brief Setup individual laser scan subscribers
   */
  void setupLaserSubscribers();

  /**
   * @brief Initialize static segmentation strategies
   */
  void initializeSegmentation();

  // ---------------- Publishing Utilities ----------------

  /**
   * @brief Publish PointCloud2 message
   */
  void publishPointCloud2(rclcpp_lifecycle::LifecyclePublisher<
                              sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const sensor_msgs::msg::PointCloud2 &cloud_msg,
                          const std::string &frame_id);

  /**
   * @brief Publish raw points as single point obstacle (no segmentation)
   */
  void publishPointObstacles(const sensor_msgs::msg::PointCloud2 &cloud,
                             const std::string &frame_id);

  /**
   * @brief Convert single laser scan to point cloud
   */
  sensor_msgs::msg::PointCloud2 convertScanToPointCloud(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
      const processing::LaserConfig &config);
};

} // namespace rises
