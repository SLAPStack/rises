#include "laserscan_preprocessor/laserscan_preprocessor_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace rises {

LaserPreprocessorNode::LaserPreprocessorNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("laserscan_preprocessor", options),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {

  RCLCPP_INFO(
      this->get_logger(),
      "[CONSTRUCTOR] LaserPreprocessorNode initialized with LASER_COUNT=%zu",
      static_cast<std::size_t>(LASER_COUNT));

  // Auto-activation setup
  this->declare_parameter<bool>("auto_activate", false);
  const bool auto_activate = this->get_parameter("auto_activate").as_bool();
  if (auto_activate) {
    this->auto_transition_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { this->auto_transition(); });
    RCLCPP_INFO(this->get_logger(),
                "[CONSTRUCTOR] Auto-activation enabled - node will "
                "automatically configure and activate");
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[CONFIGURE] Configuring node...");

  try {
    // Load configuration from parameters
    loadConfiguration();

    // Validate laser count matches build-time configuration
    if (laser_configs_.size() != static_cast<std::size_t>(LASER_COUNT)) {
      RCLCPP_ERROR(
          this->get_logger(),
          "[CONFIGURE] Configuration error: %zu lasers configured but "
          "LASER_COUNT=%zu! "
          "Recompile with -DLASER_COUNT=%zu or configure exactly %zu lasers",
          laser_configs_.size(), static_cast<std::size_t>(LASER_COUNT),
          laser_configs_.size(), static_cast<std::size_t>(LASER_COUNT));
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::FAILURE;
    }

    // Create publishers
    world_scan_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("world_scan", 10);
    processed_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "processed_scan", 10);
    obstacles_pub_ =
        this->create_publisher<rises_interfaces::msg::ObstacleArray>(
            "obstacles", 10);

    // Initialize modular components
    point_cloud_processor_ =
        std::make_unique<processing::PointCloudProcessor>(tf_buffer_);
    initializeSegmentation();
    spatial_indexer_ = std::make_unique<spatial::SpatialIndexer>();

    // Setup individual laser subscribers (no synchronization)
    setupLaserSubscribers();

    // Diagnostic updater — publishes to /diagnostics at 1 Hz
    this->diagnostic_updater_ =
        std::make_unique<diagnostic_updater::Updater>(this, 1.0);
    this->diagnostic_updater_->setHardwareID("laserscan_preprocessor");
    this->diagnostic_updater_->add(
        "preprocessor_status",
        std::bind(&LaserPreprocessorNode::produceDiagnostics, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[CONFIGURE] Node configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[CONFIGURE] Configuration failed: %s",
                 e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
}

void LaserPreprocessorNode::produceDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  const int64_t last_ns = this->last_scan_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = this->now().nanoseconds();
  const double age_sec =
      (last_ns > 0) ? static_cast<double>(now_ns - last_ns) / 1e9 : -1.0;
  const int64_t scans = this->scan_count_.load(std::memory_order_relaxed);
  const int64_t failures = this->failed_scans_.load(std::memory_order_relaxed);

  if (age_sec < 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "No scans received yet");
  } else if (age_sec > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "LiDAR data stale (last scan " + std::to_string(age_sec) +
                     "s ago)");
  } else if (failures > 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Preprocessor operational with " + std::to_string(failures) +
                     " scan failures");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Preprocessor operational");
  }

  stat.add("last_scan_age_sec", age_sec);
  stat.add("total_scans_processed", scans);
  stat.add("failed_scans", failures);
  stat.add("laser_count", static_cast<int64_t>(LASER_COUNT));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[ACTIVATE] Activating node...");

  try {
    // Activate publishers
    world_scan_pub_->on_activate();
    processed_scan_pub_->on_activate();
    obstacles_pub_->on_activate();

    RCLCPP_INFO(this->get_logger(), "[ACTIVATE] Node activated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[ACTIVATE] Activation failed: %s",
                 e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[DEACTIVATE] Deactivating node...");

  // Deactivate publishers
  world_scan_pub_->on_deactivate();
  processed_scan_pub_->on_deactivate();
  obstacles_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "[DEACTIVATE] Node deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "[CLEANUP] Cleaning up node...");

  try {
    // Reset publishers
    world_scan_pub_.reset();
    processed_scan_pub_.reset();
    obstacles_pub_.reset();

    // Reset modular components
    laser_subs_.clear();
    point_cloud_processor_.reset();
    spatial_indexer_.reset();

    // Clear configurations
    laser_configs_.clear();
    laser_frame_ids_.clear();

    RCLCPP_INFO(this->get_logger(), "[CLEANUP] Node cleaned up successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[CLEANUP] Cleanup failed: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(),
              "[SHUTDOWN] Shutting down node from state: %s",
              state.label().c_str());

  try {
    // Deactivate if currently active
    if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      on_deactivate(state);
    }

    // Cleanup if configured
    if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      on_cleanup(state);
    }

    // Final cleanup
    tf_listener_.reset();
    tf_buffer_.reset();

    RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Node shutdown complete");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[SHUTDOWN] Shutdown failed: %s",
                 e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LaserPreprocessorNode::on_error(const rclcpp_lifecycle::State &state) {
  RCLCPP_ERROR(this->get_logger(), "[ERROR] Error occurred in state: %s",
               state.label().c_str());

  try {
    // Attempt graceful cleanup
    laser_subs_.clear();

    // Deactivate publishers if they exist
    if (world_scan_pub_) {
      world_scan_pub_->on_deactivate();
    }
    if (processed_scan_pub_) {
      processed_scan_pub_->on_deactivate();
    }
    if (obstacles_pub_) {
      obstacles_pub_->on_deactivate();
    }

    RCLCPP_INFO(this->get_logger(), "[ERROR] Error cleanup complete");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Error cleanup failed: %s",
                 e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::FAILURE;
  }
}

void LaserPreprocessorNode::auto_transition() {
  const uint8_t current_state = this->get_current_state().id();

  // Skip if we already processed this state
  if (current_state == this->last_transition_state_) {
    return;
  }

  this->last_transition_state_ = current_state;

  switch (current_state) {
  case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
    RCLCPP_INFO(get_logger(),
                "[AUTO-TRANSITION] Auto-configuring lifecycle node");
    this->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    break;

  case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    RCLCPP_INFO(get_logger(),
                "[AUTO-TRANSITION] Auto-activating lifecycle node");
    this->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    break;

  case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    RCLCPP_INFO(
        get_logger(),
        "[AUTO-TRANSITION] Lifecycle node active, disabling auto-transition");
    auto_transition_timer_->cancel();
    break;

  default:
    // Do nothing for other states
    break;
  }
}

void LaserPreprocessorNode::loadConfiguration() {
  // Declare and get parameters
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("tf_prefix", "");
  this->declare_parameter("distance_threshold", 0.2);
  this->declare_parameter("angle_threshold_deg", 30.0);
  this->declare_parameter("dbscan_eps", 0.15);
  this->declare_parameter("dbscan_min_points", 3);
  this->declare_parameter("region_grow_threshold", 0.1);
  this->declare_parameter("min_segment_size", 3);
  this->declare_parameter("outlier_removal_factor", 1.5);
  this->declare_parameter("use_adaptive_thresholding", true);
  this->declare_parameter("publish_points_only", false);
  this->declare_parameter("ransac_max_iterations", 100);
  this->declare_parameter("ransac_inlier_threshold", 0.05);
  this->declare_parameter("ransac_min_inliers", 3);
  this->declare_parameter("laser_topics", std::vector<std::string>());
  // laser_frame_ids: TF frame names for each laser scanner
  // Used to transform scan data from laser frame to global frame
  this->declare_parameter("laser_frame_ids",
                          std::vector<std::string>{"laser_link"});

  // Load configuration
  global_frame_ = this->get_parameter("global_frame").as_string();
  tf_prefix_ = this->get_parameter("tf_prefix").as_string();

  config_.distance_threshold =
      this->get_parameter("distance_threshold").as_double();
  config_.angle_threshold =
      this->get_parameter("angle_threshold_deg").as_double() * M_PI / 180.0;
  config_.dbscan_eps = this->get_parameter("dbscan_eps").as_double();
  config_.dbscan_min_points = this->get_parameter("dbscan_min_points").as_int();
  config_.region_grow_threshold =
      this->get_parameter("region_grow_threshold").as_double();
  config_.min_segment_size = this->get_parameter("min_segment_size").as_int();
  config_.outlier_removal_factor =
      this->get_parameter("outlier_removal_factor").as_double();
  config_.use_adaptive_thresholding =
      this->get_parameter("use_adaptive_thresholding").as_bool();
  config_.publish_points_only =
      this->get_parameter("publish_points_only").as_bool();
  config_.ransac_max_iterations =
      this->get_parameter("ransac_max_iterations").as_int();
  config_.ransac_inlier_threshold =
      this->get_parameter("ransac_inlier_threshold").as_double();
  config_.ransac_min_inliers =
      this->get_parameter("ransac_min_inliers").as_int();

  // Load laser configurations
  std::vector<std::string> laser_topics =
      this->get_parameter("laser_topics").as_string_array();

  // Get laser_frame_ids as string array (TF frame names for each laser)
  laser_frame_ids_ = this->get_parameter("laser_frame_ids").as_string_array();

  // Validate we have exactly LASER_COUNT frame IDs specified
  if (laser_frame_ids_.size() != LASER_COUNT) {
    RCLCPP_ERROR(this->get_logger(),
                 "[CONFIG] Must specify exactly %zu laser_frame_ids, got %zu. "
                 "Example: laser_frame_ids:=['laser_link'] or ['front_laser', "
                 "'rear_laser']",
                 static_cast<std::size_t>(LASER_COUNT),
                 laser_frame_ids_.size());
    throw std::runtime_error("Incorrect number of laser_frame_ids");
  }

  // Prepend tf_prefix to each frame ID if not already present
  for (std::size_t i = 0; i < LASER_COUNT; ++i) {
    if (laser_frame_ids_[i].empty()) {
      RCLCPP_ERROR(this->get_logger(), "[CONFIG] laser_frame_ids[%zu] is empty",
                   i);
      throw std::runtime_error("Empty laser frame ID");
    }

    // Prepend tf_prefix if provided and not already present
    if (!tf_prefix_.empty() &&
        laser_frame_ids_[i].find(tf_prefix_ + "_") != 0) {
      laser_frame_ids_[i] = tf_prefix_ + "_" + laser_frame_ids_[i];
      RCLCPP_INFO(this->get_logger(), "[CONFIG] Laser %zu frame: %s", i,
                  laser_frame_ids_[i].c_str());
    }

    processing::LaserConfig cfg;
    cfg.frame_id = laser_frame_ids_[i];
    cfg.height = 0.0f; // Default height
    laser_configs_.push_back(cfg);
  }

  RCLCPP_INFO(this->get_logger(),
              "[CONFIG] Configured for %zu lasers with distance_threshold=%.3f",
              static_cast<std::size_t>(LASER_COUNT),
              config_.distance_threshold);
}

void LaserPreprocessorNode::setupLaserSubscribers() {
  // Load laser topics from configuration
  const std::vector<std::string> laser_topics =
      this->get_parameter("laser_topics").as_string_array();

  if (laser_topics.size() != LASER_COUNT) {
    RCLCPP_ERROR(this->get_logger(),
                 "[SUBS] Must specify exactly %zu laser_topics, got %zu",
                 static_cast<std::size_t>(LASER_COUNT), laser_topics.size());
    throw std::runtime_error("Incorrect number of laser_topics");
  }

  this->laser_topics_ = laser_topics;

  // Create individual subscription for each laser (no synchronization)
  for (std::size_t i = 0; i < LASER_COUNT; ++i) {
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topics[i], rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
              this->onLaserScan(msg, i);
            });

    this->laser_subs_.push_back(sub);

    RCLCPP_INFO(this->get_logger(),
                "[SUBS] Subscribed to laser %zu: %s -> frame %s", i,
                laser_topics[i].c_str(), this->laser_frame_ids_[i].c_str());
  }
}

void LaserPreprocessorNode::initializeSegmentation() {
  // Initialize static segmentation strategies (geofence pattern)
  segmentation::SegmentationConfig seg_config;
  seg_config.dbscan_eps = this->config_.dbscan_eps;
  seg_config.dbscan_min_points = this->config_.dbscan_min_points;
  seg_config.region_grow_threshold = this->config_.region_grow_threshold;
  seg_config.min_segment_size = this->config_.min_segment_size;
  seg_config.distance_threshold = this->config_.distance_threshold;
  seg_config.angle_threshold = this->config_.angle_threshold;

  segmentation::SegmentationStrategy::initialize(seg_config);

  RCLCPP_INFO(this->get_logger(), "[SEGMENTATION] Initialized static "
                                  "strategies (DBSCAN, RegionGrow, Distance)");
}

void LaserPreprocessorNode::onLaserScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
    const std::size_t laser_idx) {

  if (laser_idx >= this->laser_configs_.size()) {
    RCLCPP_WARN(this->get_logger(), "[SCAN] Invalid laser index: %zu",
                laser_idx);
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
               "[SCAN] Received scan from laser %zu: %zu ranges, frame=%s",
               laser_idx, scan->ranges.size(), scan->header.frame_id.c_str());

  this->last_scan_ns_.store(this->now().nanoseconds(),
                            std::memory_order_relaxed);
  this->scan_count_.fetch_add(1, std::memory_order_relaxed);

  // Cache lidar angular resolution for forwarding in ObstacleArray messages
  if (scan->angle_increment > 0.0f) {
    this->last_angle_increment_ = scan->angle_increment;
  }

  try {
    // Convert single scan to point cloud
    sensor_msgs::msg::PointCloud2 cloud_msg =
        this->convertScanToPointCloud(scan, this->laser_configs_[laser_idx]);

    if (cloud_msg.width * cloud_msg.height == 0) {
      RCLCPP_DEBUG(this->get_logger(),
                   "[SCAN] Empty point cloud from laser %zu, skipping",
                   laser_idx);
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "[SCAN] Converted to %u points, frame=%s",
                 cloud_msg.width * cloud_msg.height,
                 cloud_msg.header.frame_id.c_str());

    // Transform to global frame if needed
    if (cloud_msg.header.frame_id != this->global_frame_) {
      if (!this->point_cloud_processor_->transformToFrame(
              cloud_msg, this->global_frame_)) {
        RCLCPP_WARN(
            this->get_logger(),
            "[SCAN] Failed to transform cloud from laser %zu to %s, skipping",
            laser_idx, this->global_frame_.c_str());
        return;
      }
    }

    // Publish raw world scan
    publishPointCloud2(this->world_scan_pub_, cloud_msg, this->global_frame_);

    // Process point cloud through modular pipeline
    processPointCloud(cloud_msg);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "[SCAN] Processing failed for laser %zu: %s", laser_idx,
                 e.what());
    // Publish an empty ObstacleArray carrying the scan's timestamp so
    // downstream nodes (safety, aggregator, geofence) can distinguish
    // "processing failed" from "no message arrived" / "no obstacles
    // detected". Without this, the absence of a message looks identical to
    // a legitimate clear scan -- a safety-relevant ghost-clear.
    // failed_scans_ is bumped so the diagnostic updater surfaces the
    // counter to operators.
    this->failed_scans_.fetch_add(1, std::memory_order_relaxed);
    if (obstacles_pub_ && obstacles_pub_->is_activated()) {
      rises_interfaces::msg::ObstacleArray empty_array;
      empty_array.header.stamp = scan->header.stamp;
      empty_array.header.frame_id = this->global_frame_;
      empty_array.angle_increment = scan->angle_increment;
      obstacles_pub_->publish(empty_array);
    }
  }
}

sensor_msgs::msg::PointCloud2 LaserPreprocessorNode::convertScanToPointCloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
    const processing::LaserConfig &config) {

  sensor_msgs::msg::PointCloud2 cloud_msg;

  // Count valid points
  std::size_t total_points = 0;
  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];
    if (std::isfinite(range) && range >= scan->range_min &&
        range <= scan->range_max) {
      total_points++;
    }
  }

  if (total_points == 0) {
    return cloud_msg;
  }

  // Setup PointCloud2 message
  cloud_msg.header = scan->header;
  cloud_msg.height = 1;
  cloud_msg.width = total_points;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(total_points);

  // Fill point data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  float angle = scan->angle_min;
  for (std::size_t i = 0; i < scan->ranges.size();
       ++i, angle += scan->angle_increment) {
    const float range = scan->ranges[i];

    if (!std::isfinite(range) || range < scan->range_min ||
        range > scan->range_max) {
      continue;
    }

    // Convert to Cartesian coordinates in laser frame
    *iter_x = range * std::cos(angle);
    *iter_y = range * std::sin(angle);
    *iter_z = config.height;

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud_msg;
}

void LaserPreprocessorNode::processPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud) {
  RCLCPP_DEBUG(this->get_logger(),
               "[PROCESS] Processing point cloud with %u points",
               cloud.width * cloud.height);

  if (config_.publish_points_only) {
    RCLCPP_INFO(
        this->get_logger(),
        "[PROCESS] Points-only mode: publishing raw cloud and point obstacles");
    // Publish raw point cloud without segmentation
    publishPointCloud2(processed_scan_pub_, cloud, global_frame_);

    // Publish all points as individual point obstacles (no segmentation/shape
    // fitting)
    publishPointObstacles(cloud, global_frame_);
    return;
  }

  RCLCPP_DEBUG(
      this->get_logger(),
      "[PROCESS] Segmentation mode: performing point cloud segmentation");

  // Segment the point cloud using modular segmentation strategy
  sensor_msgs::msg::PointCloud2 segmented_cloud =
      point_cloud_processor_->segmentPointCloud(
          cloud, config_.distance_threshold, config_.angle_threshold);

  // Publish processed scan
  publishPointCloud2(processed_scan_pub_, segmented_cloud, global_frame_);

  // Extract and publish obstacles
  extractAndPublishObstacles(segmented_cloud, global_frame_);
}

void LaserPreprocessorNode::extractAndPublishObstacles(
    const sensor_msgs::msg::PointCloud2 &segmented_cloud,
    const std::string &frame_id) {

  RCLCPP_DEBUG(this->get_logger(),
               "[OBSTACLES] Extracting obstacles from segmented cloud");

  // Extract 2D points from segmented cloud
  const std::vector<Eigen::Vector2f> points_2d =
      processing::PointCloudProcessor::extractPoints2D(segmented_cloud);

  if (points_2d.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "[OBSTACLES] No points to process");
    rises_interfaces::msg::ObstacleArray empty_array;
    empty_array.header.frame_id = frame_id;
    // Use cloud's timestamp even for empty array
    empty_array.header.stamp = segmented_cloud.header.stamp;
    empty_array.angle_increment = this->last_angle_increment_;
    obstacles_pub_->publish(empty_array);
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
               "[OBSTACLES] Extracted %zu points, grouping by segment_id",
               points_2d.size());

  // Extract segment IDs from the cloud
  std::map<int, std::vector<Eigen::Vector2f>> segments;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(segmented_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(segmented_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<int32_t> iter_seg(segmented_cloud,
                                                          "segment_id");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_seg) {
    const int segment_id = *iter_seg;
    if (segment_id >= 0) { // Skip invalid segments
      segments[segment_id].emplace_back(*iter_x, *iter_y);
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "[OBSTACLES] Found %zu segments",
               segments.size());
  for (const auto &[seg_id, seg_points] : segments) {
    RCLCPP_DEBUG(this->get_logger(), "[OBSTACLES]   Segment %d: %zu points",
                 seg_id, seg_points.size());
  }

  // Create shape fitters
  const std::unique_ptr<shapes::LineFitter> line_fitter =
      std::make_unique<shapes::LineFitter>(config_.ransac_max_iterations,
                                           config_.ransac_inlier_threshold,
                                           config_.ransac_min_inliers);

  const std::unique_ptr<shapes::CircleFitter> circle_fitter =
      std::make_unique<shapes::CircleFitter>(config_.ransac_max_iterations,
                                             config_.ransac_inlier_threshold,
                                             config_.ransac_min_inliers);

  // Fit shapes to segments and populate obstacle array
  rises_interfaces::msg::ObstacleArray obstacle_array;
  obstacle_array.header.frame_id = frame_id;
  // Use cloud's original timestamp for TF consistency
  obstacle_array.header.stamp = segmented_cloud.header.stamp;
  obstacle_array.angle_increment = this->last_angle_increment_;

  for (const auto &[segment_id, segment_points] : segments) {
    if (segment_points.size() < config_.min_segment_size) {
      continue; // Skip small segments
    }

    // Try line fitting first
    const float line_confidence =
        line_fitter->getConfidence(segment_points, 0.0f);
    const float circle_confidence =
        circle_fitter->getConfidence(segment_points, 0.0f);

    rises_interfaces::msg::Obstacle obstacle;
    if (line_confidence > circle_confidence && line_confidence > 0.5f) {
      obstacle = line_fitter->fitShape(segment_id, segment_points, 0.0f);
    } else if (circle_confidence > 0.5f) {
      obstacle = circle_fitter->fitShape(segment_id, segment_points, 0.0f);
    } else {
      // Use point representation for ambiguous segments
      const std::unique_ptr<shapes::PointFitter> point_fitter =
          std::make_unique<shapes::PointFitter>(false);
      obstacle = point_fitter->fitShape(segment_id, segment_points, 0.0f);
    }

    if (!obstacle.vertices.empty()) {
      obstacle_array.obstacles.push_back(obstacle);
    }
  }

  RCLCPP_DEBUG(this->get_logger(),
               "[OBSTACLES] Extracted %zu obstacles from %zu segments",
               obstacle_array.obstacles.size(), segments.size());

  {
    const rclcpp::Time scan_time(segmented_cloud.header.stamp);
    const double processing_ms = (this->now() - scan_time).seconds() * 1000.0;
    RCLCPP_DEBUG(this->get_logger().get_child("perf"),
                 "[PERF] laserscan→obstacle_array: %.2f ms (%zu obstacles from "
                 "%zu segments)",
                 processing_ms, obstacle_array.obstacles.size(),
                 segments.size());
  }

  obstacles_pub_->publish(obstacle_array);
}

void LaserPreprocessorNode::publishPointObstacles(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &frame_id) {

  RCLCPP_DEBUG(this->get_logger(),
               "[OBSTACLES] Extracting point obstacles from cloud");

  // Extract all points from cloud
  const std::vector<Eigen::Vector2f> points_2d =
      processing::PointCloudProcessor::extractPoints2D(cloud);

  RCLCPP_DEBUG(this->get_logger(), "[OBSTACLES] Extracted %zu 2D points",
               points_2d.size());

  if (points_2d.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "[OBSTACLES] No points extracted, publishing empty array");
    rises_interfaces::msg::ObstacleArray empty_array;
    empty_array.header.frame_id = frame_id;
    // Use cloud's timestamp even for empty array
    empty_array.header.stamp = cloud.header.stamp;
    empty_array.angle_increment = this->last_angle_increment_;
    obstacles_pub_->publish(empty_array);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "[OBSTACLES] Creating POINT obstacle with %zu vertices",
              points_2d.size());

  // Create single obstacle with all points (no segmentation)
  rises_interfaces::msg::Obstacle obstacle;
  obstacle.id = 0;
  obstacle.type = rises_interfaces::msg::Obstacle::POINT;

  float sum_x = 0.0f, sum_y = 0.0f;
  for (const auto &pt : points_2d) {
    geometry_msgs::msg::Point vertex;
    vertex.x = pt.x();
    vertex.y = pt.y();
    vertex.z = 0.0f;
    obstacle.vertices.push_back(vertex);
    sum_x += pt.x();
    sum_y += pt.y();
  }
  // Set centroid so downstream nodes (e.g. validation_node) can use position
  // directly
  obstacle.position.x = sum_x / static_cast<float>(points_2d.size());
  obstacle.position.y = sum_y / static_cast<float>(points_2d.size());
  obstacle.position.z = 0.0;

  rises_interfaces::msg::ObstacleArray obstacle_array;
  obstacle_array.header.frame_id = frame_id;
  // Use cloud's original timestamp for TF consistency
  obstacle_array.header.stamp = cloud.header.stamp;
  obstacle_array.angle_increment = this->last_angle_increment_;
  obstacle_array.obstacles.push_back(obstacle);

  RCLCPP_INFO(this->get_logger(),
              "[OBSTACLES] Publishing obstacle array: 1 obstacle, %zu total "
              "vertices, frame=%s",
              obstacle.vertices.size(), frame_id.c_str());
  RCLCPP_DEBUG(this->get_logger(), "[OBSTACLES] Publisher active: %s",
               obstacles_pub_->is_activated() ? "YES" : "NO");

  {
    const rclcpp::Time scan_time(cloud.header.stamp);
    const double processing_ms = (this->now() - scan_time).seconds() * 1000.0;
    RCLCPP_DEBUG(this->get_logger().get_child("perf"),
                 "[PERF] laserscan→obstacle_array: %.2f ms (1 obstacle, %zu "
                 "vertices, points-only mode)",
                 processing_ms, obstacle.vertices.size());
  }

  obstacles_pub_->publish(obstacle_array);
  RCLCPP_DEBUG(this->get_logger(),
               "[OBSTACLES] Obstacle array published successfully");
}

void LaserPreprocessorNode::publishPointCloud2(
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr pub,
    const sensor_msgs::msg::PointCloud2 &cloud_msg,
    const std::string &frame_id) {

  if (!pub || !pub->is_activated()) {
    RCLCPP_DEBUG(this->get_logger(),
                 "[PUBLISH] PointCloud2 publisher not active, skipping");
    return;
  }

  // Cloud should already be in correct frame from transformation
  // Just verify frame_id matches expectation
  if (cloud_msg.header.frame_id != frame_id) {
    RCLCPP_WARN(this->get_logger(),
                "[PUBLISH] Frame mismatch: cloud is in '%s' but expected '%s'",
                cloud_msg.header.frame_id.c_str(), frame_id.c_str());
  }

  RCLCPP_DEBUG(
      this->get_logger(),
      "[PUBLISH] Publishing PointCloud2: %u points, frame=%s, stamp=%f",
      cloud_msg.width * cloud_msg.height, cloud_msg.header.frame_id.c_str(),
      cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9);

  pub->publish(cloud_msg);
}

} // namespace rises

// Node registration
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rises::LaserPreprocessorNode)