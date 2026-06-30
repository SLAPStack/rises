#include "geofence/spatial/node/geofence_config.hpp"

namespace rises {

GeofenceConfig GeofenceConfig::fromNode(rclcpp_lifecycle::LifecycleNode &node) {
  GeofenceConfig cfg;

  // Shared parameters (spatial node's publish_report_always default is true).
  GeofenceCommonConfig::declareAndRead(node, cfg, /*publish_report_always_default=*/true);

  // --- Spatial-backend-specific parameters ---

  // Core detection
  node.declare_parameter<double>("correspondence_tolerance", 0.10);
  node.declare_parameter<bool>("process_points_only", true);
  node.declare_parameter<bool>("ignore_outside_warehouse", false);

  // Segmentation
  node.declare_parameter<double>("segment_min_gap", 0.05);
  node.declare_parameter<double>("segment_gap_multiplier", 8.0);
  node.declare_parameter<double>("lidar_angle_increment", 0.001745);
  node.declare_parameter<double>("line_fit_tolerance", 0.05);
  node.declare_parameter<double>("error_segment_tracker_cell_size", 0.3);
  node.declare_parameter<double>("error_segment_tracker_max_drift", 1.0);
  node.declare_parameter<int>("min_segment_points", 3);
  node.declare_parameter<bool>("enable_error_segment_tracking", true);
  node.declare_parameter<bool>("report_error_segments_as_points", false);
  node.declare_parameter<double>("outlier_filter_distance", 0.10);

  // Height awareness
  node.declare_parameter<bool>("enable_height_awareness", false);
  node.declare_parameter<std::string>("lidar_height_source", "tf");
  node.declare_parameter<double>("lidar_height_fixed", 0.0);
  node.declare_parameter<std::string>("lidar_frame", "laser_link");
  node.declare_parameter<double>("height_match_tolerance", 0.05);
  node.declare_parameter<std::string>("height_match_mode", "intersection");

  // Latency recording (empty string = disabled). Read by the node directly.
  node.declare_parameter<std::string>("latency_output_file", "");

  // --- Read spatial values ---
  cfg.correspondence_tolerance = static_cast<float>(
      node.get_parameter("correspondence_tolerance").as_double());
  cfg.process_points_only = node.get_parameter("process_points_only").as_bool();
  cfg.ignore_outside_warehouse =
      node.get_parameter("ignore_outside_warehouse").as_bool();

  cfg.segment_min_gap =
      static_cast<float>(node.get_parameter("segment_min_gap").as_double());
  cfg.segment_gap_multiplier = static_cast<float>(
      node.get_parameter("segment_gap_multiplier").as_double());
  cfg.lidar_angle_increment = static_cast<float>(
      node.get_parameter("lidar_angle_increment").as_double());
  cfg.line_fit_tolerance =
      static_cast<float>(node.get_parameter("line_fit_tolerance").as_double());
  cfg.error_segment_tracker_cell_size = static_cast<float>(
      node.get_parameter("error_segment_tracker_cell_size").as_double());
  cfg.error_segment_tracker_max_drift = static_cast<float>(
      node.get_parameter("error_segment_tracker_max_drift").as_double());
  cfg.min_segment_points =
      static_cast<int>(node.get_parameter("min_segment_points").as_int());
  cfg.enable_error_segment_tracking =
      node.get_parameter("enable_error_segment_tracking").as_bool();
  cfg.report_error_segments_as_points =
      node.get_parameter("report_error_segments_as_points").as_bool();
  cfg.outlier_filter_distance = static_cast<float>(
      node.get_parameter("outlier_filter_distance").as_double());

  cfg.enable_height_awareness =
      node.get_parameter("enable_height_awareness").as_bool();
  cfg.lidar_height_source =
      node.get_parameter("lidar_height_source").as_string();
  cfg.lidar_height_fixed = node.get_parameter("lidar_height_fixed").as_double();
  cfg.lidar_frame = node.get_parameter("lidar_frame").as_string();
  cfg.height_match_tolerance = static_cast<float>(
      node.get_parameter("height_match_tolerance").as_double());
  cfg.height_match_mode = node.get_parameter("height_match_mode").as_string();

  return cfg;
}

std::vector<std::string> GeofenceConfig::validate() const {
  std::vector<std::string> errors = GeofenceCommonConfig::validate();
  if (!(this->correspondence_tolerance > 0.0f)) {
    errors.push_back("correspondence_tolerance must be > 0");
  }
  if (this->min_segment_points < 1) {
    errors.push_back("min_segment_points must be >= 1");
  }
  return errors;
}

} // namespace rises
