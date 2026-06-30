#include "geofence/gridmap/node/gridmap_config.hpp"

namespace rises {

GridmapConfig GridmapConfig::fromNode(rclcpp_lifecycle::LifecycleNode &node) {
  GridmapConfig cfg;

  // Shared parameters. publish_report_always defaults FALSE on the gridmap node
  // (the spatial node defaults it true); preserve that here.
  GeofenceCommonConfig::declareAndRead(node, cfg,
                                       /*publish_report_always_default=*/false);

  // Occupancy-grid parameters.
  node.declare_parameter<double>("grid_resolution", 0.05);
  node.declare_parameter<double>("grid_width_meters", 100.0);
  node.declare_parameter<double>("grid_height_meters", 100.0);
  node.declare_parameter<double>("grid_origin_x", -50.0);
  node.declare_parameter<double>("grid_origin_y", -50.0);
  node.declare_parameter<double>("obstacle_inflation_radius", 0.0);

  // Latency recording (empty string = disabled). Read by the node directly.
  node.declare_parameter<std::string>("latency_output_file", "");

  cfg.grid_resolution = node.get_parameter("grid_resolution").as_double();
  cfg.grid_width_meters = node.get_parameter("grid_width_meters").as_double();
  cfg.grid_height_meters = node.get_parameter("grid_height_meters").as_double();
  cfg.grid_origin_x = node.get_parameter("grid_origin_x").as_double();
  cfg.grid_origin_y = node.get_parameter("grid_origin_y").as_double();
  cfg.obstacle_inflation_radius = static_cast<float>(
      node.get_parameter("obstacle_inflation_radius").as_double());

  return cfg;
}

} // namespace rises
