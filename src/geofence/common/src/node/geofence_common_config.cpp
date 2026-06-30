#include "geofence/common/node/geofence_common_config.hpp"

#include <sstream>
#include <stdexcept>

namespace rises {

namespace {
constexpr std::size_t kTransformMatrixSize = 16;

void requireValidMatrix(const std::vector<double> &matrix,
                        const std::string &param_name) {
  if (matrix.size() != kTransformMatrixSize) {
    std::ostringstream msg;
    msg << "Parameter '" << param_name << "' must be a 16-element "
        << "row-major 4x4 matrix (got " << matrix.size() << " elements)";
    throw std::invalid_argument(msg.str());
  }
}

// Get a matrix parameter that may be either a double array (native YAML) or a
// string (from $(env ...) substitution). Parses "[0.0, -1.0, ...]" strings.
std::vector<double> getMatrixParam(rclcpp_lifecycle::LifecycleNode &node,
                                   const std::string &name) {
  const rclcpp::Parameter &param = node.get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    return param.as_double_array();
  }
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    std::string s = param.as_string();
    std::string::size_type start = s.find_first_of("-0123456789.");
    std::string::size_type end = s.find_last_of("0123456789.");
    if (start == std::string::npos)
      return {};
    s = s.substr(start, end - start + 1);
    std::vector<double> result;
    std::istringstream iss(s);
    std::string token;
    while (std::getline(iss, token, ',')) {
      try {
        result.push_back(std::stod(token));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node.get_logger(),
                     "[getMatrixParam] Failed to parse '%s' in %s: %s",
                     token.c_str(), name.c_str(), e.what());
        return {};
      }
    }
    RCLCPP_INFO(node.get_logger(), "Parsed %s from string (%zu values)",
                name.c_str(), result.size());
    return result;
  }
  RCLCPP_WARN(node.get_logger(),
              "Parameter %s has unexpected type %d, using default", name.c_str(),
              static_cast<int>(param.get_type()));
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}
} // namespace

void GeofenceCommonConfig::declareAndRead(rclcpp_lifecycle::LifecycleNode &node,
                                          GeofenceCommonConfig &cfg,
                                          bool publish_report_always_default) {
  // Safety / visualisation
  node.declare_parameter<bool>("enable_safety_circle", true);
  node.declare_parameter<double>("safety_circle_radius", 0.50);
  node.declare_parameter<bool>("visualizer_enabled", true);

  // TF / frames
  node.declare_parameter<bool>("use_namespace_for_map_frame", false);
  node.declare_parameter<std::string>("target_frame", "map");
  node.declare_parameter<std::string>("tf_prefix", "");
  node.declare_parameter<std::string>("base_link_frame", "base_link");

  // Report framing (publish_report_always default is node-specific)
  node.declare_parameter<bool>("publish_report_always",
                               publish_report_always_default);
  node.declare_parameter<bool>("publish_report_in_local_frame", false);
  node.declare_parameter<std::string>("report_output_frame", "");

  // Multi-robot filtering
  node.declare_parameter<bool>("enable_robot_filtering", false);
  node.declare_parameter<std::vector<std::string>>(
      "robot_ids", std::vector<std::string>{"self"});
  node.declare_parameter<std::string>("default_robot_footprint_type", "circle");
  node.declare_parameter<double>("default_robot_footprint_radius", 0.5);
  node.declare_parameter<double>("default_robot_footprint_width", 0.6);
  node.declare_parameter<double>("default_robot_footprint_height", 0.8);
  node.declare_parameter<std::vector<double>>("default_robot_footprint_polygon",
                                              std::vector<double>{});
  node.declare_parameter<double>("default_robot_footprint_margin", 0.1);
  node.declare_parameter<int64_t>("robot_pose_max_age_ms", 2000);
  node.declare_parameter<std::string>("robot_pose_topic_prefix", "robot_pose");

  // Pre-initialisation
  node.declare_parameter<std::string>("obstacles_json_file", "");
  node.declare_parameter<std::string>("contours_json_file", "");
  node.declare_parameter<bool>("publish_ready_signal", false);

  // Coordinate transforms
  rcl_interfaces::msg::ParameterDescriptor matrix_desc;
  matrix_desc.dynamic_typing = true;
  const std::vector<double> identity4x4{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  node.declare_parameter<bool>("transform_obstacles_enabled", false);
  node.declare_parameter("obstacle_transform_matrix",
                         rclcpp::ParameterValue(identity4x4), matrix_desc);
  node.declare_parameter<bool>("transform_boundaries_enabled", false);
  node.declare_parameter("boundary_transform_matrix",
                         rclcpp::ParameterValue(identity4x4), matrix_desc);
  node.declare_parameter<bool>("transform_areas_enabled", false);
  node.declare_parameter("area_transform_matrix",
                         rclcpp::ParameterValue(identity4x4), matrix_desc);
  node.declare_parameter<bool>("negate_y_in_ros", false);

  // Auto-activate
  node.declare_parameter<bool>("auto_activate", false);

  // --- Read all values ---
  cfg.enable_safety_circle = node.get_parameter("enable_safety_circle").as_bool();
  cfg.safety_circle_outer_radius =
      static_cast<float>(node.get_parameter("safety_circle_radius").as_double());
  cfg.visualizer_enabled = node.get_parameter("visualizer_enabled").as_bool();

  cfg.use_namespace_for_map_frame =
      node.get_parameter("use_namespace_for_map_frame").as_bool();
  cfg.target_frame = node.get_parameter("target_frame").as_string();
  cfg.tf_prefix = node.get_parameter("tf_prefix").as_string();
  cfg.base_link_frame = node.get_parameter("base_link_frame").as_string();

  cfg.publish_report_always =
      node.get_parameter("publish_report_always").as_bool();
  cfg.publish_report_in_local_frame =
      node.get_parameter("publish_report_in_local_frame").as_bool();
  cfg.report_output_frame = node.get_parameter("report_output_frame").as_string();

  cfg.enable_robot_filtering =
      node.get_parameter("enable_robot_filtering").as_bool();
  cfg.robot_ids = node.get_parameter("robot_ids").as_string_array();
  cfg.default_robot_footprint_type =
      node.get_parameter("default_robot_footprint_type").as_string();
  cfg.default_robot_footprint_radius =
      node.get_parameter("default_robot_footprint_radius").as_double();
  cfg.default_robot_footprint_width =
      node.get_parameter("default_robot_footprint_width").as_double();
  cfg.default_robot_footprint_height =
      node.get_parameter("default_robot_footprint_height").as_double();
  cfg.default_robot_footprint_polygon =
      node.get_parameter("default_robot_footprint_polygon").as_double_array();
  cfg.default_robot_footprint_margin =
      node.get_parameter("default_robot_footprint_margin").as_double();
  cfg.robot_pose_max_age_ms =
      node.get_parameter("robot_pose_max_age_ms").as_int();
  cfg.robot_pose_topic_prefix =
      node.get_parameter("robot_pose_topic_prefix").as_string();

  cfg.obstacles_json_file = node.get_parameter("obstacles_json_file").as_string();
  cfg.contours_json_file = node.get_parameter("contours_json_file").as_string();
  cfg.publish_ready_signal =
      node.get_parameter("publish_ready_signal").as_bool();

  cfg.transform_obstacles_enabled =
      node.get_parameter("transform_obstacles_enabled").as_bool();
  cfg.obstacle_transform_matrix =
      getMatrixParam(node, "obstacle_transform_matrix");
  cfg.transform_boundaries_enabled =
      node.get_parameter("transform_boundaries_enabled").as_bool();
  cfg.boundary_transform_matrix =
      getMatrixParam(node, "boundary_transform_matrix");
  cfg.transform_areas_enabled =
      node.get_parameter("transform_areas_enabled").as_bool();
  cfg.area_transform_matrix = getMatrixParam(node, "area_transform_matrix");
  cfg.negate_y_in_ros = node.get_parameter("negate_y_in_ros").as_bool();

  cfg.auto_activate = node.get_parameter("auto_activate").as_bool();
}

void GeofenceCommonConfig::applyCoordinateTransform(rclcpp::Logger logger) const {
  rises::geofence::CoordinateTransform::Config transform_config;
  transform_config.negate_y = this->negate_y_in_ros;

  if (this->transform_obstacles_enabled) {
    requireValidMatrix(this->obstacle_transform_matrix,
                       "obstacle_transform_matrix");
    transform_config.obstacle = {true, this->obstacle_transform_matrix};
  }

  if (this->transform_boundaries_enabled) {
    requireValidMatrix(this->boundary_transform_matrix,
                       "boundary_transform_matrix");
    transform_config.boundary = {true, this->boundary_transform_matrix};
  }

  if (this->transform_areas_enabled) {
    requireValidMatrix(this->area_transform_matrix, "area_transform_matrix");
    transform_config.area = {true, this->area_transform_matrix};
  }

  rises::geofence::CoordinateTransform::initialize(transform_config);

  if (rises::geofence::CoordinateTransform::isObstacleTransformEnabled()) {
    const auto &m = this->obstacle_transform_matrix;
    RCLCPP_INFO(logger,
                "Obstacle coordinate transformation enabled (matrix: "
                "[%.1f,%.1f,%.1f,%.1f, %.1f,%.1f,%.1f,%.1f, "
                "%.1f,%.1f,%.1f,%.1f, %.1f,%.1f,%.1f,%.1f])",
                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9],
                m[10], m[11], m[12], m[13], m[14], m[15]);
  }
  if (rises::geofence::CoordinateTransform::isBoundaryTransformEnabled()) {
    const auto &m = this->boundary_transform_matrix;
    RCLCPP_INFO(logger,
                "Boundary coordinate transformation enabled (matrix: "
                "[%.1f,%.1f,%.1f,%.1f, %.1f,%.1f,%.1f,%.1f, "
                "%.1f,%.1f,%.1f,%.1f, %.1f,%.1f,%.1f,%.1f])",
                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9],
                m[10], m[11], m[12], m[13], m[14], m[15]);
  }
  if (rises::geofence::CoordinateTransform::isAreaTransformEnabled()) {
    RCLCPP_INFO(logger, "Area coordinate transformation enabled");
  }
  if (this->negate_y_in_ros) {
    RCLCPP_INFO(logger,
                "Y-axis negation enabled in ROS space (forward/backward flip)");
  }
}

std::vector<std::string> GeofenceCommonConfig::validate() const {
  std::vector<std::string> errors;
  if (this->enable_safety_circle && !(this->safety_circle_outer_radius > 0.0f)) {
    errors.push_back(
        "safety_circle_outer_radius must be > 0 when enable_safety_circle=true");
  }
  if (this->target_frame.empty()) {
    errors.push_back("target_frame must not be empty");
  }
  if (this->base_link_frame.empty()) {
    errors.push_back("base_link_frame must not be empty");
  }
  if (this->enable_robot_filtering &&
      this->default_robot_footprint_type == "circle" &&
      !(this->default_robot_footprint_radius > 0.0)) {
    errors.push_back("default_robot_footprint_radius must be > 0 for a circle "
                     "footprint when enable_robot_filtering=true");
  }
  return errors;
}

} // namespace rises
