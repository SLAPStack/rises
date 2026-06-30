#include "fleet_interface/vda5050_converter.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/header.hpp>

#include <cmath>
#include <cstddef>

namespace {

// VDA5050 spec: nodePosition.theta is given in degrees, range [-180..180].
constexpr double DEG_TO_RAD = M_PI / 180.0;

// Hard cap on order.nodes[] length. The JSON is attacker-controllable, and an
// unbounded reserve on the node count exhausts heap before per-node validation
// runs. Orders above this cap are rejected with an empty Path; the same value
// must match message_translator/src/vda5050_converter.cpp.
constexpr std::size_t kMaxVda5050Nodes = 10'000;

struct NodePosition {
  double x;
  double y;
  double theta_rad;
  bool has_theta;
};

bool parseNodePosition(const nlohmann::json &node, NodePosition &out) {
  if (!node.contains("nodePosition"))
    return false;

  const nlohmann::json &pos = node["nodePosition"];
  if (!pos.contains("x") || !pos.contains("y"))
    return false;

  out.x = pos["x"].get<double>();
  out.y = pos["y"].get<double>();
  out.has_theta = pos.contains("theta") && !pos["theta"].is_null();
  out.theta_rad = out.has_theta ? pos["theta"].get<double>() * DEG_TO_RAD : 0.0;

  return true;
}

geometry_msgs::msg::PoseStamped makePose(const NodePosition &node_pos,
                                         const std_msgs::msg::Header &header) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = node_pos.x;
  pose.pose.position.y = node_pos.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(node_pos.theta_rad / 2.0);
  pose.pose.orientation.w = std::cos(node_pos.theta_rad / 2.0);
  return pose;
}

nlohmann::json obstacleSegmentsToInformation(
    const std::vector<rises_interfaces::msg::Obstacle> &obstacles,
    const std::string &info_type, const std::string &info_level) {
  nlohmann::json info;
  info["infoType"] = info_type;
  info["infoDescription"] = "OBSTACLE_INFO";
  info["infoLevel"] = info_level;

  nlohmann::json refs = nlohmann::json::array();
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const rises_interfaces::msg::Obstacle &obs = obstacles[i];

    nlohmann::json ref;
    ref["referenceKey"] = "segment_" + std::to_string(i);

    std::string geom;
    if (obs.type == rises_interfaces::msg::Obstacle::LINE) {
      geom = "LINE";
    } else if (obs.type == rises_interfaces::msg::Obstacle::POINT) {
      geom = "POINT";
    } else if (obs.type == rises_interfaces::msg::Obstacle::POLYGON) {
      geom = "POLYGON";
    } else {
      geom = "UNKNOWN";
    }

    for (const geometry_msgs::msg::Point &v : obs.vertices) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), ";%.4f,%.4f", v.x, v.y);
      geom += buf;
    }

    ref["referenceValue"] = geom;
    refs.push_back(std::move(ref));
  }

  info["infoReferences"] = std::move(refs);
  return info;
}

void addAgvPosition(nlohmann::json &state, const rises::AgvPosition &position) {
  if (!position.valid)
    return;
  state["agvPosition"] = {{"x", position.x},
                          {"y", position.y},
                          {"theta", position.theta},
                          {"positionInitialized", true},
                          {"mapId", ""}};
}

} // anonymous namespace

namespace rises {

nav_msgs::msg::Path Vda5050Converter::orderToPath(const std::string &json_str,
                                                  const std::string &frame_id,
                                                  const rclcpp::Time &stamp) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;

  nlohmann::json order;
  try {
    order = nlohmann::json::parse(json_str);
  } catch (const nlohmann::json::parse_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("vda5050_converter"),
                 "Failed to parse VDA5050 order JSON at byte %zu: %s", e.byte,
                 e.what());
    return path;
  }

  if (!order.contains("nodes") || !order["nodes"].is_array()) {
    RCLCPP_WARN(rclcpp::get_logger("vda5050_converter"),
                "VDA5050 order is missing the required 'nodes' array");
    return path;
  }

  const nlohmann::json &nodes = order["nodes"];
  if (nodes.size() > kMaxVda5050Nodes) {
    RCLCPP_ERROR(rclcpp::get_logger("vda5050_converter"),
                 "VDA5050 order rejected: nodes.size()=%zu exceeds cap %zu",
                 nodes.size(), kMaxVda5050Nodes);
    path.poses.clear();
    return path;
  }
  path.poses.reserve(nodes.size());

  for (const nlohmann::json &node : nodes) {
    NodePosition node_pos;
    if (!parseNodePosition(node, node_pos))
      continue;
    path.poses.push_back(makePose(node_pos, path.header));
  }

  return path;
}

std::string Vda5050Converter::obstacleReportToStateJson(
    const rises_interfaces::msg::ObstacleReport &report,
    const AgvPosition &position) {
  nlohmann::json state;

  nlohmann::json informations = nlohmann::json::array();

  if (!report.matched_obstacles.empty()) {
    informations.push_back(obstacleSegmentsToInformation(
        report.matched_obstacles, "KNOWN_SEGMENT", "INFO"));
  }

  if (!report.unmatched_obstacles.empty()) {
    informations.push_back(obstacleSegmentsToInformation(
        report.unmatched_obstacles, "UNKNOWN_SEGMENT", "WARNING"));
  }

  state["information"] = std::move(informations);

  const bool field_violation = !report.unmatched_obstacles.empty();
  state["safetyState"] = {{"eStop", "NONE"},
                          {"fieldViolation", field_violation}};

  addAgvPosition(state, position);

  return state.dump();
}

std::string Vda5050Converter::alertStateJson(const AgvPosition &position) {
  nlohmann::json state;
  state["safetyState"] = {{"eStop", "NONE"}, {"fieldViolation", true}};
  state["information"] = nlohmann::json::array();
  addAgvPosition(state, position);
  return state.dump();
}

} // namespace rises
