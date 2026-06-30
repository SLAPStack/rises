#include "fleet_interface/fleet_interface_node.hpp"
#include "fleet_interface/vda5050_converter.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace rises {

FleetInterfaceNode::FleetInterfaceNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fleet_interface_node", options) {
  // TF setup (must come before state bridge)
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  // Frame configuration. Parameter names match member semantics: the global
  // frame is the world-fixed reference (typically "map"), the target frame is
  // the robot-attached frame (typically "base_link").
  this->global_frame_ =
      this->declare_parameter<std::string>("global_frame", "map");
  this->target_frame_ =
      this->declare_parameter<std::string>("target_frame", "base_link");
  this->tf_prefix_ = this->declare_parameter<std::string>("tf_prefix", "");

  if (!this->tf_prefix_.empty() && this->tf_prefix_.back() != '_') {
    this->tf_prefix_ += '_';
  }

  // VDA5050 order subscription → Path publisher
  this->order_sub_ = this->create_subscription<std_msgs::msg::String>(
      "order", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        this->orderCallback(msg);
      });

  this->path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("incoming_path", 10);

  // VDA5050 state bridge: obstacle reports + robot position → JSON state
  {
    Vda5050StateBridge::Config state_config;
    state_config.position_source =
        this->declare_parameter<std::string>("position_source", "tf");
    state_config.state_publish_rate =
        this->declare_parameter<double>("state_publish_rate", 1.0);
    state_config.mqtt_agv_name =
        this->declare_parameter<std::string>("mqtt_agv_name", "");
    state_config.global_frame = this->global_frame_;
    state_config.target_frame = this->target_frame_;
    state_config.tf_prefix = this->tf_prefix_;
    this->state_bridge_ = std::make_unique<Vda5050StateBridge>(
        this, state_config, this->tf_buffer_);
  }

  RCLCPP_INFO(this->get_logger(),
              "[INIT] Fleet interface node started in namespace '%s' "
              "(global_frame=%s, target_frame=%s, tf_prefix='%s')",
              this->get_namespace(), this->global_frame_.c_str(),
              this->target_frame_.c_str(), this->tf_prefix_.c_str());
}

void FleetInterfaceNode::orderCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (!msg || msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty VDA5050 order message");
    return;
  }

  const nav_msgs::msg::Path path = Vda5050Converter::orderToPath(
      msg->data, this->global_frame_, this->now());

  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "VDA5050 order produced no path waypoints");
    return;
  }

  this->path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(),
              "Published path with %zu waypoints from VDA5050 order",
              path.poses.size());
}

} // namespace rises

RCLCPP_COMPONENTS_REGISTER_NODE(rises::FleetInterfaceNode)
