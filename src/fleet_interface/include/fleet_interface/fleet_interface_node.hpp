#pragma once

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fleet_interface/vda5050_state_bridge.hpp"

#include <memory>
#include <string>

namespace rises {

/**
 * Per-AGV fleet interface node.
 *
 * Responsibilities:
 *  - Subscribe to VDA5050 orders (JSON string) and convert to nav_msgs/Path
 *  - Bridge geofence obstacle reports to VDA5050 state JSON via
 * Vda5050StateBridge
 *  - Publish robot position + obstacle state on ROS and MQTT topics
 *
 * Runs one instance per AGV namespace (e.g. /agv_0/fleet_interface_node).
 */
class FleetInterfaceNode : public rclcpp::Node {
public:
  explicit FleetInterfaceNode(const rclcpp::NodeOptions &options);

  // Read-only accessors used by tests and downstream tooling that need to
  // confirm parameter wiring without scraping log lines. Members are still
  // private; production callers should rely on parameter declarations.
  const std::string &getGlobalFrame() const noexcept { return global_frame_; }
  const std::string &getTargetFrame() const noexcept { return target_frame_; }
  const std::string &getTfPrefix() const noexcept { return tf_prefix_; }

private:
  void orderCallback(const std_msgs::msg::String::SharedPtr msg);

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // VDA5050 order → Path
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Frame config
  std::string global_frame_;
  std::string target_frame_;
  std::string tf_prefix_;

  // VDA5050 state bridge (obstacle reports + position → JSON)
  std::unique_ptr<Vda5050StateBridge> state_bridge_;
};

} // namespace rises
