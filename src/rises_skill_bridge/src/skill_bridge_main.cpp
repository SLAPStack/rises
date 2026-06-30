#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rises_skill_bridge/skill_bridge_node.hpp"

// SkillBridgeNode runs each skill action handler inline and then polls the geofence
// service future WITHOUT spinning (see SkillBridgeNode::callService), relying on a
// SECOND executor thread to deliver the service response. It therefore MUST run on a
// MultiThreadedExecutor. The single-threaded executable that
// rclcpp_components_register_node(EXECUTABLE) generates deadlocks here: the inline
// action handler blocks the only executor thread, so the service-client response is
// never processed and every geofence call times out. This explicit main runs the node
// on a MultiThreadedExecutor as the node's design requires.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rises::SkillBridgeNode> node =
      std::make_shared<rises::SkillBridgeNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
