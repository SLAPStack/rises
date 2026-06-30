/// @file main.cpp
/// @brief Entry point for the ARISE mission controller node.

#include "rises_mission_controller/mission_controller_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<rises::MissionControllerNode>(options);

    // MultiThreadedExecutor required: action client callbacks run on separate threads
    // from the intent subscription callback to avoid deadlock.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
