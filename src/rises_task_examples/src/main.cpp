/// @file main.cpp
/// @brief Entry point for the ARISE task examples node.
///
/// Uses MultiThreadedExecutor because the node creates action clients
/// in a separate callback group that must be serviced concurrently
/// with the Trigger service callbacks.

#include "rises_task_examples/task_examples_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rises::TaskExamplesNode> node =
        std::make_shared<rises::TaskExamplesNode>(rclcpp::NodeOptions());

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
