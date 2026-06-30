/// @file main.cpp
/// @brief Entry point for the obstacle aggregator node.

#include "obstacle_aggregator/obstacle_aggregator_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rises::ObstacleAggregatorNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
