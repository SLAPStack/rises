/// @file main.cpp
/// @brief Entry point for the ARISE leg filter node.

#include "rises_leg_filter/leg_filter_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<rises::LegFilterNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
