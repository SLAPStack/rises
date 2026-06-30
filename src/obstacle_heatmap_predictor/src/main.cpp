/// @file main.cpp
/// @brief Entry point for the obstacle heatmap predictor node.

#include "obstacle_heatmap_predictor/heatmap_predictor_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rises::HeatmapPredictorNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
