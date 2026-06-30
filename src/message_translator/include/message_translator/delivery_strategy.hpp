#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include <vector>
#include <memory>

namespace rises {

/**
 * @brief Strategy interface for delivering map updates
 * 
 * Implements Strategy pattern for service vs topic delivery
 */
class DeliveryStrategy {
public:
    virtual ~DeliveryStrategy() = default;

    /**
     * @brief Deliver updates to geofence nodes
     * @return true if delivery successful for all AGVs
     */
    [[nodiscard]] virtual bool deliver(
        const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
        const builtin_interfaces::msg::Time& timestamp) = 0;

    /**
     * @brief Check if delivery method is ready
     */
    [[nodiscard]] virtual bool isReady() const = 0;
};

/**
 * @brief Service-based delivery strategy
 */
class ServiceDeliveryStrategy : public DeliveryStrategy {
public:
    ServiceDeliveryStrategy(
        rclcpp::Node* node,
        int agv_count,
        const std::vector<rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr>& clients,
        int chunk_size = 1024,
        double service_timeout = 300.0);

    bool deliver(
        const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
        const builtin_interfaces::msg::Time& timestamp) override;

    bool isReady() const override;

private:
    bool deliverChunk(
        const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
        const builtin_interfaces::msg::Time& timestamp,
        std::size_t chunk_index, std::size_t total_chunks);

    rclcpp::Node* node_;
    int agv_count_;
    int chunk_size_;
    double service_timeout_;
    std::vector<rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr> clients_;
};

/**
 * @brief Topic-based delivery strategy
 */
class TopicDeliveryStrategy : public DeliveryStrategy {
public:
    TopicDeliveryStrategy(
        rclcpp::Node* node,
        rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr publisher);

    bool deliver(
        const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
        const builtin_interfaces::msg::Time& timestamp) override;

    bool isReady() const override;

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr publisher_;
};

} // namespace rises
