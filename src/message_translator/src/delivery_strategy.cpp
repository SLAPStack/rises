#include "message_translator/delivery_strategy.hpp"
#include <algorithm>

namespace rises {

// ============================================================================
// ServiceDeliveryStrategy: Delivers map updates via ROS2 service calls
// Pros: Acknowledgment, error handling, guaranteed delivery per AGV
// Cons: Higher latency, requires service availability checks
// Use when: Reliability and per-AGV confirmation required
// ============================================================================

// Constructor: Initialize service delivery with AGV client list
ServiceDeliveryStrategy::ServiceDeliveryStrategy(
    rclcpp::Node* node,
    const int agv_count,
    const std::vector<rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr>& clients,
    const int chunk_size,
    const double service_timeout)
: node_(node)
, agv_count_(agv_count)
, chunk_size_(chunk_size)
, service_timeout_(service_timeout)
, clients_(clients)
{
    RCLCPP_INFO(node_->get_logger(),
        "[SERVICE] Configured: chunk_size=%d, timeout=%.0fs", chunk_size_, service_timeout_);
}

// Deliver a single chunk of obstacle updates to all AGVs via service calls
bool ServiceDeliveryStrategy::deliverChunk(
    const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
    const builtin_interfaces::msg::Time& timestamp,
    const std::size_t chunk_index, const std::size_t total_chunks)
{
    const std::shared_ptr<rises_interfaces::srv::UpdateMap::Request> request =
        std::make_shared<rises_interfaces::srv::UpdateMap::Request>();
    request->updates.header.stamp = timestamp;
    request->updates.header.frame_id = "map";
    request->updates.updates = updates;

    // Parallel service calls: launch all async requests before waiting for responses
    std::vector<std::shared_future<std::shared_ptr<rises_interfaces::srv::UpdateMap::Response>>> futures;
    std::vector<bool> service_available(this->agv_count_, false);

    for (int i = 0; i < this->agv_count_; i++) {
        if (!this->clients_[i]->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->node_->get_logger(),
                "[SERVICE] AGV %d: update_map service not available (chunk %zu/%zu)", i, chunk_index + 1, total_chunks);
            continue;
        }

        service_available[i] = true;
        std::shared_future<std::shared_ptr<rises_interfaces::srv::UpdateMap::Response>> future =
            this->clients_[i]->async_send_request(request).share();
        futures.push_back(future);
    }

    const bool all_available = std::all_of(service_available.begin(), service_available.end(),
                                     [](const bool v) { return v; });
    if (!all_available) {
        RCLCPP_ERROR(this->node_->get_logger(), "[SERVICE] Some AGV services unavailable (chunk %zu/%zu)",
            chunk_index + 1, total_chunks);
        return false;
    }

    const auto timeout = std::chrono::duration<double>(this->service_timeout_);
    bool all_success = true;
    int obstacles_added_total = 0;
    int obstacles_removed_total = 0;

    for (std::size_t i = 0; i < futures.size(); i++) {
        const std::future_status status = futures[i].wait_for(timeout);

        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->node_->get_logger(),
                "[SERVICE] AGV %zu: timeout after %.0fs (chunk %zu/%zu, %zu updates)",
                i, this->service_timeout_, chunk_index + 1, total_chunks, updates.size());
            all_success = false;
            continue;
        }

        const std::shared_ptr<rises_interfaces::srv::UpdateMap::Response> response = futures[i].get();

        if (!response->success) {
            RCLCPP_ERROR(this->node_->get_logger(),
                "[SERVICE] AGV %zu: rejected updates (chunk %zu/%zu): %s",
                i, chunk_index + 1, total_chunks, response->message.c_str());
            all_success = false;
        } else {
            obstacles_added_total += response->obstacles_added;
            obstacles_removed_total += response->obstacles_removed;
        }
    }

    if (all_success) {
        RCLCPP_INFO(this->node_->get_logger(),
            "[SERVICE] Chunk %zu/%zu delivered: %zu updates, added=%d, removed=%d",
            chunk_index + 1, total_chunks, updates.size(), obstacles_added_total, obstacles_removed_total);
    }

    return all_success;
}

// Deliver obstacle updates to all AGVs, splitting into chunks if needed
bool ServiceDeliveryStrategy::deliver(
    const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
    const builtin_interfaces::msg::Time& timestamp)
{
    const std::size_t count = updates.size();

    // No chunking needed if updates fit in a single chunk
    if (this->chunk_size_ <= 0 || count <= static_cast<std::size_t>(this->chunk_size_)) {
        return this->deliverChunk(updates, timestamp, 0, 1);
    }

    // Split into chunks
    const std::size_t total_chunks = (count + this->chunk_size_ - 1) / this->chunk_size_;
    RCLCPP_INFO(this->node_->get_logger(),
        "[SERVICE] Splitting %zu updates into %zu chunks of %d",
        count, total_chunks, this->chunk_size_);

    bool all_success = true;
    for (std::size_t c = 0; c < total_chunks; c++) {
        const std::size_t start = c * this->chunk_size_;
        const std::size_t end = std::min(start + static_cast<std::size_t>(this->chunk_size_), count);

        std::vector<rises_interfaces::msg::ObstacleUpdate> chunk(
            updates.begin() + static_cast<long>(start),
            updates.begin() + static_cast<long>(end));

        if (!this->deliverChunk(chunk, timestamp, c, total_chunks)) {
            all_success = false;
            RCLCPP_WARN(this->node_->get_logger(),
                "[SERVICE] Chunk %zu/%zu failed, continuing with remaining chunks", c + 1, total_chunks);
        }
    }

    RCLCPP_INFO(this->node_->get_logger(),
        "[SERVICE] Chunked delivery complete: %zu updates in %zu chunks, success=%s",
        count, total_chunks, all_success ? "true" : "false");

    return all_success;
}

// Check if all service clients are ready
bool ServiceDeliveryStrategy::isReady() const
{
    for (const rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr& client : this->clients_) {
        if (!client->service_is_ready()) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// TopicDeliveryStrategy: Delivers map updates via ROS2 topic publishing
// Pros: Low latency, broadcast to all subscribers, no blocking
// Cons: No acknowledgment, no per-AGV error handling, fire-and-forget
// Use when: Real-time performance critical, best-effort delivery acceptable
// ============================================================================

// Constructor: Initialize topic delivery with publisher
TopicDeliveryStrategy::TopicDeliveryStrategy(
    rclcpp::Node* node,
    rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr publisher)
: node_(node)
, publisher_(publisher)
{
}

// Publish obstacle updates to topic for broadcast delivery
bool TopicDeliveryStrategy::deliver(
    const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
    const builtin_interfaces::msg::Time& timestamp)
{
    std::unique_ptr<rises_interfaces::msg::ObstacleUpdateArray> msg =
        std::make_unique<rises_interfaces::msg::ObstacleUpdateArray>();
    msg->header.stamp = timestamp;
    msg->header.frame_id = "map";
    msg->updates = updates;
    
    this->publisher_->publish(std::move(msg));
    
    RCLCPP_INFO(this->node_->get_logger(),
        "[TOPIC] Published %zu updates", updates.size());
    
    return true;
}

// Check if publisher has active subscribers
bool TopicDeliveryStrategy::isReady() const
{
    return this->publisher_->get_subscription_count() > 0;
}

} // namespace rises
