#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <functional>
#include <memory>
#include <array>

// Build-time laser count configuration (must be provided via CMake)
#ifndef LASER_COUNT
#error "LASER_COUNT must be defined at compile time via -DLASER_COUNT=N"
#endif

// Convert macro to constexpr for template usage
namespace {
    constexpr std::size_t LASER_COUNT_VALUE = LASER_COUNT;
}

namespace rises::sync {

// Helper to extract first N elements from tuple (C++17 compatible)
template<std::size_t N, typename Tuple, std::size_t... Is>
void extractFirstN(std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>& vec, 
                   Tuple&& tuple, std::index_sequence<Is...>) {
    (vec.push_back(std::get<Is>(std::forward<Tuple>(tuple))), ...);
}

// Variadic template metaprogramming to generate sync policy with exact N LaserScan types
// Only used when N > 1 (message_filters requires at least 2 message types)
template<std::size_t N, typename Indices = std::make_index_sequence<N>>
struct SyncPolicyGenerator;

template<std::size_t N, std::size_t... Is>
struct SyncPolicyGenerator<N, std::index_sequence<Is...>> {
    template<std::size_t> using scan_t = sensor_msgs::msg::LaserScan;
    using type = message_filters::sync_policies::ApproximateTime<scan_t<Is>...>;
};

template<std::size_t N>
using SyncPolicy = typename SyncPolicyGenerator<N>::type;

/**
 * @brief Template-based laser synchronizer using message_filters
 * @tparam N Number of laser scanners to synchronize
 * @tparam NodeType ROS2 node type (e.g., rclcpp::Node, rclcpp_lifecycle::LifecycleNode)
 */
template<std::size_t N, typename NodeType = rclcpp::Node>
class LaserSynchronizer {
public:
    using ScanCallback = std::function<void(const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr>&)>;
    
    explicit LaserSynchronizer(std::shared_ptr<NodeType> node, std::size_t queue_size = 10, double slop = 0.1) 
        : node_(node), queue_size_(queue_size), slop_(slop) {}
    
    /**
     * @brief Setup synchronizer with topic names
     * @param topics Array of topic names for each laser
     * @param callback Callback function for synchronized scans
     */
    void setup(const std::array<std::string, N>& topics, ScanCallback callback);
    
    /**
     * @brief Check if synchronizer is ready
     */
    bool isReady() const { return sync_ != nullptr; }
    
    /**
     * @brief Get number of configured lasers
     */
    constexpr std::size_t getLaserCount() const { return N; }

private:
    std::shared_ptr<NodeType> node_;  // Node pointer
    std::size_t queue_size_;
    double slop_;  // Time slop for approximate time sync (seconds)
    
    // message_filters subscribers with explicit NodeType
    std::array<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, NodeType>>, N> subscribers_;
    
    // Type-erased synchronizer storage
    std::shared_ptr<void> sync_;
    
    /**
     * @brief Create synchronizer with parameter pack expansion
     * @param callback User callback function
     */
    template<std::size_t... Is>
    void createSynchronizer(ScanCallback callback, std::index_sequence<Is...>);
};

// Template implementation (must be in header)
template<std::size_t N, typename NodeType>
void LaserSynchronizer<N, NodeType>::setup(
    const std::array<std::string, N>& topics, 
    ScanCallback callback) {
    
    if constexpr (N == 1) {
        // Single laser: use regular subscriber (no synchronization needed)
        auto sub = node_->template create_subscription<sensor_msgs::msg::LaserScan>(
            topics[0], queue_size_,
            [callback](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> scan_vector;
                scan_vector.push_back(msg);
                callback(scan_vector);
            });
        sync_ = sub; // Store as type-erased
    } else {
        // Multiple lasers: use message_filters synchronization
        // Create message_filters subscribers with NodeType template parameter
        for (std::size_t i = 0; i < N; ++i) {
            subscribers_[i] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan, NodeType>>(
                node_, topics[i]);
        }
        
        // Create synchronizer with parameter pack expansion
        createSynchronizer(callback, std::make_index_sequence<N>{});
    }
}

template<std::size_t N, typename NodeType>
template<std::size_t... Is>
void LaserSynchronizer<N, NodeType>::createSynchronizer(
    ScanCallback callback, 
    std::index_sequence<Is...>) {
    
    // This should only be called when N > 1
    static_assert(N > 1, "createSynchronizer should only be called when N > 1");
    
    // Use template metaprogramming to create sync policy with exact N types
    using Policy = SyncPolicy<N>;
    auto sync = std::make_shared<message_filters::Synchronizer<Policy>>(queue_size_);
    
    // Set time slop for approximate time synchronization
    sync->setAgePenalty(slop_);
    
    // Connect all subscribers using parameter pack expansion
    sync->connectInput(*subscribers_[Is]...);
    
    // Register callback that receives all synchronized scans
    // Signal9 may pass NullType for unused slots, so we extract only first N
    sync->registerCallback(
        [callback](auto&&... scans) {
            std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> scan_vector;
            scan_vector.reserve(N);
            
            // Extract only the first N elements from tuple using helper function
            auto tuple = std::forward_as_tuple(scans...);
            extractFirstN<N>(scan_vector, tuple, std::make_index_sequence<N>{});
            
            callback(scan_vector);
        });
    
    // Store sync object (type-erased)
    sync_ = sync;
}

} // namespace rises::sync

// Use LASER_COUNT build option to instantiate the template with LifecycleNode
using LaserSynchronizerInstance = rises::sync::LaserSynchronizer<LASER_COUNT_VALUE, rclcpp_lifecycle::LifecycleNode>;