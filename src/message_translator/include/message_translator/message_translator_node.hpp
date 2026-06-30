#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rises_interfaces/msg/obstacle_array.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/msg/contours.hpp"
#include "rises_interfaces/srv/update_map.hpp"
#include "rises_interfaces/srv/set_area_state.hpp"

#include "message_translator/tf_replay_manager.hpp"
#include "message_translator/delivery_strategy.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <atomic>
#include <set>
#include <unordered_map>

namespace rises { class Vda5050StateBridge; }

namespace slapstack {

/**
 * ROS2 node that bridges VDA5050/MQTT JSON messages to typed ROS2 messages.
 *
 * Handles all fleet-interface concerns: VDA5050 order paths, Rises AABB obstacle
 * updates, warehouse contours, obstacle validation, TF relay, and laser scan relay.
 *
 * All JSON-to-ROS conversion is delegated to dedicated static converter classes
 * (AabbConverter, ContoursConverter, Vda5050Converter). This node is responsible
 * only for ROS lifecycle concerns: subscriptions, publishers, delivery strategy
 * selection, buffer management, and multi-AGV synchronisation.
 */
class MessageTranslatorNode : public rclcpp::Node {
public:
    explicit MessageTranslatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MessageTranslatorNode();

private:
    // === Subscription callbacks ===
    void publishBaseLinkPose();
    void obstacleUpdatesCallback(const std_msgs::msg::String::SharedPtr msg);
    void vda5050OrderCallback(const std_msgs::msg::String::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void contoursCallback(const std_msgs::msg::String::SharedPtr msg);
    void validationCallback(const std_msgs::msg::String::SharedPtr msg);
    void areaLocksCallback(const std_msgs::msg::String::SharedPtr msg);
    void tfStampCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    // === Buffering ===
    void bufferTimerCallback();
    void flushBuffer();
    // Loop executed by flush_thread_; signals when ready then calls flushBuffer().
    void flushThreadLoop();
    void callUpdateMapServiceImmediate(
        const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
        const builtin_interfaces::msg::Time& timestamp);

    // === Configuration ===
    std::string target_frame_;
    std::string map_frame_;
    std::string tf_prefix_;
    std::string global_frame_;

    bool   enable_buffering_;
    double buffer_timeout_sec_;
    // Set to true after the first successful flush; guards against re-buffering.
    // Written by the flush thread, read by executor callbacks — must be atomic.
    std::atomic<bool> buffer_flushed_{false};
    std::atomic<bool> is_flushing_{false};  // Guards against concurrent buffer flushes.
    std::atomic<bool> init_ready_published_{false};  // Ensures initialization_ready is published only once.
    bool   bulk_only_;  // When true, only accept messages containing "pallets" key.
    bool   use_service_for_map_updates_;

    bool   enable_replay_;
    double replay_rate_;

    int  agv_count_;
    bool publish_init_ready_;

    bool        call_completion_service_;
    std::string completion_service_name_;

    bool                     wait_for_geofence_ready_;
    double                   geofence_ready_timeout_;
    std::set<std::string>    ready_agvs_;
    std::mutex               ready_mutex_;
    // Set once when all geofence nodes signal readiness (or on timeout).
    // Written inside ready_mutex_, read without it in hot callbacks — must be atomic.
    std::atomic<bool>        all_geofences_ready_{false};

    // === Composition ===
    std::unique_ptr<rises::TFReplayManager>  tf_replay_manager_;
    std::unique_ptr<rises::DeliveryStrategy> delivery_strategy_;

    // === Threading ===
    std::mutex             last_message_mutex_;
    std::mutex             buffer_mutex_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr buffer_timer_;

    // Dedicated flush thread — prevents blocking the ROS2 executor during
    // service calls to geofence nodes (which can take up to 60 s total).
    std::thread             flush_thread_;
    std::mutex              flush_cv_mutex_;
    std::condition_variable flush_cv_;
    bool                    flush_requested_{false};   // guarded by flush_cv_mutex_
    std::atomic<bool>       shutdown_{false};

    // === TF ===
    std::shared_ptr<tf2_ros::Buffer>                  tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>        tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>     tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // === Buffered state ===
    std::vector<rises_interfaces::msg::ObstacleUpdate> buffered_updates_;
    // Wall clock for buffer silence detection — immune to use_sim_time.
    std::chrono::steady_clock::time_point last_update_wall_time_;

    // === Service delivery tuning ===
    int    service_chunk_size_;
    double service_timeout_;

    // === Publishers ===
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr           base_link_pub_;
    rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<rises_interfaces::msg::ObstacleArray>::SharedPtr      validation_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                       path_pub_;
    rclcpp::Publisher<rises_interfaces::msg::Contours>::SharedPtr           contours_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                       ready_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr               laserscan_pub_;
    // === Subscribers ===
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     obstacle_updates_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     order_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     contours_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     validation_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     area_locks_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr  tf_raw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;

    // === VDA5050 state bridge (obstacle reports, alerts, position, periodic state) ===
    std::unique_ptr<rises::Vda5050StateBridge> state_bridge_;

    // Geofence ready subscribers use TRANSIENT_LOCAL QoS for the late-joiner pattern.
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> geofence_ready_subs_;
    rclcpp::TimerBase::SharedPtr geofence_ready_timer_;

    // === Service clients ===
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr completion_service_client_;
    rclcpp::Client<rises_interfaces::srv::SetAreaState>::SharedPtr area_state_client_;

    // === Multi-AGV ready coordination ===
    void geofenceReadyCallback(const std::string& agv_namespace,
                               const std_msgs::msg::Bool::SharedPtr msg);
    void geofenceReadyTimeoutCallback();
    void checkAllGeofencesReady();

    // === Per-AGV order relay ===
    // Subscribes to raw per-AGV order topics from rosbag and republishes them
    // after a delay, ensuring Unity's TCP subscribers are ready.
    bool relay_orders_;
    double order_relay_delay_;
    struct BufferedOrder {
        int agv_id;
        std_msgs::msg::String::SharedPtr msg;
    };
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> agv_order_subs_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>    agv_order_pubs_;
    std::mutex                  order_relay_mutex_;
    std::vector<BufferedOrder>  buffered_agv_orders_;
    std::atomic<bool>           orders_flushed_{false};
    rclcpp::TimerBase::SharedPtr order_relay_timer_;
    void agvOrderRelayCallback(int agv_id, const std_msgs::msg::String::SharedPtr msg);
    void flushBufferedOrders();
    void orderRelayTimerCallback();
};

} // namespace slapstack
