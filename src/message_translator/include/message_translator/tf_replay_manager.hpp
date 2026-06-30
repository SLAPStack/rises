#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <rises_interfaces/msg/obstacle_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <mutex>
#include <memory>

namespace rises {

/**
 * @brief Manages buffering and replay of all messages after geofence initialization
 * 
 * Buffers TF, LaserScan, and validation messages during initialization,
 * then replays them at a configurable rate after geofence nodes confirm map updates.
 */
class TFReplayManager {
public:
    struct TimestampedTF {
        rclcpp::Time timestamp;
        tf2_msgs::msg::TFMessage message;
    };

    struct TimestampedLaserScan {
        rclcpp::Time timestamp;
        sensor_msgs::msg::LaserScan message;
        std::string topic_name;
    };

    struct TimestampedValidation {
        rclcpp::Time timestamp;
        std_msgs::msg::String message;
    };

    explicit TFReplayManager(
        rclcpp::Node* node,
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster);

    /**
     * @brief Buffer a TF message for later replay
     */
    void bufferMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    /**
     * @brief Buffer a LaserScan message for later replay
     */
    void bufferLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, 
                         const std::string& topic_name);

    /**
     * @brief Buffer a validation message for later replay
     */
    void bufferValidation(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Set laserscan publisher for replay
     */
    void setLaserScanPublisher(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub) {
        laserscan_pub_ = pub;
    }

    /**
     * @brief Set validation publisher for replay
     */
    void setValidationPublisher(rclcpp::Publisher<rises_interfaces::msg::ObstacleArray>::SharedPtr pub) {
        validation_pub_ = pub;
    }

    /**
     * @brief Immediately relay a TF message (no buffering)
     */
    void relayMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    /**
     * @brief Start replaying all buffered messages at configured rate
     */
    void startReplay(double replay_rate);

    /**
     * @brief Check if currently replaying
     */
    bool isReplaying() const { return is_replaying_; }

    /**
     * @brief Get number of buffered messages
     */
    std::size_t getBufferedCount() const;

    /**
     * @brief Clear all buffered messages
     */
    void clearBuffer();

private:
    void replayTimerCallback();
    void publishTransform(const geometry_msgs::msg::TransformStamped& transform);
    void publishLaserScan(const TimestampedLaserScan& scan);
    void publishValidation(const TimestampedValidation& validation);

    rclcpp::Node* node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    rclcpp::Publisher<rises_interfaces::msg::ObstacleArray>::SharedPtr validation_pub_;

    std::vector<TimestampedTF> buffered_tf_;
    std::vector<TimestampedLaserScan> buffered_laserscans_;
    std::vector<TimestampedValidation> buffered_validations_;
    
    std::size_t replay_tf_index_;
    std::size_t replay_scan_index_;
    std::size_t replay_validation_index_;
    
    rclcpp::Time replay_start_time_;
    rclcpp::Time replay_base_time_;
    double replay_rate_;
    bool is_replaying_;

    rclcpp::TimerBase::SharedPtr replay_timer_;
    mutable std::mutex replay_mutex_;
};

} // namespace rises
