#include "fleet_interface/vda5050_state_bridge.hpp"
#include "fleet_interface/vda5050_converter.hpp"

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cmath>
#include <chrono>

namespace rises {

Vda5050StateBridge::Vda5050StateBridge(
    rclcpp::Node* node,
    const Config& config,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(node)
    , config_(config)
    , tf_buffer_(std::move(tf_buffer))
{
    this->state_pub_ = node->create_publisher<std_msgs::msg::String>(
        "obstacle_state", 10);

    // Build MQTT topic: /mqtt/agv/<mqtt_agv_name>/obstacle_state
    std::string mqtt_topic = "/mqtt/agv/";
    if (!config_.mqtt_agv_name.empty()) {
        mqtt_topic += config_.mqtt_agv_name + "/";
    }
    mqtt_topic += "obstacle_state";
    this->state_mqtt_pub_ = node->create_publisher<std_msgs::msg::String>(
        mqtt_topic, 10);

    this->report_sub_ = node->create_subscription<rises_interfaces::msg::ObstacleReport>(
        "obstacle_report", 10,
        [this](const rises_interfaces::msg::ObstacleReport::SharedPtr msg) {
            this->obstacleReportCallback(msg);
        });

    this->alert_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "obstacle_alert", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->obstacleAlertCallback(msg);
        });

    if (config_.position_source == "topic") {
        this->pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "robot_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->state_mutex_);
                this->last_pose_ = msg;
            });
        RCLCPP_INFO(node->get_logger(), "[VDA5050 STATE] Robot position source: topic (robot_pose)");
    } else {
        RCLCPP_INFO(node->get_logger(), "[VDA5050 STATE] Robot position source: TF (%s -> %s)",
            config_.global_frame.c_str(),
            (config_.tf_prefix + config_.target_frame).c_str());
    }

    if (config_.state_publish_rate > 0.0) {
        const std::chrono::duration<double> period(1.0 / config_.state_publish_rate);
        this->state_timer_ = node->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { this->stateTimerCallback(); });
        RCLCPP_INFO(node->get_logger(), "[VDA5050 STATE] Periodic publish rate: %.1f Hz",
            config_.state_publish_rate);
    }

    RCLCPP_INFO(node->get_logger(), "[VDA5050 STATE] Bridge initialized (mqtt_topic=%s)",
        mqtt_topic.c_str());
}

void Vda5050StateBridge::publishStateJson(const std::string& json)
{
    std::unique_ptr<std_msgs::msg::String> ros_msg = std::make_unique<std_msgs::msg::String>();
    ros_msg->data = json;
    this->state_pub_->publish(std::move(ros_msg));

    std::unique_ptr<std_msgs::msg::String> mqtt_msg = std::make_unique<std_msgs::msg::String>();
    mqtt_msg->data = json;
    this->state_mqtt_pub_->publish(std::move(mqtt_msg));
}

void Vda5050StateBridge::obstacleReportCallback(
    const rises_interfaces::msg::ObstacleReport::SharedPtr msg)
{
    if (!msg) return;

    {
        std::lock_guard<std::mutex> lock(this->state_mutex_);
        this->last_report_ = msg;
    }

    const AgvPosition pos = this->getRobotPosition();
    const std::string state_json = Vda5050Converter::obstacleReportToStateJson(*msg, pos);
    this->publishStateJson(state_json);

    RCLCPP_DEBUG(this->node_->get_logger(),
        "Published VDA5050 obstacle state: %zu matched, %zu unmatched segments",
        msg->matched_obstacles.size(), msg->unmatched_obstacles.size());
}

void Vda5050StateBridge::obstacleAlertCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg || !msg->data) return;

    const AgvPosition pos = this->getRobotPosition();
    const std::string state_json = Vda5050Converter::alertStateJson(pos);
    this->publishStateJson(state_json);

    RCLCPP_WARN(this->node_->get_logger(),
        "Published immediate VDA5050 alert state (fieldViolation=true)");
}

void Vda5050StateBridge::stateTimerCallback()
{
    rises_interfaces::msg::ObstacleReport::SharedPtr report;
    {
        std::lock_guard<std::mutex> lock(this->state_mutex_);
        report = this->last_report_;
    }

    if (!report) return;

    const AgvPosition pos = this->getRobotPosition();
    const std::string state_json = Vda5050Converter::obstacleReportToStateJson(*report, pos);
    this->publishStateJson(state_json);
}

AgvPosition Vda5050StateBridge::getRobotPosition() const
{
    AgvPosition pos;

    if (this->config_.position_source == "topic") {
        std::lock_guard<std::mutex> lock(this->state_mutex_);
        if (this->last_pose_) {
            pos.x = this->last_pose_->pose.position.x;
            pos.y = this->last_pose_->pose.position.y;
            const geometry_msgs::msg::Quaternion& q = this->last_pose_->pose.orientation;
            pos.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            pos.valid = true;
        }
    } else if (this->tf_buffer_) {
        try {
            const std::string target = this->config_.tf_prefix + this->config_.target_frame;
            const geometry_msgs::msg::TransformStamped tf = this->tf_buffer_->lookupTransform(
                this->config_.global_frame, target, tf2::TimePointZero);
            pos.x = tf.transform.translation.x;
            pos.y = tf.transform.translation.y;
            const geometry_msgs::msg::Quaternion& q = tf.transform.rotation;
            pos.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            pos.valid = true;
        } catch (const tf2::TransformException&) {
            // Position not available
        }
    }

    return pos;
}

} // namespace rises
