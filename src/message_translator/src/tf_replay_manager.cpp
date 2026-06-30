#include "message_translator/tf_replay_manager.hpp"
#include <algorithm>
#include <nlohmann/json.hpp>

namespace rises {

TFReplayManager::TFReplayManager(
    rclcpp::Node* node,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster)
: node_(node)
, tf_broadcaster_(tf_broadcaster)
, tf_static_broadcaster_(tf_static_broadcaster)
, replay_tf_index_(0)
, replay_scan_index_(0)
, replay_validation_index_(0)
, replay_rate_(1.0)
, is_replaying_(false)
{
}

void TFReplayManager::bufferMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    
    TimestampedTF timestamped_tf;
    // Use first transform's timestamp for replay synchronization
    // Critical: ensures TF messages replay in correct temporal order with other data streams
    if (!msg->transforms.empty()) {
        timestamped_tf.timestamp = msg->transforms[0].header.stamp;
    } else {
        timestamped_tf.timestamp = this->node_->now();
    }
    timestamped_tf.message = *msg;
    this->buffered_tf_.push_back(timestamped_tf);
    
    static int log_count = 0;
    if (log_count++ % 50 == 0) {
        RCLCPP_DEBUG(this->node_->get_logger(), 
            "[TF REPLAY] Buffered %zu TF messages", 
            this->buffered_tf_.size());
    }
}

void TFReplayManager::bufferLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                       const std::string& topic_name)
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    
    TimestampedLaserScan timestamped_scan;
    timestamped_scan.timestamp = msg->header.stamp;
    timestamped_scan.message = *msg;
    timestamped_scan.topic_name = topic_name;
    this->buffered_laserscans_.push_back(timestamped_scan);
    
    static int log_count = 0;
    if (log_count++ % 20 == 0) {
        RCLCPP_DEBUG(this->node_->get_logger(), 
            "[LASERSCAN REPLAY] Buffered %zu LaserScan messages", 
            this->buffered_laserscans_.size());
    }
}

void TFReplayManager::bufferValidation(const std_msgs::msg::String::SharedPtr msg)
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    
    TimestampedValidation timestamped_validation;
    timestamped_validation.timestamp = this->node_->now();
    timestamped_validation.message = *msg;
    this->buffered_validations_.push_back(timestamped_validation);
    
    static int log_count = 0;
    if (log_count++ % 10 == 0) {
        RCLCPP_DEBUG(this->node_->get_logger(), 
            "[VALIDATION REPLAY] Buffered %zu validation messages", 
            this->buffered_validations_.size());
    }
}

void TFReplayManager::relayMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for (const geometry_msgs::msg::TransformStamped& transform : msg->transforms) {
        this->publishTransform(transform);
    }
}

void TFReplayManager::publishTransform(const geometry_msgs::msg::TransformStamped& transform)
{
    const std::string parent = transform.header.frame_id;
    
    // Route to appropriate broadcaster based on parent frame
    // Static transforms (base_link relations) use static broadcaster for latching behavior
    // Dynamic transforms (map relations) use regular broadcaster for updates
    if (parent == "map") {
        this->tf_broadcaster_->sendTransform(transform);
    } else if (parent.find("base_link") != std::string::npos) {
        this->tf_static_broadcaster_->sendTransform(transform);
    } else {
        this->tf_broadcaster_->sendTransform(transform);
    }
}

void TFReplayManager::startReplay(const double replay_rate)
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    
    const std::size_t total_count = this->buffered_tf_.size() + this->buffered_laserscans_.size() + this->buffered_validations_.size();
    if (total_count == 0) {
        RCLCPP_WARN(this->node_->get_logger(), "[REPLAY] No messages to replay");
        return;
    }
    
    // Temporal sorting ensures chronological replay across all message types
    // Without sorting, interleaved TF and sensor data would desynchronize
    std::sort(this->buffered_tf_.begin(), this->buffered_tf_.end(),
        [](const TimestampedTF& a, const TimestampedTF& b) {
            return a.timestamp < b.timestamp;
        });
    
    std::sort(this->buffered_laserscans_.begin(), this->buffered_laserscans_.end(),
        [](const TimestampedLaserScan& a, const TimestampedLaserScan& b) {
            return a.timestamp < b.timestamp;
        });
    
    std::sort(this->buffered_validations_.begin(), this->buffered_validations_.end(),
        [](const TimestampedValidation& a, const TimestampedValidation& b) {
            return a.timestamp < b.timestamp;
        });
    
    RCLCPP_INFO(this->node_->get_logger(), 
        "[REPLAY] Starting replay: %zu TF, %zu LaserScans, %zu validations at %.2fx speed",
        this->buffered_tf_.size(), this->buffered_laserscans_.size(), this->buffered_validations_.size(), replay_rate);
    
    this->replay_tf_index_ = 0;
    this->replay_scan_index_ = 0;
    this->replay_validation_index_ = 0;
    this->replay_rate_ = replay_rate;
    this->replay_start_time_ = this->node_->now();
    
    // Calculate time offset between recorded data and current replay time
    // Enables rate-scaled playback: (recorded_time - base) * rate + replay_start
    rclcpp::Time earliest = this->node_->now();
    if (!this->buffered_tf_.empty()) earliest = this->buffered_tf_[0].timestamp;
    if (!this->buffered_laserscans_.empty() && this->buffered_laserscans_[0].timestamp < earliest) 
        earliest = this->buffered_laserscans_[0].timestamp;
    if (!this->buffered_validations_.empty() && this->buffered_validations_[0].timestamp < earliest) 
        earliest = this->buffered_validations_[0].timestamp;
    
    this->replay_base_time_ = earliest;
    this->is_replaying_ = true;
    
    // Timer frequency scales with replay rate for proper temporal behavior
    // 2x rate = 2x speed, 0.5x rate = half speed (slow motion)
    const double interval_ms = 10.0 / replay_rate;
    this->replay_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(interval_ms)),
        std::bind(&TFReplayManager::replayTimerCallback, this)
    );
}

void TFReplayManager::replayTimerCallback()
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    
    const bool all_done = (this->replay_tf_index_ >= this->buffered_tf_.size() &&
                          this->replay_scan_index_ >= this->buffered_laserscans_.size() &&
                          this->replay_validation_index_ >= this->buffered_validations_.size());
    
    if (all_done) {
        const std::size_t total = this->replay_tf_index_ + this->replay_scan_index_ + this->replay_validation_index_;
        RCLCPP_INFO(this->node_->get_logger(), 
            "[REPLAY] Complete: %zu messages replayed (%zu TF, %zu scans, %zu validations)", 
            total, this->replay_tf_index_, this->replay_scan_index_, this->replay_validation_index_);
        this->replay_timer_->cancel();
        this->is_replaying_ = false;
        this->buffered_tf_.clear();
        this->buffered_laserscans_.clear();
        this->buffered_validations_.clear();
        return;
    }
    
    // Calculate scaled time: wall clock * rate + base time
    const rclcpp::Duration elapsed = this->node_->now() - this->replay_start_time_;
    const rclcpp::Duration scaled_elapsed = elapsed * this->replay_rate_;
    const rclcpp::Time current_replay_time = this->replay_base_time_ + scaled_elapsed;
    
    // Publish TF messages up to current replay time
    while (this->replay_tf_index_ < this->buffered_tf_.size()) {
        const TimestampedTF& tf = this->buffered_tf_[this->replay_tf_index_];
        if (tf.timestamp > current_replay_time) break;
        
        for (const geometry_msgs::msg::TransformStamped& transform : tf.message.transforms) {
            this->publishTransform(transform);
        }
        this->replay_tf_index_++;
    }
    
    // Publish LaserScans up to current replay time
    while (this->replay_scan_index_ < this->buffered_laserscans_.size()) {
        const TimestampedLaserScan& scan = this->buffered_laserscans_[this->replay_scan_index_];
        if (scan.timestamp > current_replay_time) break;
        
        this->publishLaserScan(scan);
        this->replay_scan_index_++;
    }
    
    // Publish validations up to current replay time
    while (this->replay_validation_index_ < this->buffered_validations_.size()) {
        const TimestampedValidation& validation = this->buffered_validations_[this->replay_validation_index_];
        if (validation.timestamp > current_replay_time) break;
        
        this->publishValidation(validation);
        this->replay_validation_index_++;
    }
    
    static int log_counter = 0;
    if (log_counter++ % 100 == 0) {
        const std::size_t total = this->replay_tf_index_ + this->replay_scan_index_ + this->replay_validation_index_;
        const std::size_t total_count = this->buffered_tf_.size() + this->buffered_laserscans_.size() + this->buffered_validations_.size();
        RCLCPP_DEBUG(this->node_->get_logger(), 
            "[REPLAY] Progress: %zu/%zu messages (%.1f%%)",
            total, total_count, 100.0 * total / total_count);
    }
}

void TFReplayManager::publishLaserScan(const TimestampedLaserScan& scan)
{
    if (this->laserscan_pub_) {
        this->laserscan_pub_->publish(scan.message);
        RCLCPP_DEBUG(this->node_->get_logger(), 
            "[REPLAY] Published LaserScan to %s", scan.topic_name.c_str());
    } else {
        RCLCPP_WARN_ONCE(this->node_->get_logger(), 
            "[REPLAY] LaserScan publisher not set, skipping replay");
    }
}

void TFReplayManager::publishValidation(const TimestampedValidation& validation)
{
    if (!this->validation_pub_) {
        RCLCPP_WARN_ONCE(this->node_->get_logger(), 
            "[REPLAY] Validation publisher not set, skipping replay");
        return;
    }
    
    try {
        const nlohmann::json j = nlohmann::json::parse(validation.message.data);
        
        if (!j.contains("aabb") || !j.contains("id")) {
            RCLCPP_WARN(this->node_->get_logger(), "Invalid validation JSON during replay");
            return;
        }
        
        const int64_t id = j["id"];
        const std::vector<std::vector<float>> aabb = j["aabb"].get<std::vector<std::vector<float>>>();
        
        rises_interfaces::msg::Obstacle obs;
        if (aabb.size() == 2 && aabb[0].size() == 2 && aabb[1].size() == 2) {
            const float x0 = aabb[0][0], y0 = aabb[0][1];
            const float x1 = aabb[1][0], y1 = aabb[1][1];
            
            obs.id = id;
            obs.type = rises_interfaces::msg::Obstacle::RECTANGLE;
            obs.position.x = (x0 + x1) / 2.0f;
            obs.position.y = (y0 + y1) / 2.0f;
            obs.position.z = 0.0f;
            obs.width = std::abs(x1 - x0);
            obs.height = std::abs(y1 - y0);
            obs.orientation = 0.0f;
        }
        
        std::unique_ptr<rises_interfaces::msg::ObstacleArray> obs_array =
            std::make_unique<rises_interfaces::msg::ObstacleArray>();
        obs_array->header.stamp = validation.timestamp;
        obs_array->header.frame_id = "map";
        obs_array->obstacles.push_back(obs);
        
        this->validation_pub_->publish(std::move(obs_array));
        RCLCPP_DEBUG(this->node_->get_logger(), "[REPLAY] Published validation obstacle id=%ld", id);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->node_->get_logger(), 
            "[REPLAY] Failed to parse validation JSON: %s", e.what());
    }
}

std::size_t TFReplayManager::getBufferedCount() const
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    return this->buffered_tf_.size() + this->buffered_laserscans_.size() + this->buffered_validations_.size();
}

void TFReplayManager::clearBuffer()
{
    const std::lock_guard<std::mutex> lock(this->replay_mutex_);
    this->buffered_tf_.clear();
    this->buffered_laserscans_.clear();
    this->buffered_validations_.clear();
    this->replay_tf_index_ = 0;
    this->replay_scan_index_ = 0;
    this->replay_validation_index_ = 0;
}

} // namespace rises
