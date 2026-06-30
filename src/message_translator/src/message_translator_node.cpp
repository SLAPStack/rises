#include "message_translator/message_translator_node.hpp"
#include "message_translator/aabb_converter.hpp"
#include "message_translator/area_locks_converter.hpp"
#include "message_translator/contours_converter.hpp"
#include "message_translator/vda5050_converter.hpp"
#include "message_translator/vda5050_state_bridge.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "rises_interfaces/msg/contours.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <unordered_map>
#include <unordered_set>
slapstack::MessageTranslatorNode::MessageTranslatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("message_translator", options)
{
    // Large queue depth to prevent message drops during fast rosbag playback (e.g. 5x speed).
    this->obstacle_updates_sub_ = this->create_subscription<std_msgs::msg::String>(
        "obstacle_json", rclcpp::QoS(5000).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->obstacleUpdatesCallback(msg);
        });

    this->order_sub_ = this->create_subscription<std_msgs::msg::String>(
        "order", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->vda5050OrderCallback(msg);
        });
    this->contours_sub_ = this->create_subscription<std_msgs::msg::String>(
        "warehouse_contours_json", rclcpp::QoS(10).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->contoursCallback(msg);
        });

    this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "incoming_path", 10);
    
    const rclcpp::QoS contours_qos = rclcpp::QoS(10).reliable().transient_local();
    this->contours_pub_ = this->create_publisher<rises_interfaces::msg::Contours>(
        "warehouse_contours", contours_qos);
    
    const rclcpp::QoS map_updates_qos = rclcpp::QoS(100).reliable().durability_volatile();
    this->obstacle_pub_ =
        this->create_publisher<rises_interfaces::msg::ObstacleUpdateArray>(
            "map_updates", map_updates_qos);
    this->validation_sub_ = this->create_subscription<std_msgs::msg::String>(
        "validation_mqtt", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->validationCallback(msg);
        });
    this->validation_pub_ =
        this->create_publisher<rises_interfaces::msg::ObstacleArray>(
            "validation", 10);

    this->area_locks_sub_ = this->create_subscription<std_msgs::msg::String>(
        "area_locks_json", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->areaLocksCallback(msg);
        });
    this->area_state_client_ =
        this->create_client<rises_interfaces::srv::SetAreaState>("set_area_state");

    this->base_link_pub_ = this->create_publisher<
        geometry_msgs::msg::PoseStamped>(
            "base_link_pose", 10);
    
    // CRITICAL QoS CONFIGURATION FOR TF RELAY FROM ROSBAG2
    // rosbag2 player uses VOLATILE durability regardless of original recording QoS!
    // Recorded TF messages may have TRANSIENT_LOCAL, but playback forces VOLATILE.
    // We must match the player's output QoS (RELIABLE + VOLATILE), not recording QoS.
    // Mismatch causes "incompatible QoS" warnings and dropped transforms.
    const rclcpp::QoS tf_qos = rclcpp::QoS(100)
        .reliable()                  // Match bag's RELIABLE reliability
        .durability_volatile();      // Match rosbag2 player's VOLATILE playback
    
    // Disable QoS event callbacks to suppress informational warnings about incompatible subscribers
    // Other nodes may use different QoS; warnings are noise when relay functions correctly
    rclcpp::SubscriptionOptions sub_options;
    sub_options.event_callbacks.incompatible_qos_callback = nullptr;
    
    // Log QoS configuration for debugging
    RCLCPP_INFO(this->get_logger(),
        "[TF RELAY] Subscribing with QoS: reliability=%s, durability=%s, depth=%zu",
        tf_qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "RELIABLE" : "BEST_EFFORT",
        tf_qos.durability() == rclcpp::DurabilityPolicy::Volatile ? "VOLATILE" : "TRANSIENT_LOCAL",
        tf_qos.depth());

    this->tf_raw_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/mqtt/agv/tf", tf_qos,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                this->tfStampCallback(msg);
            },
            sub_options);

    RCLCPP_INFO(this->get_logger(), "[TF RELAY] Subscribed to /mqtt/agv/tf (will republish to /tf and /tf_static)");

    try {
        this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "[TF RELAY] TransformBroadcaster created successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[TF RELAY] Failed to create TransformBroadcaster: %s", e.what());
        throw;
    }

    try {
        this->tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "[TF RELAY] StaticTransformBroadcaster created successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[TF RELAY] Failed to create StaticTransformBroadcaster: %s", e.what());
        throw;
    }

    RCLCPP_INFO(this->get_logger(), "[TF RELAY] Transform broadcasters initialized (using default TF QoS)");

    this->map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    this->target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    this->declare_parameter<std::string>("tf_prefix", "");
    this->tf_prefix_ = this->get_parameter("tf_prefix").as_string();

    this->declare_parameter("prefix_global_frame", false);
    const bool prefix_global = this->get_parameter("prefix_global_frame").as_bool();
    this->global_frame_ = prefix_global ? (this->tf_prefix_ + this->map_frame_) : this->map_frame_;

    // Ensure TF prefix ends with underscore for proper frame ID construction
    if (!this->tf_prefix_.empty() && this->tf_prefix_.back() != '_')
        this->tf_prefix_ += '_';
    
    // Buffering mechanism for synchronized obstacle + TF + scan replay
    // Prevents partial data from reaching geofence before static map is established
    this->enable_buffering_ = this->declare_parameter<bool>("enable_buffering", false);
    this->bulk_only_ = this->declare_parameter<bool>("bulk_only", false);
    this->buffer_timeout_sec_ = this->declare_parameter<double>("buffer_timeout", 1.0);
    this->last_update_wall_time_ = std::chrono::steady_clock::now();
    
    // TF replay with time scaling for rosbag playback rate control
    // Enables synchronized replay of transforms at custom speeds (e.g., 2x, 0.5x)
    this->enable_replay_ = this->declare_parameter<bool>("enable_replay", true);
    this->replay_rate_ = this->declare_parameter<double>("replay_rate", 1.0);
    
    this->tf_replay_manager_ = std::make_unique<rises::TFReplayManager>(
        this,
        this->tf_broadcaster_,
        this->tf_static_broadcaster_
    );
    RCLCPP_INFO(this->get_logger(), "[INIT] TFReplayManager created (replay %s, rate: %.1fx)",
        this->enable_replay_ ? "ENABLED" : "DISABLED", this->replay_rate_);
    
    // Create laserscan publisher for replay
    this->laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", 10);
    
    // Set publishers on TFReplayManager for message replay
    this->tf_replay_manager_->setLaserScanPublisher(this->laserscan_pub_);
    // validation_pub_ will be set after delivery strategy initialization
    
    // Subscribe to laserscans for buffering
    this->laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/mqtt/agv/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->laserScanCallback(msg);
        });
    
    RCLCPP_INFO(this->get_logger(), "[INIT] Subscribed to /mqtt/agv/scan for buffering and replay");

    // VDA5050 state bridge: obstacle reports/alerts -> JSON state -> ROS + MQTT
    {
        rises::Vda5050StateBridge::Config state_config;
        state_config.position_source = this->declare_parameter<std::string>("position_source", "tf");
        state_config.state_publish_rate = this->declare_parameter<double>("state_publish_rate", 1.0);
        state_config.global_frame = this->global_frame_;
        state_config.target_frame = this->target_frame_;
        state_config.tf_prefix = this->tf_prefix_;
        this->state_bridge_ = std::make_unique<rises::Vda5050StateBridge>(
            this, state_config, this->tf_buffer_);
    }

    this->use_service_for_map_updates_ = this->declare_parameter<bool>("use_service_for_map_updates", true);

    this->service_chunk_size_ = this->declare_parameter<int>("service_chunk_size", 1024);
    this->service_timeout_ = this->declare_parameter<double>("service_timeout", 300.0);

    // Multi-AGV coordination: number of geofence nodes to coordinate with
    // Each AGV runs a geofence node; translator waits for all to be ready
    this->agv_count_ = this->declare_parameter<int>("agv_count", 1);

    const bool use_env_agv_count = this->declare_parameter<bool>("use_env_agv_count", false);
    if (use_env_agv_count) {
        // Allow runtime override of AGV count via environment variable
        // Useful for dynamic deployment scaling without launch file changes
        const char* env_count = std::getenv("AGV_COUNT");
        if (env_count != nullptr) {
            try {
                this->agv_count_ = std::stoi(env_count);
                RCLCPP_INFO(this->get_logger(),
                    "AGV count overridden from environment: %d", this->agv_count_);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(),
                    "Invalid AGV_COUNT environment variable, using parameter value: %d", this->agv_count_);
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                "use_env_agv_count is true but AGV_COUNT not set, using parameter value: %d", this->agv_count_);
        }
    }

    if (this->agv_count_ < 1) {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid agv_count (%d), must be >= 1. Using 1.", this->agv_count_);
        this->agv_count_ = 1;
    }

    this->publish_init_ready_ = this->declare_parameter<bool>("publish_init_ready", false);
    this->call_completion_service_ = this->declare_parameter<bool>("call_completion_service", false);
    this->completion_service_name_ = this->declare_parameter<std::string>("completion_service_name", "map_update_complete");

    this->wait_for_geofence_ready_ = this->declare_parameter<bool>("wait_for_geofence_ready", false);
    this->geofence_ready_timeout_ = this->declare_parameter<double>("geofence_ready_timeout", 30.0);

    RCLCPP_INFO(this->get_logger(),
        "[INIT] Translator node in namespace: '%s', configured for %d AGV(s)",
        this->get_namespace(), this->agv_count_);

    {
        rclcpp::QoS ready_qos(1);
        ready_qos.transient_local();
        ready_qos.reliable();
        this->ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("initialization_ready", ready_qos);
    }

    // Geofence ready signal coordination for multi-AGV synchronization
    // Ensures all geofence nodes complete initialization before data flow begins
    if (this->wait_for_geofence_ready_) {
        RCLCPP_INFO(this->get_logger(), "[INIT] Waiting for geofence ready signals enabled (timeout: %.1fs)",
            this->geofence_ready_timeout_);

        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.transient_local();
        qos.reliable();

        for (int i = 0; i < this->agv_count_; i++) {
            const std::string agv_namespace = "agv_" + std::to_string(i);
            const std::string topic_name = "/" + agv_namespace + "/geofence_ready";

            // Use TRANSIENT_LOCAL to receive the signal even if published before subscription
            // ("late-joiner" pattern for initialization coordination)
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub = this->create_subscription<std_msgs::msg::Bool>(
                topic_name, qos,
                [this, agv_namespace](const std_msgs::msg::Bool::SharedPtr msg) {
                    this->geofenceReadyCallback(agv_namespace, msg);
                });
            this->geofence_ready_subs_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "  - Subscribed to: %s (QoS: TRANSIENT_LOCAL, RELIABLE, depth=10)",
                topic_name.c_str());
        }

        this->geofence_ready_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->geofence_ready_timeout_),
            [this]() { this->geofenceReadyTimeoutCallback(); });
    } else {
        this->all_geofences_ready_.store(true);
    }

    // === Per-AGV order relay ===
    // When enabled, the translator subscribes to raw per-AGV order topics from
    // the rosbag (e.g. /mqtt/agv/id0/order) and republishes them on the remapped
    // topics (e.g. /agv_0/order) that Unity's OrderROSSubscriber listens on.
    // This solves the timing race where rosbag finishes publishing before
    // Unity's ROS-TCP-Endpoint has DDS subscribers ready for order topics.
    //
    // Orders are buffered and flushed either when:
    //   (a) initialization_ready is published (map init complete), or
    //   (b) order_relay_delay seconds have passed since the first order arrived
    //       (fallback when buffering is disabled or map init never completes).
    this->relay_orders_ = this->declare_parameter<bool>("relay_orders", true);
    this->order_relay_delay_ = this->declare_parameter<double>("order_relay_delay", 10.0);

    if (this->relay_orders_ && this->agv_count_ > 0) {
        RCLCPP_INFO(this->get_logger(),
            "[INIT] Order relay enabled for %d AGV(s) (flush delay: %.1fs)",
            this->agv_count_, this->order_relay_delay_);

        for (int i = 0; i < this->agv_count_; i++) {
            // Subscribe to raw rosbag topic (absolute, not affected by namespace)
            const std::string sub_topic =
                "/mqtt/agv/id" + std::to_string(i) + "/order";

            // Publish on the topic Unity expects (absolute)
            const std::string pub_topic =
                "/agv_" + std::to_string(i) + "/order";

            auto pub = this->create_publisher<std_msgs::msg::String>(pub_topic, 10);
            this->agv_order_pubs_.push_back(pub);

            auto sub = this->create_subscription<std_msgs::msg::String>(
                sub_topic, rclcpp::QoS(100).reliable().durability_volatile(),
                [this, i](const std_msgs::msg::String::SharedPtr msg) {
                    this->agvOrderRelayCallback(i, msg);
                });
            this->agv_order_subs_.push_back(sub);

            RCLCPP_INFO(this->get_logger(),
                "  - Order relay: %s -> %s", sub_topic.c_str(), pub_topic.c_str());
        }
    }

    if (this->use_service_for_map_updates_) {
        RCLCPP_INFO(this->get_logger(), "[INIT] Using SERVICE delivery strategy for %d AGV(s)...", this->agv_count_);
        
        std::vector<rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr> clients;
        for (int i = 0; i < this->agv_count_; i++) {
            std::string service_name;
            if (this->agv_count_ == 1) {
                service_name = "update_map";
            } else {
                service_name = "/agv_" + std::to_string(i) + "/update_map";
            }
            rclcpp::Client<rises_interfaces::srv::UpdateMap>::SharedPtr client = this->create_client<rises_interfaces::srv::UpdateMap>(service_name);
            clients.push_back(client);
            RCLCPP_INFO(this->get_logger(), "  - Service client %d: %s", i, service_name.c_str());
        }
        
        // ServiceDeliveryStrategy holds a non-owning back-reference to this node
        this->delivery_strategy_ = std::make_unique<rises::ServiceDeliveryStrategy>(
            this,
            this->agv_count_,
            std::move(clients),
            this->service_chunk_size_,
            this->service_timeout_
        );
    } else {
        RCLCPP_INFO(this->get_logger(), "[INIT] Using TOPIC delivery strategy");
        
        // TopicDeliveryStrategy holds a non-owning back-reference to this node
        this->delivery_strategy_ = std::make_unique<rises::TopicDeliveryStrategy>(
            this,
            this->obstacle_pub_
        );
    }

    if (this->call_completion_service_) {
        this->completion_service_client_ = this->create_client<std_srvs::srv::Trigger>(this->completion_service_name_);
        RCLCPP_INFO(this->get_logger(), "[INIT] Created completion service client: %s", this->completion_service_name_.c_str());
    }
    
    this->tf_replay_manager_->setValidationPublisher(this->validation_pub_);
    RCLCPP_INFO(this->get_logger(), "[INIT] TFReplayManager publishers configured");

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "========== TRANSLATOR CONFIGURATION ==========");
    RCLCPP_INFO(this->get_logger(), "Delivery Method: %s",
        this->use_service_for_map_updates_ ? "SERVICE" : "TOPIC");
    RCLCPP_INFO(this->get_logger(), "Buffering: %s%s",
        this->enable_buffering_ ? "ENABLED" : "DISABLED",
        this->enable_buffering_ ? (" (silence timeout: " + std::to_string(this->buffer_timeout_sec_) + "s)").c_str() : "");
    RCLCPP_INFO(this->get_logger(), "Bulk Only: %s",
        this->bulk_only_ ? "YES (individual messages discarded)" : "NO");
    RCLCPP_INFO(this->get_logger(), "Replay: %s%s",
        this->enable_replay_ ? "ENABLED" : "DISABLED",
        this->enable_replay_ ? (" (rate: " + std::to_string(this->replay_rate_) + "x)").c_str() : "");
    RCLCPP_INFO(this->get_logger(), "AGV Count: %d", this->agv_count_);
    RCLCPP_INFO(this->get_logger(), "Publish Init Ready Topic: %s",
        this->publish_init_ready_ ? "YES" : "NO");
    RCLCPP_INFO(this->get_logger(), "Completion Service Callback: %s%s",
        this->call_completion_service_ ? "YES" : "NO",
        this->call_completion_service_ ? (" (" + this->completion_service_name_ + ")").c_str() : "");
    RCLCPP_INFO(this->get_logger(), "Wait for Geofence Ready: %s%s",
        this->wait_for_geofence_ready_ ? "YES" : "NO",
        this->wait_for_geofence_ready_ ? (" (timeout: " + std::to_string(this->geofence_ready_timeout_) + "s)").c_str() : "");
    RCLCPP_INFO(this->get_logger(), "Order Relay: %s%s",
        this->relay_orders_ ? "YES" : "NO",
        this->relay_orders_ ? (" (" + std::to_string(this->agv_count_) + " AGVs)").c_str() : "");
    RCLCPP_INFO(this->get_logger(), "Sim Time: %s",
        this->get_parameter("use_sim_time").as_bool() ? "YES" : "NO");
    RCLCPP_INFO(this->get_logger(), "Buffer Timing: steady_clock (wall time)");
    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), "");

    if (this->enable_buffering_) {
        this->buffer_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->bufferTimerCallback(); }
        );
    }

    // Start flush thread last so that all state is fully initialised before the
    // thread begins waiting for flush signals.
    this->flush_thread_ = std::thread([this]() { this->flushThreadLoop(); });
}

slapstack::MessageTranslatorNode::~MessageTranslatorNode()
{
    this->shutdown_.store(true);
    this->flush_cv_.notify_all();
    if (this->flush_thread_.joinable())
        this->flush_thread_.join();
}

void slapstack::MessageTranslatorNode::flushThreadLoop()
{
    while (!this->shutdown_.load()) {
        std::unique_lock<std::mutex> lock(this->flush_cv_mutex_);
        this->flush_cv_.wait(lock, [this]() {
            return this->flush_requested_ || this->shutdown_.load();
        });

        if (this->shutdown_.load()) break;

        this->flush_requested_ = false;
        lock.unlock();

        this->flushBuffer();
    }
}

void slapstack::MessageTranslatorNode::publishBaseLinkPose()
{
    try {
        const std::string target = this->tf_prefix_ + this->target_frame_;
        const geometry_msgs::msg::TransformStamped tf_map_to_base = this->tf_buffer_->lookupTransform(
            this->global_frame_, target, tf2::TimePointZero
        );

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = this->global_frame_;

        pose_msg.pose.position.x = tf_map_to_base.transform.translation.x;
        pose_msg.pose.position.y = tf_map_to_base.transform.translation.y;
        pose_msg.pose.position.z = tf_map_to_base.transform.translation.z;

        pose_msg.pose.orientation = tf_map_to_base.transform.rotation;

        this->base_link_pub_->publish(pose_msg);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform from '%s' to '%s': %s",
                    this->global_frame_.c_str(), 
                    (this->tf_prefix_ + this->target_frame_).c_str(),
                    ex.what());
    }
}


void slapstack::MessageTranslatorNode::obstacleUpdatesCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty obstacle update message");
        return;
    }

    const bool will_buffer = this->enable_buffering_
                           || (this->wait_for_geofence_ready_ && !this->all_geofences_ready_.load());

    // When bulk_only is enabled, discard any message that doesn't contain the
    // "pallets" key (i.e. individual obstacle messages). This keeps the map
    // identical to the JSON snapshot produced by extract_map_data.py.
    if (this->bulk_only_ && msg->data.find("\"pallets\"") == std::string::npos) {
        RCLCPP_DEBUG(this->get_logger(),
            "[MAP_DEBUG] bulk_only: skipping individual obstacle message");
        return;
    }

    std::vector<rises_interfaces::msg::ObstacleUpdate> parsed_updates =
        rises::AabbConverter::parseObstacleUpdates(msg->data);

    if (parsed_updates.empty()) {
        return;
    }

    {
        std::size_t inserts = 0, deletes = 0;
        for (const auto& u : parsed_updates) {
            if (u.operation == rises_interfaces::msg::ObstacleUpdate::OP_INSERT) inserts++;
            else deletes++;
        }
        RCLCPP_DEBUG(this->get_logger(),
            "[MAP_DEBUG] obstacleUpdatesCallback: parsed %zu updates (%zu INSERTs, %zu DELETEs) from JSON",
            parsed_updates.size(), inserts, deletes);
    }

    RCLCPP_DEBUG(this->get_logger(),
        "[MAP_DEBUG] Buffer decision: enable_buffering=%d, wait_for_geofence=%d, geofences_ready=%d, will_buffer=%d",
        this->enable_buffering_, this->wait_for_geofence_ready_,
        this->all_geofences_ready_.load(), will_buffer);

    if (will_buffer) {
        std::lock_guard<std::mutex> lock(this->buffer_mutex_);

        this->buffered_updates_.insert(
            this->buffered_updates_.end(),
            parsed_updates.begin(),
            parsed_updates.end()
        );
        this->last_update_wall_time_ = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(this->get_logger(),
            "[MAP_DEBUG] Buffered %zu updates (buffer total: %zu, waiting for %.2fs silence)",
            parsed_updates.size(), this->buffered_updates_.size(),
            this->buffer_timeout_sec_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[MAP_DEBUG] Buffering: %zu updates received (buffer total: %zu, waiting for %.2fs silence)",
            parsed_updates.size(), this->buffered_updates_.size(),
            this->buffer_timeout_sec_);
    } else {
        if (this->use_service_for_map_updates_) {
            const uint32_t count = static_cast<uint32_t>(parsed_updates.size());
            RCLCPP_INFO(this->get_logger(),
                "[MAP_DEBUG] Calling update_map service for %d AGV(s) with %u obstacles...",
                this->agv_count_, count);
            this->callUpdateMapServiceImmediate(parsed_updates, this->now());
        } else {
            std::unique_ptr<rises_interfaces::msg::ObstacleUpdateArray> arr_msg =
                std::make_unique<rises_interfaces::msg::ObstacleUpdateArray>();
            arr_msg->header.stamp    = this->now();
            arr_msg->header.frame_id = this->global_frame_;
            arr_msg->updates         = parsed_updates;

            const uint32_t count = static_cast<uint32_t>(arr_msg->updates.size());
            RCLCPP_DEBUG(this->get_logger(), "[MAP_DEBUG] Publishing %u obstacles to map updates topic", count);
            this->obstacle_pub_->publish(std::move(arr_msg));
        }
    }
}



void slapstack::MessageTranslatorNode::bufferTimerCallback()
{
    if (!this->enable_buffering_) {
        return;
    }

    // Prevent redundant signals when flush is already in progress.
    if (this->is_flushing_.load()) {
        RCLCPP_DEBUG(this->get_logger(), "[MAP_DEBUG] [TIMER] Flush already in progress, skipping");
        return;
    }

    // Check if silence timeout has elapsed (no new updates for buffer_timeout_sec_)
    bool should_flush = false;
    std::size_t buffer_size = 0;
    double silence_elapsed = 0.0;

    {
        std::lock_guard<std::mutex> lock(this->buffer_mutex_);
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        silence_elapsed = std::chrono::duration<double>(now - this->last_update_wall_time_).count();
        buffer_size = this->buffered_updates_.size();

        if (!this->buffered_updates_.empty() && silence_elapsed >= this->buffer_timeout_sec_) {
            should_flush = true;
        }
    }

    if (buffer_size > 0) {
        RCLCPP_DEBUG(this->get_logger(),
            "[MAP_DEBUG] Timer tick: buffer=%zu, silence=%.2f/%.2fs, flushing=%d, flushed=%d, init_ready_pub=%d",
            buffer_size, silence_elapsed, this->buffer_timeout_sec_,
            this->is_flushing_.load(), this->buffer_flushed_.load(),
            this->init_ready_published_.load());
    }

    if (should_flush) {
        if (!this->wait_for_geofence_ready_ || this->all_geofences_ready_.load()) {
            RCLCPP_INFO(this->get_logger(),
                "[MAP_DEBUG] [TIMER] Buffer silence timeout (%.2fs >= %.2fs) - signalling flush thread (%zu updates)",
                silence_elapsed, this->buffer_timeout_sec_, buffer_size);
            {
                std::lock_guard<std::mutex> cv_lock(this->flush_cv_mutex_);
                this->flush_requested_ = true;
            }
            this->flush_cv_.notify_one();
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[MAP_DEBUG] [TIMER] Buffer has %zu updates but waiting for geofences to be ready",
                buffer_size);
        }
    } else if (buffer_size > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[MAP_DEBUG] [TIMER] Buffer has %zu updates, waiting for silence (%.2f/%.2fs)",
            buffer_size, silence_elapsed, this->buffer_timeout_sec_);
    }

    // Publish initialization_ready once ALL map data has been flushed:
    // buffer is empty, at least one flush succeeded, and we haven't published yet.
    if (buffer_size == 0
        && this->buffer_flushed_.load()
        && !this->init_ready_published_.load()
        && this->publish_init_ready_)
    {
        this->init_ready_published_.store(true);
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = true;
        this->ready_pub_->publish(ready_msg);
        RCLCPP_INFO(this->get_logger(),
            "[MAP_DEBUG] [TIMER] Published /initialization_ready — all buffered map updates delivered");

        // Flush any per-AGV orders that arrived before init was complete.
        if (this->relay_orders_) {
            this->flushBufferedOrders();
        }
    }
}

void slapstack::MessageTranslatorNode::flushBuffer()
{
    RCLCPP_INFO(this->get_logger(), "[MAP_DEBUG] [FLUSH START] flushBuffer called at %.3fs", this->now().seconds());
    
    bool expected = false;
    if (!this->is_flushing_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(this->get_logger(), "[MAP_DEBUG] Flush already in progress, skipping concurrent flush attempt");
        return;
    }

    std::vector<rises_interfaces::msg::ObstacleUpdate> updates_to_send;

    {
        std::lock_guard<std::mutex> lock(this->buffer_mutex_);

        if (this->buffered_updates_.empty()) {
            RCLCPP_WARN(this->get_logger(), "[MAP_DEBUG] Flush called but buffer is empty — skipping");
            this->is_flushing_.store(false);
            return;
        }

        updates_to_send = std::move(this->buffered_updates_);
        this->buffered_updates_.clear();
        this->last_update_wall_time_ = std::chrono::steady_clock::now();
    }

    // Collapse buffer using running-map approach: replay INSERT/DELETE
    // operations sequentially to compute the final map state.  This is
    // semantically identical to extract_map_data.py and produces a
    // deterministic result regardless of message ordering or drops.
    //   INSERT → add/overwrite in the map (keeps latest geometry per ID)
    //   DELETE → remove from the map
    // The final map is sent as pure INSERTs to the geofence nodes.
    {
        const std::size_t raw_count = updates_to_send.size();
        std::size_t insert_ops = 0;
        std::size_t delete_ops = 0;

        std::unordered_map<int64_t, rises_interfaces::msg::ObstacleUpdate> map;
        map.reserve(raw_count);

        for (rises_interfaces::msg::ObstacleUpdate& u : updates_to_send) {
            if (u.operation == rises_interfaces::msg::ObstacleUpdate::OP_INSERT) {
                insert_ops++;
                u.operation = rises_interfaces::msg::ObstacleUpdate::OP_INSERT;
                map.insert_or_assign(u.obstacle.id, std::move(u));
            } else {
                delete_ops++;
                map.erase(u.obstacle.id);
            }
        }

        // Rebuild: all surviving entries become INSERTs.
        updates_to_send.clear();
        updates_to_send.reserve(map.size());
        for (auto& [id, u] : map) {
            updates_to_send.push_back(std::move(u));
        }

        RCLCPP_INFO(this->get_logger(),
            "[MAP_DEBUG] [FLUSH] Collapsed %zu raw operations (%zu INSERTs, %zu DELETEs) → %zu final obstacles (running-map approach)",
            raw_count, insert_ops, delete_ops, updates_to_send.size());
    }

    const std::size_t count = updates_to_send.size();

    RCLCPP_INFO(this->get_logger(),
        "[MAP_DEBUG] [FLUSH] Delivering %zu buffered obstacles using %s strategy",
        count, this->use_service_for_map_updates_ ? "SERVICE" : "TOPIC");
    
    const builtin_interfaces::msg::Time stamp = this->now();
    const bool all_success = this->delivery_strategy_->deliver(updates_to_send, stamp);

    if (all_success) {
        RCLCPP_INFO(this->get_logger(), "[MAP_DEBUG] [FLUSH] Buffer flush SUCCESSFUL (%zu updates)", count);

        // Mark that at least one flush has succeeded.
        // initialization_ready is NOT published here — we wait until the
        // buffer timer confirms no more data is arriving (empty buffer after
        // silence timeout), so the signal only fires once ALL map updates
        // have been delivered to the geofence nodes.
        const bool first_flush = !this->buffer_flushed_.load();
        this->buffer_flushed_.store(true);

        RCLCPP_DEBUG(this->get_logger(),
            "[MAP_DEBUG] Flush complete: first_flush=%d, buffer_flushed=%d, init_ready_published=%d",
            first_flush, this->buffer_flushed_.load(), this->init_ready_published_.load());

        if (first_flush && this->enable_replay_) {
            this->tf_replay_manager_->startReplay(this->replay_rate_);
        }
    } else {
        // Put updates back into the buffer so they are not lost.
        // The buffer timer will trigger another flush attempt once the
        // silence timeout elapses again.
        {
            std::lock_guard<std::mutex> lock(this->buffer_mutex_);
            this->buffered_updates_.insert(
                this->buffered_updates_.begin(),
                std::make_move_iterator(updates_to_send.begin()),
                std::make_move_iterator(updates_to_send.end()));
            this->last_update_wall_time_ = std::chrono::steady_clock::now();
        }
        RCLCPP_ERROR(this->get_logger(),
            "[MAP_DEBUG] [FLUSH] Buffer flush FAILED — %zu updates returned to buffer; will retry on next timer tick",
            count);
    }

    if (this->call_completion_service_) {
        if (!this->completion_service_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(),
                "Completion service '%s' not available - skipping callback",
                this->completion_service_name_.c_str());
        } else {
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();
            std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future = this->completion_service_client_->async_send_request(request).future.share();

            const std::future_status status = future.wait_for(std::chrono::seconds(5));
            if (status == std::future_status::ready) {
                const std::shared_ptr<std_srvs::srv::Trigger::Response> response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "Called completion service '%s': %s",
                        this->completion_service_name_.c_str(),
                        response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Completion service '%s' returned failure: %s",
                        this->completion_service_name_.c_str(),
                        response->message.c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Completion service '%s' call timeout after 5 seconds",
                    this->completion_service_name_.c_str());
            }
        }
    }

    this->is_flushing_.store(false);
    RCLCPP_DEBUG(this->get_logger(), "[MAP_DEBUG] [FLUSH] Flush complete, guard cleared");
}

void slapstack::MessageTranslatorNode::callUpdateMapServiceImmediate(
    const std::vector<rises_interfaces::msg::ObstacleUpdate>& updates,
    const builtin_interfaces::msg::Time& /*timestamp*/)
{
    // SERVICE delivery uses async_send_request + wait_for(), which requires
    // the executor to spin for processing the response callback.  Calling
    // it inline from a subscription callback (single-threaded executor)
    // would deadlock.  Queue the updates and let the dedicated flush thread
    // handle the service call instead.
    {
        std::lock_guard<std::mutex> lock(this->buffer_mutex_);
        this->buffered_updates_.insert(
            this->buffered_updates_.end(),
            updates.begin(),
            updates.end());
        this->last_update_wall_time_ = std::chrono::steady_clock::now();
    }

    {
        std::lock_guard<std::mutex> lock(this->flush_cv_mutex_);
        this->flush_requested_ = true;
    }
    this->flush_cv_.notify_one();

    RCLCPP_DEBUG(this->get_logger(),
        "[MAP_DEBUG] [IMMEDIATE] Queued %zu updates for flush-thread delivery", updates.size());
}


void slapstack::MessageTranslatorNode::validationCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // If buffering is enabled and not yet flushed, buffer this message for replay.
    if (this->enable_buffering_ && !this->buffer_flushed_.load()) {
        this->tf_replay_manager_->bufferValidation(msg);

        static int validation_buffer_count = 0;
        if (validation_buffer_count++ % 10 == 0) {
            RCLCPP_DEBUG(this->get_logger(),
                "[VALIDATION BUFFER] Buffered validation message for replay");
        }
        return;
    }

    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty validation message");
        return;
    }

    const rises_interfaces::msg::Obstacle obs =
        rises::AabbConverter::parseValidationObstacle(msg->data);

    if (obs.type == 0) {
        return;  // AabbConverter already logged the failure.
    }

    std::unique_ptr<rises_interfaces::msg::ObstacleArray> arr =
        std::make_unique<rises_interfaces::msg::ObstacleArray>();
    arr->header.stamp    = this->now();
    arr->header.frame_id = this->global_frame_;
    arr->obstacles.push_back(obs);

    this->validation_pub_->publish(std::move(arr));
    RCLCPP_DEBUG(this->get_logger(),
        "Published validation obstacle id=%lu type=%u at (%.3f, %.3f)",
        obs.id, obs.type, obs.position.x, obs.position.y);
}

void slapstack::MessageTranslatorNode::areaLocksCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty area_locks message");
        return;
    }

    bool ok = false;
    const rises_interfaces::msg::AreaState area_state =
        rises::AreaLocksConverter::parse(msg->data, ok);

    if (!ok) {
        return;  // AreaLocksConverter already logged the failure.
    }

    if (!this->area_state_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(),
            "set_area_state service not available; dropping area_locks message (id=%ld)",
            area_state.id);
        return;
    }

    auto request = std::make_shared<rises_interfaces::srv::SetAreaState::Request>();
    request->area_id = area_state.id;
    request->lock = (area_state.operation == rises_interfaces::msg::AreaState::LOCK);
    request->x0 = area_state.x0;
    request->y0 = area_state.y0;
    request->x1 = area_state.x1;
    request->y1 = area_state.y1;

    this->area_state_client_->async_send_request(request,
        [this, area_state](rclcpp::Client<rises_interfaces::srv::SetAreaState>::SharedFuture future) {
            try {
                const auto response = future.get();
                if (response->success) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "Area %s successful (id=%ld): %s",
                        area_state.operation == rises_interfaces::msg::AreaState::LOCK ? "lock" : "unlock",
                        area_state.id, response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Area %s failed (id=%ld): %s",
                        area_state.operation == rises_interfaces::msg::AreaState::LOCK ? "lock" : "unlock",
                        area_state.id, response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                    "set_area_state service call failed (id=%ld): %s",
                    area_state.id, e.what());
            }
        });
}

void slapstack::MessageTranslatorNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // If buffering is enabled and not yet flushed, buffer this message for replay
    if (this->enable_buffering_ && !this->buffer_flushed_.load()) {
        this->tf_replay_manager_->bufferLaserScan(msg, "/mqtt/agv/scan");
        
        static int laserscan_buffer_count = 0;
        if (laserscan_buffer_count++ % 20 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "[LASERSCAN BUFFER] Buffered %d LaserScan messages for replay", laserscan_buffer_count);
        }
        return;  // Don't publish yet, wait for geofence initialization
    }
    
    // Normal relay mode (after initialization or if buffering disabled)
    this->laserscan_pub_->publish(*msg);
    
    static int laserscan_relay_count = 0;
    if (laserscan_relay_count++ % 50 == 0) {
        RCLCPP_DEBUG(this->get_logger(), 
            "[LASERSCAN RELAY] Relayed %d LaserScan messages", laserscan_relay_count);
    }
}

void slapstack::MessageTranslatorNode::tfStampCallback(
    // ReSharper disable once CppPassValueParameterByConstReference -
    // ROS2 sub expects callback with a SharedPtr (or ConstSharedPtr) by value.
    const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    const std::lock_guard<std::mutex> lock(this->last_message_mutex_);

    if (msg->transforms.empty()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "[TF RELAY] Received empty TF message");
        return;
    }

    // Log first time we receive TF messages
    static bool first_tf_received = false;
    if (!first_tf_received) {
        RCLCPP_INFO(this->get_logger(), 
            "[TF RELAY] ✓ First TF message received with %zu transforms", 
            msg->transforms.size());
        for (const geometry_msgs::msg::TransformStamped& tf : msg->transforms) {
            RCLCPP_INFO(this->get_logger(), 
                "[TF RELAY]   - %s → %s", 
                tf.header.frame_id.c_str(), 
                tf.child_frame_id.c_str());
        }
        first_tf_received = true;
    }

    // === Delegate to TFReplayManager ===
    // If buffering is enabled and not yet flushed, buffer the message
    // Otherwise, relay it immediately
    if (this->enable_buffering_ && !this->buffer_flushed_.load()) {
        this->tf_replay_manager_->bufferMessage(msg);
    } else {
        this->tf_replay_manager_->relayMessage(msg);
    }
}

void slapstack::MessageTranslatorNode::vda5050OrderCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    const nav_msgs::msg::Path path = rises::Vda5050Converter::orderToPath(
        msg->data, this->global_frame_, this->now());

    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "VDA5050 order produced no path waypoints");
        return;
    }

    this->path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(),
        "Published path with %zu waypoints from VDA5050 order", path.poses.size());
}

void slapstack::MessageTranslatorNode::contoursCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty warehouse contours message");
        return;
    }

    const rises_interfaces::msg::Contours contours =
        rises::ContoursConverter::parse(msg->data, this->global_frame_);

    // Log first few hull points for coordinate debugging
    if (!contours.outer_contour_hull.points.empty()) {
        const std::size_t n = std::min<std::size_t>(3, contours.outer_contour_hull.points.size());
        for (std::size_t i = 0; i < n; ++i) {
            const auto& pt = contours.outer_contour_hull.points[i];
            RCLCPP_INFO(this->get_logger(), "[CONTOUR DEBUG] Parsed hull[%zu]: (%.3f, %.3f)", i, pt.x, pt.y);
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Publishing warehouse contours: outer points=%zu, inner polygons=%zu, outer segments=%zu",
        contours.outer_contour_hull.points.size(),
        contours.inner_contours.size(),
        contours.outer_contour_segments.size());

    this->contours_pub_->publish(contours);
}

void slapstack::MessageTranslatorNode::geofenceReadyCallback(
    const std::string& agv_namespace,
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(this->ready_mutex_);
        if (this->ready_agvs_.find(agv_namespace) != this->ready_agvs_.end()) {
            return;
        }
        this->ready_agvs_.insert(agv_namespace);
        RCLCPP_INFO(this->get_logger(),
            "[GEOFENCE READY] %s signaled ready (%zu/%d AGVs ready)",
            agv_namespace.c_str(), this->ready_agvs_.size(), this->agv_count_);
    }

    this->checkAllGeofencesReady();
}

void slapstack::MessageTranslatorNode::geofenceReadyTimeoutCallback()
{
    if (this->geofence_ready_timer_) {
        this->geofence_ready_timer_->cancel();
    }

    const std::lock_guard<std::mutex> lock(this->ready_mutex_);
    if (this->all_geofences_ready_.load()) {
        return;
    }

    RCLCPP_WARN(this->get_logger(),
        "[GEOFENCE READY TIMEOUT] Only %zu/%d geofences ready after %.1fs - proceeding anyway",
        this->ready_agvs_.size(), this->agv_count_, this->geofence_ready_timeout_);

    this->all_geofences_ready_.store(true);

    // Trigger the flush thread so any buffered updates are delivered now.
    {
        std::lock_guard<std::mutex> cv_lock(this->flush_cv_mutex_);
        this->flush_requested_ = true;
    }
    this->flush_cv_.notify_one();
}

void slapstack::MessageTranslatorNode::checkAllGeofencesReady()
{
    const std::lock_guard<std::mutex> lock(this->ready_mutex_);

    if (this->all_geofences_ready_.load()) {
        return;
    }

    if (static_cast<int>(this->ready_agvs_.size()) >= this->agv_count_) {
        RCLCPP_INFO(this->get_logger(),
            "[GEOFENCE READY] All %d geofence nodes are ready!",
            this->agv_count_);

        this->all_geofences_ready_.store(true);

        if (this->geofence_ready_timer_) {
            this->geofence_ready_timer_->cancel();
        }

        // Trigger the flush thread so any buffered updates are delivered now.
        {
            std::lock_guard<std::mutex> cv_lock(this->flush_cv_mutex_);
            this->flush_requested_ = true;
        }
        this->flush_cv_.notify_one();
    }
}

// =========================================================================
// Per-AGV order relay
// =========================================================================

void slapstack::MessageTranslatorNode::agvOrderRelayCallback(
    int agv_id, const std_msgs::msg::String::SharedPtr msg)
{
    if (this->orders_flushed_.load()) {
        // Already flushed — forward immediately (post-init or post-timer).
        this->agv_order_pubs_[agv_id]->publish(*msg);
        RCLCPP_INFO(this->get_logger(),
            "[ORDER RELAY] Forwarded order for AGV %d", agv_id);
        return;
    }

    // Buffer the order.
    std::lock_guard<std::mutex> lock(this->order_relay_mutex_);
    this->buffered_agv_orders_.push_back({agv_id, msg});
    RCLCPP_INFO(this->get_logger(),
        "[ORDER RELAY] Buffered order for AGV %d (total buffered: %zu)",
        agv_id, this->buffered_agv_orders_.size());

    // Start the flush timer on the first buffered order.
    // The timer fires once after order_relay_delay_ seconds, giving Unity's
    // TCP subscribers time to register before orders are published.
    if (!this->order_relay_timer_) {
        RCLCPP_INFO(this->get_logger(),
            "[ORDER RELAY] Starting flush timer (%.1fs)", this->order_relay_delay_);
        this->order_relay_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->order_relay_delay_),
            [this]() { this->orderRelayTimerCallback(); });
    }
}

void slapstack::MessageTranslatorNode::orderRelayTimerCallback()
{
    // One-shot: cancel immediately.
    if (this->order_relay_timer_) {
        this->order_relay_timer_->cancel();
    }

    if (this->orders_flushed_.load()) {
        return;  // Already flushed by init_ready path.
    }

    RCLCPP_INFO(this->get_logger(),
        "[ORDER RELAY] Flush timer fired — publishing buffered orders");
    this->flushBufferedOrders();
}

void slapstack::MessageTranslatorNode::flushBufferedOrders()
{
    if (this->orders_flushed_.load()) {
        return;
    }
    this->orders_flushed_.store(true);

    std::lock_guard<std::mutex> lock(this->order_relay_mutex_);
    if (this->buffered_agv_orders_.empty()) {
        RCLCPP_INFO(this->get_logger(),
            "[ORDER RELAY] No buffered orders to flush");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "[ORDER RELAY] Flushing %zu buffered orders", this->buffered_agv_orders_.size());

    for (const auto& order : this->buffered_agv_orders_) {
        if (order.agv_id >= 0
            && order.agv_id < static_cast<int>(this->agv_order_pubs_.size()))
        {
            this->agv_order_pubs_[order.agv_id]->publish(*order.msg);
            RCLCPP_INFO(this->get_logger(),
                "[ORDER RELAY] Flushed order for AGV %d", order.agv_id);
        }
    }
    this->buffered_agv_orders_.clear();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(slapstack::MessageTranslatorNode)