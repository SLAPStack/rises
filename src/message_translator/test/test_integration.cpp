#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

#include "message_translator/message_translator_node.hpp"

// Integration test that tests the complete workflow
class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create main node
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"mqtt.enabled", false}, // Disable MQTT for integration tests
            {"map_frame", "integration_map"},
            {"target_frame", "integration_base_link"},
            {"tf_prefix", "test_robot_"},
            {"base_link_pub_rate", 5.0},
            {"prefix_global_frame", false}
        });
        
        node_ = std::make_shared<slapstack::MessageTranslatorNode>(options);
        
        // Create helper node for TF broadcasting
        tf_node_ = std::make_shared<rclcpp::Node>("integration_tf_publisher");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(tf_node_);
        
        // Setup subscribers to collect all published messages
        setupSubscribers();
        
        // Setup publishers for sending test data
        setupPublishers();
        
        // Allow nodes to initialize
        rclcpp::spin_some(node_);
        rclcpp::spin_some(tf_node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TearDown() override {
        node_.reset();
        tf_node_.reset();
        rclcpp::shutdown();
    }

private:
    void setupSubscribers() {
        obstacle_sub_ = node_->create_subscription<rises_interfaces::msg::ObstacleUpdateArray>(
            "map_updates", 10,
            [this](const rises_interfaces::msg::ObstacleUpdateArray::SharedPtr msg) {
                received_obstacles_.push_back(*msg);
            });
            
        validation_sub_ = node_->create_subscription<rises_interfaces::msg::ObstacleArray>(
            "validation", 10,
            [this](const rises_interfaces::msg::ObstacleArray::SharedPtr msg) {
                received_validation_.push_back(*msg);
            });
            
        path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
            "incoming_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                received_paths_.push_back(*msg);
            });
            
        contours_sub_ = node_->create_subscription<rises_interfaces::msg::Contours>(
            "warehouse_contours", 10,
            [this](const rises_interfaces::msg::Contours::SharedPtr msg) {
                received_contours_.push_back(*msg);
            });
            
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "base_link_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                received_poses_.push_back(*msg);
            });
            
        tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                received_tf_.push_back(*msg);
            });
    }

    void setupPublishers() {
        obstacle_pub_ = node_->create_publisher<std_msgs::msg::String>("obstacle_json", 10);
        validation_pub_ = node_->create_publisher<std_msgs::msg::String>("validation_mqtt", 10);
        order_pub_ = node_->create_publisher<std_msgs::msg::String>("order", 10);
        contours_pub_ = node_->create_publisher<std_msgs::msg::String>("warehouse_contours_mqtt", 10);
        tf_raw_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf_raw", 10);
    }

protected:
    // Helper methods for publishing test data
    void publishObstacleJson(const nlohmann::json& json) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json.dump();
        obstacle_pub_->publish(*msg);
        spinAndWait();
    }

    void publishValidationJson(const nlohmann::json& json) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json.dump();
        validation_pub_->publish(*msg);
        spinAndWait();
    }

    void publishOrderJson(const nlohmann::json& json) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json.dump();
        order_pub_->publish(*msg);
        spinAndWait();
    }

    void publishContoursJson(const nlohmann::json& json) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json.dump();
        contours_pub_->publish(*msg);
        spinAndWait();
    }

    void publishTransform(const std::string& parent_frame, const std::string& child_frame,
                         double x = 0.0, double y = 0.0, double z = 0.0) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = tf_node_->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.transform.rotation.w = 1.0; // Identity rotation
        
        tf_broadcaster_->sendTransform(transform);
        spinAndWait();
    }

    void spinAndWait() {
        rclcpp::spin_some(node_);
        rclcpp::spin_some(tf_node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(node_);
        rclcpp::spin_some(tf_node_);
    }

    // Node instances
    std::shared_ptr<slapstack::MessageTranslatorNode> node_;
    std::shared_ptr<rclcpp::Node> tf_node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers for sending test data
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr validation_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contours_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_raw_pub_;

    // Subscribers and received data
    rclcpp::Subscription<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<rises_interfaces::msg::ObstacleArray>::SharedPtr validation_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<rises_interfaces::msg::Contours>::SharedPtr contours_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

public:
    std::vector<rises_interfaces::msg::ObstacleUpdateArray> received_obstacles_;
    std::vector<rises_interfaces::msg::ObstacleArray> received_validation_;
    std::vector<nav_msgs::msg::Path> received_paths_;
    std::vector<rises_interfaces::msg::Contours> received_contours_;
    std::vector<geometry_msgs::msg::PoseStamped> received_poses_;
    std::vector<tf2_msgs::msg::TFMessage> received_tf_;
};

// Test complete obstacle processing workflow
TEST_F(IntegrationTest, CompleteObstacleWorkflow) {
    // Test data
    nlohmann::json obstacle_data = {
        {"pallets", {
            {
                {"id", 1001},
                {"operation", "INSERT"},
                {"aabb", {{5.0f, 5.0f}, {7.0f, 8.0f}}}
            },
            {
                {"id", 1002},
                {"operation", "DELETE"},
                {"aabb", {{-2.0f, -3.0f}, {0.0f, -1.0f}}}
            }
        }}
    };

    // Clear any previous data
    received_obstacles_.clear();
    
    // Publish obstacle data
    publishObstacleJson(obstacle_data);
    
    // Verify obstacle processing
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    EXPECT_EQ(obstacle_msg.header.frame_id, "integration_map");
    ASSERT_EQ(obstacle_msg.updates.size(), 2);
    
    // Check first obstacle (INSERT)
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.id, 1001);
    EXPECT_EQ(obstacle_msg.updates[0].operation, rises_interfaces::msg::ObstacleUpdate::OP_INSERT);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.position.x, 6.0f); // Center
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.position.y, 6.5f); // Center
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.width, 2.0f);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.height, 3.0f);
    
    // Check second obstacle (DELETE)
    EXPECT_EQ(obstacle_msg.updates[1].obstacle.id, 1002);
    EXPECT_EQ(obstacle_msg.updates[1].operation, rises_interfaces::msg::ObstacleUpdate::OP_DELETE);
}

// Test complete path processing workflow
TEST_F(IntegrationTest, CompletePathWorkflow) {
    nlohmann::json order_data = {
        {"nodes", {
            {
                {"nodePosition", {
                    {"x", 0.0},
                    {"y", 0.0},
                    {"theta", 0.0}
                }}
            },
            {
                {"nodePosition", {
                    {"x", 5.0},
                    {"y", 3.0},
                    {"theta", 90.0}
                }}
            },
            {
                {"nodePosition", {
                    {"x", 10.0},
                    {"y", 6.0},
                    {"theta", 180.0}
                }}
            }
        }}
    };

    received_paths_.clear();
    publishOrderJson(order_data);
    
    ASSERT_EQ(received_paths_.size(), 1);
    const auto& path_msg = received_paths_[0];
    
    EXPECT_EQ(path_msg.header.frame_id, "integration_map");
    ASSERT_EQ(path_msg.poses.size(), 3);
    
    // Check waypoints
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.position.y, 0.0);
    
    EXPECT_DOUBLE_EQ(path_msg.poses[1].pose.position.x, 5.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[1].pose.position.y, 3.0);
    
    EXPECT_DOUBLE_EQ(path_msg.poses[2].pose.position.x, 10.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[2].pose.position.y, 6.0);
    
    // Check orientations (90° rotation)
    double expected_z = std::sin(M_PI / 4); // 90°/2 in radians
    double expected_w = std::cos(M_PI / 4);
    EXPECT_NEAR(path_msg.poses[1].pose.orientation.z, expected_z, 1e-6);
    EXPECT_NEAR(path_msg.poses[1].pose.orientation.w, expected_w, 1e-6);
}

// Test complete validation workflow
TEST_F(IntegrationTest, CompleteValidationWorkflow) {
    nlohmann::json validation_data = {
        {"id", 2001},
        {"aabb", {{-5.0f, -5.0f}, {5.0f, 5.0f}}}
    };

    received_validation_.clear();
    publishValidationJson(validation_data);
    
    ASSERT_EQ(received_validation_.size(), 1);
    const auto& validation_msg = received_validation_[0];
    
    EXPECT_EQ(validation_msg.header.frame_id, "integration_map");
    ASSERT_EQ(validation_msg.obstacles.size(), 1);
    
    const auto& obstacle = validation_msg.obstacles[0];
    EXPECT_EQ(obstacle.id, 2001);
    EXPECT_FLOAT_EQ(obstacle.position.x, 0.0f); // Center
    EXPECT_FLOAT_EQ(obstacle.position.y, 0.0f); // Center
    EXPECT_FLOAT_EQ(obstacle.width, 10.0f);
    EXPECT_FLOAT_EQ(obstacle.height, 10.0f);
}

// Test complete contour processing workflow
TEST_F(IntegrationTest, CompleteContourWorkflow) {
    nlohmann::json contour_data = {
        {"outer_contour_hull", {
            {0.0f, 0.0f}, {20.0f, 0.0f}, {20.0f, 15.0f}, {0.0f, 15.0f}
        }},
        {"outer_contour_segments", {
            {{0.0f, 0.0f}, {20.0f, 0.0f}},
            {{20.0f, 0.0f}, {20.0f, 15.0f}},
            {{20.0f, 15.0f}, {0.0f, 15.0f}},
            {{0.0f, 15.0f}, {0.0f, 0.0f}}
        }},
        {"inner_contours", {
            {
                {{5.0f, 5.0f}, {10.0f, 5.0f}},
                {{10.0f, 5.0f}, {10.0f, 10.0f}},
                {{10.0f, 10.0f}, {5.0f, 10.0f}},
                {{5.0f, 10.0f}, {5.0f, 5.0f}}
            }
        }}
    };

    received_contours_.clear();
    publishContoursJson(contour_data);
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    EXPECT_EQ(contours_msg.header.frame_id, "integration_map");
    
    // Check outer hull
    ASSERT_EQ(contours_msg.outer_contour_hull.points.size(), 4);
    
    // Check outer segments
    ASSERT_EQ(contours_msg.outer_contour_segments.size(), 4);
    
    // Check inner contours
    ASSERT_EQ(contours_msg.inner_contours.size(), 1);
    ASSERT_EQ(contours_msg.inner_contours[0].points.size(), 5); // Including closing point
}

// Test TF processing and pose publishing workflow
TEST_F(IntegrationTest, CompleteTfWorkflow) {
    // Publish a transform
    publishTransform("integration_map", "test_robot_integration_base_link", 3.5, 4.5, 0.2);
    
    // Wait for pose publication
    received_poses_.clear();
    for (int i = 0; i < 20; ++i) {
        spinAndWait();
        if (!received_poses_.empty()) break;
    }
    
    ASSERT_GE(received_poses_.size(), 1);
    const auto& pose_msg = received_poses_[0];
    
    EXPECT_EQ(pose_msg.header.frame_id, "integration_map");
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.x, 3.5);
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.y, 4.5);
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.z, 0.2);
}

// Test mixed workflow with multiple message types
TEST_F(IntegrationTest, MixedMessageWorkflow) {
    // Clear all received messages
    received_obstacles_.clear();
    received_paths_.clear();
    received_validation_.clear();
    received_contours_.clear();

    // Publish obstacles
    nlohmann::json obstacle_data = {
        {"id", 3001},
        {"operation", "INSERT"},
        {"aabb", {{1.0f, 1.0f}, {2.0f, 2.0f}}}
    };
    publishObstacleJson(obstacle_data);

    // Publish order
    nlohmann::json order_data = {
        {"nodes", {
            {{"nodePosition", {{"x", 1.5}, {"y", 1.5}}}}
        }}
    };
    publishOrderJson(order_data);

    // Publish validation
    nlohmann::json validation_data = {
        {"id", 3002},
        {"aabb", {{0.5f, 0.5f}, {2.5f, 2.5f}}}
    };
    publishValidationJson(validation_data);

    // Publish TF
    publishTransform("integration_map", "test_robot_integration_base_link", 1.75, 1.75, 0.0);

    // Wait for all messages to be processed
    for (int i = 0; i < 10; ++i) {
        spinAndWait();
    }

    // Verify all message types were processed
    EXPECT_GE(received_obstacles_.size(), 1);
    EXPECT_GE(received_paths_.size(), 1);
    EXPECT_GE(received_validation_.size(), 1);
    
    // Check that obstacle and validation reference the same area
    if (!received_obstacles_.empty() && !received_validation_.empty()) {
        const auto& obstacle = received_obstacles_[0].updates[0].obstacle;
        const auto& validation = received_validation_[0].obstacles[0];
        
        // Both should be in roughly the same area
        EXPECT_NEAR(obstacle.position.x, validation.position.x, 1.0);
        EXPECT_NEAR(obstacle.position.y, validation.position.y, 1.0);
    }
}

// Test error recovery and resilience
TEST_F(IntegrationTest, ErrorRecoveryWorkflow) {
    // Send some invalid messages followed by valid ones
    
    // Invalid obstacle message
    publishObstacleJson(nlohmann::json::parse("{\"invalid\": \"data\"}"));
    
    // Valid obstacle message
    nlohmann::json valid_obstacle = {
        {"id", 4001},
        {"operation", "INSERT"},
        {"aabb", {{0.0f, 0.0f}, {1.0f, 1.0f}}}
    };
    publishObstacleJson(valid_obstacle);
    
    // Invalid order message
    publishOrderJson(nlohmann::json::parse("{\"not_nodes\": []}"));
    
    // Valid order message
    nlohmann::json valid_order = {
        {"nodes", {
            {{"nodePosition", {{"x", 0.5}, {"y", 0.5}}}}
        }}
    };
    publishOrderJson(valid_order);

    // System should recover and process valid messages
    EXPECT_GE(received_obstacles_.size(), 1);
    EXPECT_GE(received_paths_.size(), 1);
    
    // Check that valid messages were processed correctly
    if (!received_obstacles_.empty()) {
        EXPECT_EQ(received_obstacles_.back().updates[0].obstacle.id, 4001);
    }
    
    if (!received_paths_.empty()) {
        EXPECT_DOUBLE_EQ(received_paths_.back().poses[0].pose.position.x, 0.5);
    }
}

// Test high-frequency message processing
TEST_F(IntegrationTest, HighFrequencyProcessing) {
    const int num_messages = 50;
    
    received_obstacles_.clear();
    
    // Send many obstacle messages rapidly
    for (int i = 0; i < num_messages; ++i) {
        nlohmann::json obstacle_data = {
            {"id", 5000 + i},
            {"operation", i % 2 == 0 ? "INSERT" : "DELETE"},
            {"aabb", {{static_cast<float>(i), static_cast<float>(i)}, 
                     {static_cast<float>(i+1), static_cast<float>(i+1)}}}
        };
        
        publishObstacleJson(obstacle_data);
        
        // Small delay to prevent overwhelming the system
        if (i % 10 == 0) {
            spinAndWait();
        }
    }
    
    // Final spin to catch any remaining messages
    for (int i = 0; i < 10; ++i) {
        spinAndWait();
    }
    
    // Should process all or most messages without losing data
    EXPECT_GE(received_obstacles_.size(), num_messages / 2); // At least half should be processed
    
    // Check that messages are in reasonable order
    if (received_obstacles_.size() >= 2) {
        auto first_id = received_obstacles_[0].updates[0].obstacle.id;
        auto last_id = received_obstacles_.back().updates[0].obstacle.id;
        EXPECT_LT(first_id, last_id); // IDs should generally increase
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}