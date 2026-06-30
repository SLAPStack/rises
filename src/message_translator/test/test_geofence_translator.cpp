#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>

#include "message_translator/message_translator_node.hpp"

class TestMessageTranslatorNode : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create node options with test parameters
        rclcpp::NodeOptions options;
        
        // Set test parameters
        options.parameter_overrides({
            {"map_frame", "test_map"},
            {"target_frame", "test_base_link"},
            {"tf_prefix", "test_"},
            {"base_link_pub_rate", 10.0},
            {"mqtt.enabled", false}, // Disable MQTT for unit tests
            {"prefix_global_frame", false}
        });
        
        node_ = std::make_shared<slapstack::MessageTranslatorNode>(options);
        
        // Allow some time for initialization
        rclcpp::spin_some(node_);
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<slapstack::MessageTranslatorNode> node_;
};

// Test basic node initialization
TEST_F(TestMessageTranslatorNode, NodeInitialization) {
    ASSERT_NE(node_, nullptr);
    EXPECT_STREQ(node_->get_name(), "message_translator");
    
    // Check that parameters were set correctly
    EXPECT_EQ(node_->get_parameter("map_frame").as_string(), "test_map");
    EXPECT_EQ(node_->get_parameter("target_frame").as_string(), "test_base_link");
    EXPECT_EQ(node_->get_parameter("tf_prefix").as_string(), "test_");
    EXPECT_EQ(node_->get_parameter("base_link_pub_rate").as_double(), 10.0);
    EXPECT_FALSE(node_->get_parameter("mqtt.enabled").as_bool());
}

// Test JSON obstacle message processing
class JsonObstacleProcessingTest : public TestMessageTranslatorNode {
protected:
    void SetUp() override {
        TestMessageTranslatorNode::SetUp();
        
        // Create subscribers to capture published messages
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
    }

    void publishObstacleJson(const std::string& json_str) {
        auto publisher = node_->create_publisher<std_msgs::msg::String>("obstacle_json", 10);
        rclcpp::spin_some(node_); // Allow publisher to initialize
        
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json_str;
        publisher->publish(*msg);
        
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node_);
    }
    
    void publishValidationJson(const std::string& json_str) {
        auto publisher = node_->create_publisher<std_msgs::msg::String>("validation_mqtt", 10);
        rclcpp::spin_some(node_);
        
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json_str;
        publisher->publish(*msg);
        
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node_);
    }
    
    void publishOrderJson(const std::string& json_str) {
        auto publisher = node_->create_publisher<std_msgs::msg::String>("order", 10);
        rclcpp::spin_some(node_);
        
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json_str;
        publisher->publish(*msg);
        
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node_);
    }

    std::vector<rises_interfaces::msg::ObstacleUpdateArray> received_obstacles_;
    std::vector<rises_interfaces::msg::ObstacleArray> received_validation_;
    std::vector<nav_msgs::msg::Path> received_paths_;
    std::vector<rises_interfaces::msg::Contours> received_contours_;
    
private:
    rclcpp::Subscription<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<rises_interfaces::msg::ObstacleArray>::SharedPtr validation_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<rises_interfaces::msg::Contours>::SharedPtr contours_sub_;
};

// Test valid obstacle JSON processing
TEST_F(JsonObstacleProcessingTest, ValidObstacleJsonSingleObject) {
    nlohmann::json test_json = {
        {"id", 123},
        {"operation", "INSERT"},
        {"aabb", {{1.0f, 2.0f}, {3.0f, 4.0f}}}
    };
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    EXPECT_EQ(obstacle_msg.header.frame_id, "test_map");
    ASSERT_EQ(obstacle_msg.updates.size(), 1);
    
    const auto& update = obstacle_msg.updates[0];
    EXPECT_EQ(update.obstacle.id, 123);
    EXPECT_EQ(update.operation, rises_interfaces::msg::ObstacleUpdate::OP_INSERT);
    EXPECT_EQ(update.obstacle.type, rises_interfaces::msg::Obstacle::RECTANGLE);
    EXPECT_FLOAT_EQ(update.obstacle.position.x, 2.0f); // Center x
    EXPECT_FLOAT_EQ(update.obstacle.position.y, 3.0f); // Center y
    EXPECT_FLOAT_EQ(update.obstacle.width, 2.0f);  // |3.0 - 1.0|
    EXPECT_FLOAT_EQ(update.obstacle.height, 2.0f); // |4.0 - 2.0|
}

// Test obstacle JSON with pallets wrapper
TEST_F(JsonObstacleProcessingTest, ValidObstacleJsonWithPallets) {
    nlohmann::json test_json = {
        {"pallets", {
            {
                {"id", 1},
                {"operation", "INSERT"},
                {"aabb", {{0.0f, 0.0f}, {1.0f, 1.0f}}}
            },
            {
                {"id", 2},
                {"operation", "DELETE"},
                {"aabb", {{2.0f, 2.0f}, {3.0f, 3.0f}}}
            }
        }}
    };
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    ASSERT_EQ(obstacle_msg.updates.size(), 2);
    
    // Check first obstacle
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.id, 1);
    EXPECT_EQ(obstacle_msg.updates[0].operation, rises_interfaces::msg::ObstacleUpdate::OP_INSERT);
    
    // Check second obstacle
    EXPECT_EQ(obstacle_msg.updates[1].obstacle.id, 2);
    EXPECT_EQ(obstacle_msg.updates[1].operation, rises_interfaces::msg::ObstacleUpdate::OP_DELETE);
}

// Test obstacle JSON array format
TEST_F(JsonObstacleProcessingTest, ValidObstacleJsonArray) {
    nlohmann::json test_json = nlohmann::json::array({
        {
            {"id", 100},
            {"operation", "INSERT"},
            {"aabb", {{-1.0f, -1.0f}, {1.0f, 1.0f}}}
        }
    });
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    ASSERT_EQ(obstacle_msg.updates.size(), 1);
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.id, 100);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.position.x, 0.0f);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.position.y, 0.0f);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.width, 2.0f);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.height, 2.0f);
}

// Edge case: Empty message
TEST_F(JsonObstacleProcessingTest, EmptyMessage) {
    publishObstacleJson("");
    EXPECT_EQ(received_obstacles_.size(), 0);
}

// Edge case: Invalid JSON
TEST_F(JsonObstacleProcessingTest, InvalidJson) {
    publishObstacleJson("{invalid json}");
    EXPECT_EQ(received_obstacles_.size(), 0);
}

// Edge case: Missing required fields
TEST_F(JsonObstacleProcessingTest, MissingRequiredFields) {
    nlohmann::json test_json = {
        {"id", 123}
        // Missing "operation" and "aabb"
    };
    
    publishObstacleJson(test_json.dump());
    EXPECT_EQ(received_obstacles_.size(), 0);
}

// Edge case: Invalid operation
TEST_F(JsonObstacleProcessingTest, InvalidOperation) {
    nlohmann::json test_json = {
        {"id", 123},
        {"operation", "INVALID_OP"},
        {"aabb", {{1.0f, 2.0f}, {3.0f, 4.0f}}}
    };
    
    publishObstacleJson(test_json.dump());
    EXPECT_EQ(received_obstacles_.size(), 0);
}

// Edge case: Invalid AABB format
TEST_F(JsonObstacleProcessingTest, InvalidAABBFormat) {
    nlohmann::json test_json = {
        {"id", 123},
        {"operation", "INSERT"},
        {"aabb", {{1.0f}}} // Invalid: only one coordinate
    };
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    ASSERT_EQ(obstacle_msg.updates.size(), 1);
    // Should create circle obstacle as fallback
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.type, rises_interfaces::msg::Obstacle::CIRCLE);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.radius, 0.01f);
}

// Edge case: Zero-size AABB
TEST_F(JsonObstacleProcessingTest, ZeroSizeAABB) {
    nlohmann::json test_json = {
        {"id", 123},
        {"operation", "INSERT"},
        {"aabb", {{1.0f, 1.0f}, {1.0f, 1.0f}}} // Zero width and height
    };
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    ASSERT_EQ(obstacle_msg.updates.size(), 1);
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.type, rises_interfaces::msg::Obstacle::RECTANGLE);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.width, 0.0f);
    EXPECT_FLOAT_EQ(obstacle_msg.updates[0].obstacle.height, 0.0f);
}

// Test validation message processing
TEST_F(JsonObstacleProcessingTest, ValidValidationMessage) {
    nlohmann::json test_json = {
        {"id", 456},
        {"aabb", {{-2.0f, -3.0f}, {2.0f, 3.0f}}}
    };
    
    publishValidationJson(test_json.dump());
    
    ASSERT_EQ(received_validation_.size(), 1);
    const auto& validation_msg = received_validation_[0];
    
    EXPECT_EQ(validation_msg.header.frame_id, "test_map");
    ASSERT_EQ(validation_msg.obstacles.size(), 1);
    
    const auto& obstacle = validation_msg.obstacles[0];
    EXPECT_EQ(obstacle.id, 456);
    EXPECT_EQ(obstacle.type, rises_interfaces::msg::Obstacle::RECTANGLE);
    EXPECT_FLOAT_EQ(obstacle.position.x, 0.0f);
    EXPECT_FLOAT_EQ(obstacle.position.y, 0.0f);
    EXPECT_FLOAT_EQ(obstacle.width, 4.0f);
    EXPECT_FLOAT_EQ(obstacle.height, 6.0f);
}

// Edge case: Invalid validation message
TEST_F(JsonObstacleProcessingTest, InvalidValidationMessage) {
    nlohmann::json test_json = {
        {"id", 456}
        // Missing "aabb"
    };
    
    publishValidationJson(test_json.dump());
    EXPECT_EQ(received_validation_.size(), 0);
}

// Test order/path processing
TEST_F(JsonObstacleProcessingTest, ValidOrderMessage) {
    nlohmann::json test_json = {
        {"nodes", {
            {
                {"nodePosition", {
                    {"x", 1.0},
                    {"y", 2.0},
                    {"theta", 45.0}
                }}
            },
            {
                {"nodePosition", {
                    {"x", 3.0},
                    {"y", 4.0},
                    {"theta", 90.0}
                }}
            }
        }}
    };
    
    publishOrderJson(test_json.dump());
    
    ASSERT_EQ(received_paths_.size(), 1);
    const auto& path_msg = received_paths_[0];
    
    EXPECT_EQ(path_msg.header.frame_id, "test_map");
    ASSERT_EQ(path_msg.poses.size(), 2);
    
    // Check first pose
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.position.z, 0.0);
    
    // Check orientation (45 degrees = π/4 radians)
    double expected_z = std::sin(M_PI / 8);  // 45°/2 in radians
    double expected_w = std::cos(M_PI / 8);
    EXPECT_NEAR(path_msg.poses[0].pose.orientation.z, expected_z, 1e-6);
    EXPECT_NEAR(path_msg.poses[0].pose.orientation.w, expected_w, 1e-6);
    
    // Check second pose
    EXPECT_DOUBLE_EQ(path_msg.poses[1].pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[1].pose.position.y, 4.0);
}

// Edge case: Order without theta
TEST_F(JsonObstacleProcessingTest, OrderWithoutTheta) {
    nlohmann::json test_json = {
        {"nodes", {
            {
                {"nodePosition", {
                    {"x", 1.0},
                    {"y", 2.0}
                    // No theta
                }}
            }
        }}
    };
    
    publishOrderJson(test_json.dump());
    
    ASSERT_EQ(received_paths_.size(), 1);
    const auto& path_msg = received_paths_[0];
    
    ASSERT_EQ(path_msg.poses.size(), 1);
    // Orientation should be identity (no rotation)
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.orientation.w, 1.0);
}

// Edge case: Order with null theta
TEST_F(JsonObstacleProcessingTest, OrderWithNullTheta) {
    nlohmann::json test_json = {
        {"nodes", {
            {
                {"nodePosition", {
                    {"x", 1.0},
                    {"y", 2.0},
                    {"theta", nullptr}
                }}
            }
        }}
    };
    
    publishOrderJson(test_json.dump());
    
    ASSERT_EQ(received_paths_.size(), 1);
    const auto& path_msg = received_paths_[0];
    
    ASSERT_EQ(path_msg.poses.size(), 1);
    // Should handle null theta gracefully
    EXPECT_DOUBLE_EQ(path_msg.poses[0].pose.orientation.w, 1.0);
}

// Edge case: Order missing nodes
TEST_F(JsonObstacleProcessingTest, OrderMissingNodes) {
    nlohmann::json test_json = {
        {"not_nodes", "invalid"}
    };
    
    publishOrderJson(test_json.dump());
    EXPECT_EQ(received_paths_.size(), 0);
}

// Edge case: Order with invalid node structure
TEST_F(JsonObstacleProcessingTest, OrderInvalidNodeStructure) {
    nlohmann::json test_json = {
        {"nodes", {
            {
                {"nodePosition", {
                    {"x", "invalid_string"},
                    {"y", 2.0}
                }}
            }
        }}
    };
    
    publishOrderJson(test_json.dump());
    
    // Should not publish a path when no valid nodes are present
    EXPECT_EQ(received_paths_.size(), 0);
}

// Test extreme coordinate values
TEST_F(JsonObstacleProcessingTest, ExtremeCoordinateValues) {
    nlohmann::json test_json = {
        {"id", 999},
        {"operation", "INSERT"},
        {"aabb", {{-1000000.0f, -1000000.0f}, {1000000.0f, 1000000.0f}}}
    };
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    ASSERT_EQ(obstacle_msg.updates.size(), 1);
    const auto& update = obstacle_msg.updates[0];
    
    EXPECT_FLOAT_EQ(update.obstacle.position.x, 0.0f);  // Center
    EXPECT_FLOAT_EQ(update.obstacle.position.y, 0.0f);  // Center
    EXPECT_FLOAT_EQ(update.obstacle.width, 2000000.0f);
    EXPECT_FLOAT_EQ(update.obstacle.height, 2000000.0f);
}

// Test very large JSON messages
TEST_F(JsonObstacleProcessingTest, LargeJsonMessage) {
    nlohmann::json test_json = nlohmann::json::array();
    
    // Create 1000 obstacles
    for (int i = 0; i < 1000; ++i) {
        test_json.push_back({
            {"id", i},
            {"operation", "INSERT"},
            {"aabb", {{static_cast<float>(i), static_cast<float>(i)}, 
                     {static_cast<float>(i+1), static_cast<float>(i+1)}}}
        });
    }
    
    publishObstacleJson(test_json.dump());
    
    ASSERT_EQ(received_obstacles_.size(), 1);
    const auto& obstacle_msg = received_obstacles_[0];
    
    EXPECT_EQ(obstacle_msg.updates.size(), 1000);
    
    // Spot check a few obstacles
    EXPECT_EQ(obstacle_msg.updates[0].obstacle.id, 0);
    EXPECT_EQ(obstacle_msg.updates[999].obstacle.id, 999);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}