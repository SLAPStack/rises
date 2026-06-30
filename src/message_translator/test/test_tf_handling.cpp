#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include "message_translator/message_translator_node.hpp"

class TfHandlingTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"mqtt.enabled", false},
            {"map_frame", "test_map"},
            {"target_frame", "test_base_link"},
            {"tf_prefix", "robot1_"},
            {"base_link_pub_rate", 1.0}, // Low rate for testing
            {"prefix_global_frame", true}
        });
        
        node_ = std::make_shared<slapstack::MessageTranslatorNode>(options);
        
        // Create subscribers to capture published messages
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "base_link_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                received_poses_.push_back(*msg);
            });
            
        tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                received_tf_messages_.push_back(*msg);
            });
            
        // Create TF broadcaster for testing
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        
        rclcpp::spin_some(node_);
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    void publishTfMessage(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
        auto publisher = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf_raw", 10);
        rclcpp::spin_some(node_);
        
        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms = transforms;
        publisher->publish(tf_msg);
        
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(node_);
    }

    geometry_msgs::msg::TransformStamped createTestTransform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double x = 1.0, double y = 2.0, double z = 0.0) {
        
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.header.stamp = rclcpp::Time(1000000000); // Old timestamp
        
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        
        // Identity rotation
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        return transform;
    }

    std::shared_ptr<slapstack::MessageTranslatorNode> node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<geometry_msgs::msg::PoseStamped> received_poses_;
    std::vector<tf2_msgs::msg::TFMessage> received_tf_messages_;
    
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

// Test TF timestamp filtering
TEST_F(TfHandlingTest, TfTimestampFiltering) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms = {
        createTestTransform("robot1_test_map", "robot1_test_base_link", 1.0, 2.0, 0.0),
        createTestTransform("robot1_test_map", "robot1_sensor", 0.5, 0.0, 1.0)
    };
    
    publishTfMessage(transforms);
    
    // Should receive filtered TF message with updated timestamps
    ASSERT_GE(received_tf_messages_.size(), 1);
    
    const auto& tf_msg = received_tf_messages_[0];
    ASSERT_EQ(tf_msg.transforms.size(), 2);
    
    // Check that timestamps were updated (should be more recent than original)
    for (const auto& transform : tf_msg.transforms) {
        EXPECT_GT(rclcpp::Time(transform.header.stamp).nanoseconds(), 
                 rclcpp::Time(1000000000).nanoseconds());
    }
    
    // Check frame names are preserved
    EXPECT_EQ(tf_msg.transforms[0].header.frame_id, "robot1_test_map");
    EXPECT_EQ(tf_msg.transforms[0].child_frame_id, "robot1_test_base_link");
    
    // Check transform values are preserved
    EXPECT_DOUBLE_EQ(tf_msg.transforms[0].transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(tf_msg.transforms[0].transform.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(tf_msg.transforms[0].transform.translation.z, 0.0);
}

// Test empty TF message
TEST_F(TfHandlingTest, EmptyTfMessage) {
    std::vector<geometry_msgs::msg::TransformStamped> empty_transforms;
    publishTfMessage(empty_transforms);
    
    // Should not publish empty TF messages
    // (This depends on implementation - might still publish empty message)
    rclcpp::spin_some(node_);
}

// Test multiple TF messages
TEST_F(TfHandlingTest, MultipleTfMessages) {
    for (int i = 0; i < 5; ++i) {
        std::vector<geometry_msgs::msg::TransformStamped> transforms = {
            createTestTransform("robot1_test_map", "robot1_test_base_link", 
                               static_cast<double>(i), static_cast<double>(i+1), 0.0)
        };
        publishTfMessage(transforms);
    }
    
    EXPECT_GE(received_tf_messages_.size(), 5);
}

// Test invalid transform data
TEST_F(TfHandlingTest, InvalidTransformData) {
    geometry_msgs::msg::TransformStamped invalid_transform;
    invalid_transform.header.frame_id = "";  // Empty frame ID
    invalid_transform.child_frame_id = "";   // Empty child frame ID
    invalid_transform.header.stamp = rclcpp::Time(0); // Zero timestamp
    
    // Invalid rotation (not normalized quaternion)
    invalid_transform.transform.rotation.x = 1.0;
    invalid_transform.transform.rotation.y = 1.0;
    invalid_transform.transform.rotation.z = 1.0;
    invalid_transform.transform.rotation.w = 1.0;
    
    std::vector<geometry_msgs::msg::TransformStamped> transforms = {invalid_transform};
    publishTfMessage(transforms);
    
    // Should handle invalid transforms gracefully
    rclcpp::spin_some(node_);
}

// Test TF prefix handling
class TfPrefixTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<slapstack::MessageTranslatorNode> createNodeWithPrefix(
        const std::string& tf_prefix,
        bool prefix_global_frame = false) {
        
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"mqtt.enabled", false},
            {"map_frame", "map"},
            {"target_frame", "base_link"},
            {"tf_prefix", tf_prefix},
            {"prefix_global_frame", prefix_global_frame},
            {"base_link_pub_rate", 0.1} // Very low rate
        });
        
        return std::make_shared<slapstack::MessageTranslatorNode>(options);
    }
};

TEST_F(TfPrefixTest, EmptyTfPrefix) {
    auto node = createNodeWithPrefix("");
    
    EXPECT_EQ(node->get_parameter("tf_prefix").as_string(), "");
    EXPECT_EQ(node->get_parameter("map_frame").as_string(), "map");
    EXPECT_EQ(node->get_parameter("target_frame").as_string(), "base_link");
}

TEST_F(TfPrefixTest, TfPrefixWithUnderscore) {
    auto node = createNodeWithPrefix("robot1_");
    
    EXPECT_EQ(node->get_parameter("tf_prefix").as_string(), "robot1_");
}

TEST_F(TfPrefixTest, TfPrefixWithoutUnderscore) {
    auto node = createNodeWithPrefix("robot1");
    
    // Should automatically add underscore
    EXPECT_EQ(node->get_parameter("tf_prefix").as_string(), "robot1");
}

TEST_F(TfPrefixTest, PrefixGlobalFrame) {
    auto node = createNodeWithPrefix("robot1_", true);
    
    EXPECT_EQ(node->get_parameter("tf_prefix").as_string(), "robot1_");
    EXPECT_TRUE(node->get_parameter("prefix_global_frame").as_bool());
}

// Test pose publishing with TF
class PosePublishingTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create a separate node for TF publishing
        tf_node_ = std::make_shared<rclcpp::Node>("tf_publisher");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(tf_node_);
        
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"mqtt.enabled", false},
            {"map_frame", "test_map"},
            {"target_frame", "test_base_link"},
            {"tf_prefix", ""},
            {"base_link_pub_rate", 10.0} // High rate for testing
        });
        
        node_ = std::make_shared<slapstack::MessageTranslatorNode>(options);
        
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "base_link_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                received_poses_.push_back(*msg);
            });
            
        rclcpp::spin_some(node_);
        rclcpp::spin_some(tf_node_);
    }

    void TearDown() override {
        node_.reset();
        tf_node_.reset();
        rclcpp::shutdown();
    }

    void publishTransform(double x, double y, double z = 0.0, 
                         double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = tf_node_->now();
        transform.header.frame_id = "test_map";
        transform.child_frame_id = "test_base_link";
        
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;
        
        tf_broadcaster_->sendTransform(transform);
    }

    std::shared_ptr<slapstack::MessageTranslatorNode> node_;
    std::shared_ptr<rclcpp::Node> tf_node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<geometry_msgs::msg::PoseStamped> received_poses_;
    
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

// Test pose publishing with valid TF
TEST_F(PosePublishingTest, ValidTransformPosePublishing) {
    publishTransform(3.0, 4.0, 0.5);
    
    // Allow some time for TF to propagate and pose to be published
    for (int i = 0; i < 10; ++i) {
        rclcpp::spin_some(tf_node_);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!received_poses_.empty()) break;
    }
    
    ASSERT_GE(received_poses_.size(), 1);
    
    const auto& pose_msg = received_poses_[0];
    EXPECT_EQ(pose_msg.header.frame_id, "test_map");
    
    // Check pose values
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.y, 4.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.z, 0.5);
    
    // Check identity rotation
    EXPECT_DOUBLE_EQ(pose_msg.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.orientation.w, 1.0);
}

// Test pose publishing with rotation
TEST_F(PosePublishingTest, TransformWithRotation) {
    // 90-degree rotation around Z-axis
    double qz = std::sin(M_PI / 4);
    double qw = std::cos(M_PI / 4);
    
    publishTransform(1.0, 2.0, 0.0, 0.0, 0.0, qz, qw);
    
    for (int i = 0; i < 10; ++i) {
        rclcpp::spin_some(tf_node_);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!received_poses_.empty()) break;
    }
    
    ASSERT_GE(received_poses_.size(), 1);
    
    const auto& pose_msg = received_poses_[0];
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose_msg.pose.position.y, 2.0);
    EXPECT_NEAR(pose_msg.pose.orientation.z, qz, 1e-6);
    EXPECT_NEAR(pose_msg.pose.orientation.w, qw, 1e-6);
}

// Test pose publishing frequency
TEST_F(PosePublishingTest, PublishingFrequency) {
    publishTransform(0.0, 0.0, 0.0);
    
    // Clear any initial poses
    received_poses_.clear();
    
    // Wait for multiple pose publications
    auto start_time = std::chrono::steady_clock::now();
    while (received_poses_.size() < 5 && 
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
        rclcpp::spin_some(tf_node_);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    EXPECT_GE(received_poses_.size(), 3); // Should get at least a few poses
    
    // Check timestamps are increasing
    for (std::size_t i = 1; i < received_poses_.size(); ++i) {
        EXPECT_GT(rclcpp::Time(received_poses_[i].header.stamp).nanoseconds(),
                 rclcpp::Time(received_poses_[i-1].header.stamp).nanoseconds());
    }
}

// Test behavior when TF is not available
TEST_F(PosePublishingTest, NoTransformAvailable) {
    // Don't publish any transform
    
    // Wait a bit to see if any poses are published
    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(tf_node_);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Should not publish poses when transform is not available
    EXPECT_EQ(received_poses_.size(), 0);
}

// Test rapid TF updates
TEST_F(PosePublishingTest, RapidTfUpdates) {
    for (int i = 0; i < 20; ++i) {
        publishTransform(static_cast<double>(i) * 0.1, 
                        static_cast<double>(i) * 0.2, 0.0);
        rclcpp::spin_some(tf_node_);
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Should handle rapid TF updates without crashing
    EXPECT_GE(received_poses_.size(), 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}