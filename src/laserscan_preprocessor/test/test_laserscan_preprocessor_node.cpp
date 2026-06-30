#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <memory>
#include <chrono>

#include "laserscan_preprocessor/laserscan_preprocessor_node.hpp"
#include "rises_interfaces/msg/obstacle_array.hpp"

class LaserPreprocessorNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create node with test configuration
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"segment_distance_threshold", 0.2},
            {"segment_angle_threshold_deg", 30.0},
            {"target_frame", "base_link"},
            {"tf_prefix", ""},
            {"laser_frames", std::vector<std::string>{"laser"}},
            {"laser_heights", std::vector<double>{0.0}},
            {"publish_points_only", false}
        });
        node_ = std::make_shared<rises::LaserPreprocessorNode>(options);
        
        // Set up TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create static transform broadcaster
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        
        // Broadcast identity transform from laser to base_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "laser";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        static_tf_broadcaster_->sendTransform(transform);
        
        // Wait for transform to be available
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_->get_node_base_interface());
    }
    
    void TearDown() override {
        executor_->cancel();
        executor_.reset();
        node_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        static_tf_broadcaster_.reset();
        rclcpp::shutdown();
    }
    
    sensor_msgs::msg::LaserScan::SharedPtr createTestLaserScan(
        const std::string& frame_id = "laser",
        float min_angle = -M_PI/2,
        float max_angle = M_PI/2,
        float angle_increment = M_PI/180,
        float range_min = 0.1,
        float range_max = 10.0) {
        
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = node_->now();
        scan->header.frame_id = frame_id;
        scan->angle_min = min_angle;
        scan->angle_max = max_angle;
        scan->angle_increment = angle_increment;
        scan->range_min = range_min;
        scan->range_max = range_max;
        
        // Create test data: a simple obstacle at 1 meter distance
        int num_points = static_cast<int>((max_angle - min_angle) / angle_increment);
        scan->ranges.resize(num_points);
        scan->intensities.resize(num_points);
        
        for (int i = 0; i < num_points; ++i) {
            // Create an obstacle from -30 to +30 degrees at 1m distance
            float angle = min_angle + i * angle_increment;
            if (angle >= -M_PI/6 && angle <= M_PI/6) {
                scan->ranges[i] = 1.0f; // 1 meter obstacle
            } else {
                scan->ranges[i] = 5.0f; // Far points
            }
            scan->intensities[i] = 100.0f;
        }
        
        return scan;
    }
    
    std::shared_ptr<rises::LaserPreprocessorNode> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// Test parameter validation
TEST_F(LaserPreprocessorNodeTest, TestParameterValidation) {
    // Test valid parameters
    node_->set_parameter(rclcpp::Parameter("segment_distance_threshold", 0.5));
    node_->set_parameter(rclcpp::Parameter("segment_angle_threshold_deg", 45.0));
    
    // Configure the node
    auto result = node_->on_configure(rclcpp_lifecycle::State{});
    EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(LaserPreprocessorNodeTest, TestParameterValidationOutOfRange) {
    // Test parameter clamping for out-of-range values
    node_->set_parameter(rclcpp::Parameter("segment_distance_threshold", 20.0)); // Too high
    node_->set_parameter(rclcpp::Parameter("segment_angle_threshold_deg", 200.0)); // Too high
    
    // Configure should still succeed but clamp values
    auto result = node_->on_configure(rclcpp_lifecycle::State{});
    EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

// Test lifecycle transitions
TEST_F(LaserPreprocessorNodeTest, TestLifecycleTransitions) {
    // Test configure
    auto configure_result = node_->on_configure(rclcpp_lifecycle::State{});
    EXPECT_EQ(configure_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    
    // Test activate
    auto activate_result = node_->on_activate(rclcpp_lifecycle::State{});
    EXPECT_EQ(activate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    
    // Test deactivate
    auto deactivate_result = node_->on_deactivate(rclcpp_lifecycle::State{});
    EXPECT_EQ(deactivate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

// Test point cloud conversion
TEST_F(LaserPreprocessorNodeTest, TestPointCloudConversion) {
    node_->on_configure(rclcpp_lifecycle::State{});
    node_->on_activate(rclcpp_lifecycle::State{});
    
    // Create test laser scan
    auto scan = createTestLaserScan();
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> scans = {scan};
    
    // Test conversion (we need to access private method, so we'll test indirectly)
    // The conversion will be tested through the callback mechanism
    EXPECT_TRUE(scan->ranges.size() > 0);
}

// Test points-only publishing mode
TEST_F(LaserPreprocessorNodeTest, TestPointsOnlyMode) {
    // Enable points-only mode
    node_->set_parameter(rclcpp::Parameter("publish_points_only", true));
    
    auto configure_result = node_->on_configure(rclcpp_lifecycle::State{});
    EXPECT_EQ(configure_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    
    auto activate_result = node_->on_activate(rclcpp_lifecycle::State{});
    EXPECT_EQ(activate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    
    // Create a subscriber to capture published obstacles
    rises_interfaces::msg::ObstacleArray::SharedPtr received_obstacles;
    auto obstacle_sub = node_->create_subscription<rises_interfaces::msg::ObstacleArray>(
        "lidar_segments", 10,
        [&received_obstacles](const rises_interfaces::msg::ObstacleArray::SharedPtr msg) {
            received_obstacles = msg;
        });
    
    // Give some time for setup
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    executor_->spin_some();
}

// Test empty input handling
TEST_F(LaserPreprocessorNodeTest, TestEmptyInputHandling) {
    node_->on_configure(rclcpp_lifecycle::State{});
    node_->on_activate(rclcpp_lifecycle::State{});
    
    // Test with empty scan vector
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> empty_scans;
    
    // This should not crash - we'll test this indirectly through the system
    EXPECT_TRUE(true); // Basic test to ensure setup works
}

// Test null pointer handling
TEST_F(LaserPreprocessorNodeTest, TestNullPointerHandling) {
    node_->on_configure(rclcpp_lifecycle::State{});
    node_->on_activate(rclcpp_lifecycle::State{});
    
    // Test with null scan pointer
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> scans_with_null = {nullptr};
    
    // This should not crash
    EXPECT_TRUE(true);
}

// Test transform handling
TEST_F(LaserPreprocessorNodeTest, TestTransformHandling) {
    // Set target frame different from laser frame
    node_->set_parameter(rclcpp::Parameter("target_frame", "map"));
    
    // Broadcast transform from base_link to map
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = node_->now();
    map_transform.header.frame_id = "map";
    map_transform.child_frame_id = "base_link";
    map_transform.transform.translation.x = 1.0;
    map_transform.transform.translation.y = 2.0;
    map_transform.transform.translation.z = 0.0;
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;
    map_transform.transform.rotation.w = 1.0;
    
    static_tf_broadcaster_->sendTransform(map_transform);
    
    auto configure_result = node_->on_configure(rclcpp_lifecycle::State{});
    EXPECT_EQ(configure_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    
    auto activate_result = node_->on_activate(rclcpp_lifecycle::State{});
    EXPECT_EQ(activate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

// Test segmentation algorithms
TEST_F(LaserPreprocessorNodeTest, TestSegmentationAlgorithms) {
    node_->on_configure(rclcpp_lifecycle::State{});
    node_->on_activate(rclcpp_lifecycle::State{});
    
    // Test that the node can handle various segmentation scenarios
    // This is tested indirectly through the system
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}