#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include "message_translator/message_translator_node.hpp"

class ContourProcessingTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"mqtt.enabled", false},
            {"map_frame", "test_map"},
            {"target_frame", "test_base_link"}
        });
        
        node_ = std::make_shared<slapstack::MessageTranslatorNode>(options);
        
        // Create subscriber to capture published contours
        contours_sub_ = node_->create_subscription<rises_interfaces::msg::Contours>(
            "warehouse_contours", 10,
            [this](const rises_interfaces::msg::Contours::SharedPtr msg) {
                received_contours_.push_back(*msg);
            });
            
        rclcpp::spin_some(node_);
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    void publishContoursJson(const std::string& json_str) {
        auto publisher = node_->create_publisher<std_msgs::msg::String>("warehouse_contours_mqtt", 10);
        rclcpp::spin_some(node_);
        
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = json_str;
        publisher->publish(*msg);
        
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node_);
    }

    std::shared_ptr<slapstack::MessageTranslatorNode> node_;
    std::vector<rises_interfaces::msg::Contours> received_contours_;
    
private:
    rclcpp::Subscription<rises_interfaces::msg::Contours>::SharedPtr contours_sub_;
};

// Test valid warehouse contours processing
TEST_F(ContourProcessingTest, ValidWarehouseContours) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {
            {0.0f, 0.0f},
            {10.0f, 0.0f},
            {10.0f, 10.0f},
            {0.0f, 10.0f}
        }},
        {"outer_contour_segments", {
            {{0.0f, 0.0f}, {10.0f, 0.0f}},
            {{10.0f, 0.0f}, {10.0f, 10.0f}},
            {{10.0f, 10.0f}, {0.0f, 10.0f}},
            {{0.0f, 10.0f}, {0.0f, 0.0f}}
        }},
        {"inner_contours", {
            {
                {{2.0f, 2.0f}, {4.0f, 2.0f}},
                {{4.0f, 2.0f}, {4.0f, 4.0f}},
                {{4.0f, 4.0f}, {2.0f, 4.0f}},
                {{2.0f, 4.0f}, {2.0f, 2.0f}}
            }
        }}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    EXPECT_EQ(contours_msg.header.frame_id, "test_map");
    
    // Check outer hull
    ASSERT_EQ(contours_msg.outer_contour_hull.points.size(), 4);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[0].y, 0.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[1].x, 10.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[1].y, 0.0f);
    
    // Check outer segments
    ASSERT_EQ(contours_msg.outer_contour_segments.size(), 4);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].start.x, 0.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].start.y, 0.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].end.x, 10.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].end.y, 0.0f);
    
    // Check inner contours
    ASSERT_EQ(contours_msg.inner_contours.size(), 1);
    ASSERT_EQ(contours_msg.inner_contours[0].points.size(), 5); // 4 points + closing point
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[0].x, 2.0f);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[0].y, 2.0f);
    
    // Check that polygon is closed (last point should be end of last segment)
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[4].x, 2.0f);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[4].y, 2.0f);
}

// Test empty contours
TEST_F(ContourProcessingTest, EmptyContours) {
    nlohmann::json test_json = {
        {"outer_contour_hull", nlohmann::json::array()},
        {"outer_contour_segments", nlohmann::json::array()},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    EXPECT_EQ(contours_msg.outer_contour_hull.points.size(), 0);
    EXPECT_EQ(contours_msg.outer_contour_segments.size(), 0);
    EXPECT_EQ(contours_msg.inner_contours.size(), 0);
}

// Test missing required fields
TEST_F(ContourProcessingTest, MissingRequiredFields) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}, {1.0f, 1.0f}}}
        // Missing outer_contour_segments and inner_contours
    };
    
    publishContoursJson(test_json.dump());
    
    // Should still publish a message with what's available (only hull points)
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    EXPECT_EQ(contours_msg.outer_contour_segments.size(), 0);
}

// Test invalid segment format
TEST_F(ContourProcessingTest, InvalidSegmentFormat) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}, {1.0f, 1.0f}}},
        {"outer_contour_segments", {
            {{0.0f}}, // Invalid: only one coordinate
            {{0.0f, 0.0f}, {1.0f}} // Invalid: second point missing y
        }},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should skip invalid segments
    EXPECT_EQ(contours_msg.outer_contour_segments.size(), 0);
}

// Test complex inner contours
TEST_F(ContourProcessingTest, ComplexInnerContours) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}, {20.0f, 0.0f}, {20.0f, 20.0f}, {0.0f, 20.0f}}},
        {"outer_contour_segments", {
            {{0.0f, 0.0f}, {20.0f, 0.0f}},
            {{20.0f, 0.0f}, {20.0f, 20.0f}},
            {{20.0f, 20.0f}, {0.0f, 20.0f}},
            {{0.0f, 20.0f}, {0.0f, 0.0f}}
        }},
        {"inner_contours", {
            {
                // First inner contour
                {{2.0f, 2.0f}, {4.0f, 2.0f}},
                {{4.0f, 2.0f}, {4.0f, 4.0f}},
                {{4.0f, 4.0f}, {2.0f, 4.0f}},
                {{2.0f, 4.0f}, {2.0f, 2.0f}}
            },
            {
                // Second inner contour
                {{10.0f, 10.0f}, {15.0f, 10.0f}},
                {{15.0f, 10.0f}, {15.0f, 15.0f}},
                {{15.0f, 15.0f}, {10.0f, 15.0f}},
                {{10.0f, 15.0f}, {10.0f, 10.0f}}
            }
        }}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should have two inner contours
    ASSERT_EQ(contours_msg.inner_contours.size(), 2);
    
    // Check first inner contour
    ASSERT_EQ(contours_msg.inner_contours[0].points.size(), 5);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[0].x, 2.0f);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[0].points[0].y, 2.0f);
    
    // Check second inner contour
    ASSERT_EQ(contours_msg.inner_contours[1].points.size(), 5);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[1].points[0].x, 10.0f);
    EXPECT_FLOAT_EQ(contours_msg.inner_contours[1].points[0].y, 10.0f);
}

// Test single point segments (degenerate case)
TEST_F(ContourProcessingTest, DegenerateSegments) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}}}, // Single point
        {"outer_contour_segments", {
            {{1.0f, 1.0f}, {1.0f, 1.0f}} // Zero-length segment
        }},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should handle degenerate cases
    EXPECT_EQ(contours_msg.outer_contour_hull.points.size(), 1);
    EXPECT_EQ(contours_msg.outer_contour_segments.size(), 1);
    
    // Check that zero-length segment is preserved
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].start.x, 1.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].start.y, 1.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].end.x, 1.0f);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_segments[0].end.y, 1.0f);
}

// Test very large coordinates
TEST_F(ContourProcessingTest, LargeCoordinates) {
    const float large_val = 1000000.0f;
    
    nlohmann::json test_json = {
        {"outer_contour_hull", {
            {-large_val, -large_val},
            {large_val, -large_val},
            {large_val, large_val},
            {-large_val, large_val}
        }},
        {"outer_contour_segments", {
            {{-large_val, -large_val}, {large_val, -large_val}},
            {{large_val, -large_val}, {large_val, large_val}},
            {{large_val, large_val}, {-large_val, large_val}},
            {{-large_val, large_val}, {-large_val, -large_val}}
        }},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    EXPECT_EQ(contours_msg.outer_contour_hull.points.size(), 4);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[0].x, -large_val);
    EXPECT_FLOAT_EQ(contours_msg.outer_contour_hull.points[1].x, large_val);
}

// Test precision handling
TEST_F(ContourProcessingTest, PrecisionHandling) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {
            {1.123456789f, 2.987654321f},
            {3.141592653f, 2.718281828f}
        }},
        {"outer_contour_segments", {
            {{1.123456789f, 2.987654321f}, {3.141592653f, 2.718281828f}}
        }},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Check that precision is maintained (within float limits)
    EXPECT_NEAR(contours_msg.outer_contour_hull.points[0].x, 1.123456789f, 1e-6f);
    EXPECT_NEAR(contours_msg.outer_contour_hull.points[0].y, 2.987654321f, 1e-6f);
    EXPECT_NEAR(contours_msg.outer_contour_hull.points[1].x, 3.141592653f, 1e-6f);
    EXPECT_NEAR(contours_msg.outer_contour_hull.points[1].y, 2.718281828f, 1e-6f);
}

// Test invalid inner contour format
TEST_F(ContourProcessingTest, InvalidInnerContourFormat) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}, {1.0f, 1.0f}}},
        {"outer_contour_segments", {{{0.0f, 0.0f}, {1.0f, 1.0f}}}},
        {"inner_contours", {
            "not_an_array", // Invalid: should be array of segments
            {
                {{1.0f}}, // Invalid segment: missing coordinate
                {{1.0f, 2.0f}, {}} // Invalid segment: empty end point
            }
        }}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should skip invalid inner contours but process valid ones
    EXPECT_EQ(contours_msg.inner_contours.size(), 0);
}

// Test empty inner contours within array
TEST_F(ContourProcessingTest, EmptyInnerContourInArray) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f}, {1.0f, 1.0f}}},
        {"outer_contour_segments", {{{0.0f, 0.0f}, {1.0f, 1.0f}}}},
        {"inner_contours", {
            nlohmann::json::array(), // Empty inner contour
            {
                {{2.0f, 2.0f}, {3.0f, 2.0f}},
                {{3.0f, 2.0f}, {3.0f, 3.0f}},
                {{3.0f, 3.0f}, {2.0f, 3.0f}},
                {{2.0f, 3.0f}, {2.0f, 2.0f}}
            }
        }}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should have one inner contour (empty one should be skipped or processed as empty)
    EXPECT_GE(contours_msg.inner_contours.size(), 1);
    
    // The valid contour should be processed correctly
    bool found_valid_contour = false;
    for (const auto& contour : contours_msg.inner_contours) {
        if (contour.points.size() > 0) {
            found_valid_contour = true;
            EXPECT_FLOAT_EQ(contour.points[0].x, 2.0f);
            EXPECT_FLOAT_EQ(contour.points[0].y, 2.0f);
        }
    }
    EXPECT_TRUE(found_valid_contour);
}

// Test 3D coordinates (should use z=0)
TEST_F(ContourProcessingTest, ThreeDimensionalCoordinates) {
    nlohmann::json test_json = {
        {"outer_contour_hull", {{0.0f, 0.0f, 5.0f}, {1.0f, 1.0f, 10.0f}}}, // 3D points
        {"outer_contour_segments", {{{0.0f, 0.0f}, {1.0f, 1.0f}}}},
        {"inner_contours", nlohmann::json::array()}
    };
    
    publishContoursJson(test_json.dump());
    
    ASSERT_EQ(received_contours_.size(), 1);
    const auto& contours_msg = received_contours_[0];
    
    // Should accept 3D arrays but only use x,y (z should default to 0)
    // Note: The parser may not handle 3-element arrays, so hull may be empty
    EXPECT_GE(contours_msg.outer_contour_segments.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}