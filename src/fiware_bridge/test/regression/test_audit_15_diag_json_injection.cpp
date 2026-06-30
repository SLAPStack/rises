// Regression tests for audit finding #15 (diagnostic JSON injection) in the
// fiware_bridge diagnostics forwarder.
//
// These tests verify that when DiagnosticArray KeyValue pairs are forwarded as
// JSON, the reserved keys "level" and "message" cannot be overwritten by an
// attacker-supplied KeyValue, that over-cap key or value strings are dropped,
// and that control / non-printable bytes in keys are sanitised so the forwarded
// JSON never contains raw control bytes. A plain allowed key is forwarded
// unchanged.
//
// Detection mechanism (the "spy"): the bridge forwards every diagnostic into
// std_msgs::msg::String JSON on the ROS topic /fiware/system_health. The test
// spy is a plain rclcpp::Node subscriber on that topic. We feed
// DiagnosticArray messages on /diagnostics and parse the captured JSON with
// nlohmann::json. No HTTP / DDS-Enabler mock is required because the bridge
// is purely a ROS-topic translator (see fiware_bridge_node.hpp doc-comment).
//
// Status on develop:
//   - ReservedKeyLevelIsRejectedOrEscaped: RED on develop.
//   - ReservedKeyMessageIsRejectedOrEscaped: RED on develop.
//   - ArbitraryAllowedKeyForwarded: GREEN on develop and post-fix; positive
//     control.
//   - KeyLengthOverCapRejected: RED on develop.
//   - ValueLengthOverCapRejected: RED on develop.
//   - UnicodeKeyHandledSafely: RED on develop (raw control bytes survive the
//     JSON round-trip on the parsed key).

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiware_bridge/fiware_bridge_node.hpp"

namespace {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

constexpr std::chrono::milliseconds kSpinSlice{10};
constexpr int kMaxSpinIterations = 200;
constexpr double kFastThrottleHz = 100.0;
constexpr std::size_t kAttackPayloadBytes = 1024u * 1024u;    // 1 MiB.
constexpr std::size_t kMaxAcceptableJsonBytes = 256u * 1024u; // 256 KiB.

class DiagInjectionFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void buildBridge() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"diag_throttle_hz", kFastThrottleHz},
        {"report_throttle_hz", kFastThrottleHz},
        {"odom_throttle_hz", kFastThrottleHz},
        {"heatmap_throttle_hz", kFastThrottleHz},
        {"obstacles_json_file", std::string{}},
    });
    node_ = std::make_shared<rises::FiwareBridgeNode>(options);

    publisher_node_ = std::make_shared<rclcpp::Node>("test_diag_pub");
    diag_pub_ = publisher_node_->create_publisher<DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());

    spy_node_ = std::make_shared<rclcpp::Node>("test_diag_spy");
    captured_.clear();
    spy_sub_ = spy_node_->create_subscription<std_msgs::msg::String>(
        "fiware/system_health", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          captured_.push_back(msg->data);
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);
    executor_->add_node(spy_node_);
  }

  void TearDown() override {
    executor_.reset();
    spy_sub_.reset();
    diag_pub_.reset();
    spy_node_.reset();
    publisher_node_.reset();
    node_.reset();
    captured_.clear();
  }

  void spinFor(std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(kSpinSlice);
    }
  }

  bool spinUntilCaptured(int max_iterations = kMaxSpinIterations) {
    for (int i = 0; i < max_iterations; ++i) {
      executor_->spin_some();
      if (!captured_.empty()) {
        return true;
      }
      std::this_thread::sleep_for(kSpinSlice);
    }
    return false;
  }

  void publishDiag(const std::string &node_name, std::uint8_t level,
                   const std::string &message,
                   const std::vector<KeyValue> &values) {
    DiagnosticStatus status;
    status.name = node_name;
    status.level = level;
    status.message = message;
    status.hardware_id = "test";
    status.values = values;

    DiagnosticArray array;
    array.header.frame_id = "test";
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  static KeyValue kv(const std::string &key, const std::string &value) {
    KeyValue out;
    out.key = key;
    out.value = value;
    return out;
  }

  static bool containsRawControlByte(const std::string &s) {
    for (char c : s) {
      const unsigned char uc = static_cast<unsigned char>(c);
      // ASCII control bytes that JSON must escape: 0x00..0x1F, 0x7F.
      if (uc < 0x20u || uc == 0x7Fu) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<rises::FiwareBridgeNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> spy_node_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spy_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> captured_;
};

} // namespace

// "level" is reserved. An attacker who sets KeyValue{key="level", value="7"}
// must NOT shadow the canonical numeric diagnostic level field.
TEST_F(DiagInjectionFixture, ReservedKeyLevelIsRejectedOrEscaped) {
  buildBridge();

  publishDiag("attacker_node", DiagnosticStatus::OK, "ok", {kv("level", "7")});

  ASSERT_TRUE(spinUntilCaptured()) << "Bridge did not forward diagnostics";

  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("attacker_node"));
  const nlohmann::json &info = parsed["attacker_node"];

  ASSERT_TRUE(info.contains("level")) << "level field missing entirely";
  EXPECT_TRUE(info["level"].is_number())
      << "Attacker-supplied string overwrote canonical numeric level.";
  EXPECT_EQ(info["level"].get<int>(), static_cast<int>(DiagnosticStatus::OK));
}

// "message" is reserved. Same attack vector.
TEST_F(DiagInjectionFixture, ReservedKeyMessageIsRejectedOrEscaped) {
  buildBridge();

  publishDiag("attacker_node", DiagnosticStatus::WARN, "canonical_warning",
              {kv("message", "ATTACKER_OVERWRITE")});

  ASSERT_TRUE(spinUntilCaptured());
  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("attacker_node"));
  const nlohmann::json &info = parsed["attacker_node"];

  ASSERT_TRUE(info.contains("message"));
  EXPECT_EQ(info["message"].get<std::string>(), "canonical_warning")
      << "Attacker overwrote canonical message.";
}

// Benign keys are forwarded normally.
TEST_F(DiagInjectionFixture, ArbitraryAllowedKeyForwarded) {
  buildBridge();

  publishDiag("sensor_a", DiagnosticStatus::OK, "ok",
              {kv("sensor_temperature", "42.0")});

  ASSERT_TRUE(spinUntilCaptured());
  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("sensor_a"));
  ASSERT_TRUE(parsed["sensor_a"].contains("sensor_temperature"));
  EXPECT_EQ(parsed["sensor_a"]["sensor_temperature"].get<std::string>(),
            "42.0");
}

// Key with 1 MiB length must be dropped.
TEST_F(DiagInjectionFixture, KeyLengthOverCapRejected) {
  buildBridge();

  const std::string huge_key(kAttackPayloadBytes, 'k');
  publishDiag("bloated_node", DiagnosticStatus::OK, "ok",
              {kv(huge_key, "small_value")});

  ASSERT_TRUE(spinUntilCaptured());
  EXPECT_LT(captured_.front().size(), kMaxAcceptableJsonBytes)
      << "Forwarded JSON kept a >=1 MiB attacker-supplied key.";

  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("bloated_node"));
  EXPECT_FALSE(parsed["bloated_node"].contains(huge_key))
      << "Oversized key was not dropped.";
}

// Value with 1 MiB length must be dropped.
TEST_F(DiagInjectionFixture, ValueLengthOverCapRejected) {
  buildBridge();

  const std::string huge_value(kAttackPayloadBytes, 'v');
  publishDiag("bloated_node", DiagnosticStatus::OK, "ok",
              {kv("blob", huge_value)});

  ASSERT_TRUE(spinUntilCaptured());
  EXPECT_LT(captured_.front().size(), kMaxAcceptableJsonBytes)
      << "Forwarded JSON kept a >=1 MiB attacker-supplied value.";

  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("bloated_node"));
  if (parsed["bloated_node"].contains("blob")) {
    EXPECT_LT(parsed["bloated_node"]["blob"].get<std::string>().size(),
              kAttackPayloadBytes)
        << "Oversized value was not truncated or dropped.";
  }
}

// Keys containing control characters must be sanitised so the forwarded JSON
// never contains raw control bytes (after JSON re-parse / re-emit).
TEST_F(DiagInjectionFixture, UnicodeKeyHandledSafely) {
  buildBridge();

  // Key contains a NUL byte, a bell, and an emoji.
  std::string evil_key;
  evil_key.push_back('a');
  evil_key.push_back('\x00');
  evil_key.push_back('\x07');
  evil_key.append("\xF0\x9F\x9A\x80"); // U+1F680 rocket.
  publishDiag("unicode_node", DiagnosticStatus::OK, "ok", {kv(evil_key, "v")});

  ASSERT_TRUE(spinUntilCaptured());

  const nlohmann::json parsed = nlohmann::json::parse(captured_.front());
  ASSERT_TRUE(parsed.contains("unicode_node"));
  for (const auto &[k, v] : parsed["unicode_node"].items()) {
    EXPECT_FALSE(containsRawControlByte(k))
        << "Forwarded JSON contains raw control bytes in key: " << k;
  }
}
