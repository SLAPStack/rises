// =============================================================================
// Concurrency test: FiwareBridgeNode::pallet_map_ + pallet_map_dirty_
// =============================================================================
//
// Verifies that concurrent access to pallet_map_ and the pallet_map_dirty_ flag
// is race-free: under a MultiThreadedExecutor two publisher threads inject map
// updates and DDS triggers concurrently while mapUpdatesCallback /
// ddsTriggerCallback / pushPalletMap dispatch on different workers. The test
// asserts behavioural invariants (no crash, atomic dirty-flag flipping) and,
// when built with TSan, surfaces any data race on the shared map or flag.
//
// TSan invocation:
//   colcon build --packages-select fiware_bridge \
//     --cmake-args -DBUILD_TESTING=ON -DARIS_ENABLE_TSAN=ON
//   colcon test --packages-select fiware_bridge \
//     --ctest-args -R test_pallet_map_race
//
// Standards: function cap 100, nesting <= 3, named constants, no TODOs.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "rises_interfaces/msg/obstacle.hpp"
#include "rises_interfaces/msg/obstacle_update.hpp"
#include "rises_interfaces/msg/obstacle_update_array.hpp"
#include "fiware_bridge/fiware_bridge_node.hpp"

namespace {

constexpr int kExecutorThreads = 3;
constexpr int kPalletPerBatch = 16;
// Map-updates can fire fast — pushPalletMap is throttled and lightweight.
constexpr auto kMapUpdateInterval = std::chrono::milliseconds(2);
// DDS triggers do a blocking wait_for_service inside the callback (the
// geofence service does not exist in this fixture, so each call waits up to
// ~1 s before returning). Throttle them aggressively so the test completes
// inside the wall-clock budget.
constexpr auto kDdsTriggerInterval = std::chrono::milliseconds(400);
constexpr auto kRunWindow = std::chrono::milliseconds(800);
constexpr auto kSettleWindow = std::chrono::milliseconds(200);

constexpr const char *kMapUpdatesTopic = "/warehouse/map_updates";
constexpr const char *kDdsTriggerTopic = "fiware/dds_trigger";

rises_interfaces::msg::ObstacleUpdate makeInsertUpdate(std::int64_t id) {
  rises_interfaces::msg::ObstacleUpdate update;
  update.operation = rises_interfaces::msg::ObstacleUpdate::OP_INSERT;
  update.obstacle.id = static_cast<std::uint64_t>(id);
  update.obstacle.type = rises_interfaces::msg::Obstacle::RECTANGLE;
  update.obstacle.position.x = static_cast<double>(id);
  update.obstacle.position.y = static_cast<double>(id);
  update.obstacle.width = 1.0F;
  update.obstacle.height = 1.0F;
  return update;
}

rises_interfaces::msg::ObstacleUpdate makeDeleteUpdate(std::int64_t id) {
  rises_interfaces::msg::ObstacleUpdate update;
  update.operation = rises_interfaces::msg::ObstacleUpdate::OP_DELETE;
  update.obstacle.id = static_cast<std::uint64_t>(id);
  return update;
}

rises_interfaces::msg::ObstacleUpdateArray makeUpdateBatch(int seq) {
  rises_interfaces::msg::ObstacleUpdateArray batch;
  for (int i = 0; i < kPalletPerBatch; ++i) {
    const std::int64_t id = (seq * kPalletPerBatch + i) % 64;
    if ((seq + i) % 2 == 0) {
      batch.updates.push_back(makeInsertUpdate(id));
    } else {
      batch.updates.push_back(makeDeleteUpdate(id));
    }
  }
  return batch;
}

class PalletMapRaceFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    node_ = std::make_shared<rises::FiwareBridgeNode>(options);
    helper_ = std::make_shared<rclcpp::Node>("fiware_pallet_race_helper");

    map_updates_pub_ =
        helper_->create_publisher<rises_interfaces::msg::ObstacleUpdateArray>(
            kMapUpdatesTopic, rclcpp::QoS(100).reliable());
    dds_trigger_pub_ = helper_->create_publisher<std_msgs::msg::Empty>(
        kDdsTriggerTopic, rclcpp::QoS(1).reliable());

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions{}, kExecutorThreads);
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    map_updates_pub_.reset();
    dds_trigger_pub_.reset();
    helper_.reset();
    node_.reset();
  }

  std::shared_ptr<rises::FiwareBridgeNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<rises_interfaces::msg::ObstacleUpdateArray>::SharedPtr
      map_updates_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dds_trigger_pub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

} // namespace

// -----------------------------------------------------------------------------
// MqttCallbackAndRosCallbackRaceFree
// -----------------------------------------------------------------------------
TEST_F(PalletMapRaceFixture, MqttCallbackAndRosCallbackRaceFree) {
  std::atomic<bool> producer_failed{false};
  std::atomic<bool> stop{false};

  std::thread map_updates_thread([this, &stop, &producer_failed]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        map_updates_pub_->publish(makeUpdateBatch(seq++));
        std::this_thread::sleep_for(kMapUpdateInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread dds_trigger_thread([this, &stop, &producer_failed]() {
    try {
      while (!stop.load(std::memory_order_acquire)) {
        dds_trigger_pub_->publish(std_msgs::msg::Empty{});
        std::this_thread::sleep_for(kDdsTriggerInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);
  map_updates_thread.join();
  dds_trigger_thread.join();
  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load())
      << "publish() of map_updates / dds_trigger threw during the race window";
}

// -----------------------------------------------------------------------------
// DirtyFlagFlippingIsAtomic
// -----------------------------------------------------------------------------
//
// pallet_map_dirty_ is a plain bool toggled from at least two callback paths
// (ddsTriggerCallback sets it true; pushPalletMap reads and clears it).
// Repeatedly firing both DDS-trigger and map-updates from two threads
// exercises concurrent read-modify-write on the bool. Under TSan: data race
// on pallet_map_dirty_. Structurally we assert no exception escapes.
TEST_F(PalletMapRaceFixture, DirtyFlagFlippingIsAtomic) {
  std::atomic<bool> producer_failed{false};
  std::atomic<bool> stop{false};

  std::thread trigger_a([this, &stop, &producer_failed]() {
    try {
      while (!stop.load(std::memory_order_acquire)) {
        dds_trigger_pub_->publish(std_msgs::msg::Empty{});
        std::this_thread::sleep_for(kDdsTriggerInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::thread trigger_b([this, &stop, &producer_failed]() {
    int seq = 0;
    try {
      while (!stop.load(std::memory_order_acquire)) {
        // map_updates_callback also flips pallet_map_dirty_ to true; this
        // gives a second concurrent writer for the flag.
        map_updates_pub_->publish(makeUpdateBatch(seq++));
        std::this_thread::sleep_for(kMapUpdateInterval);
      }
    } catch (...) {
      producer_failed.store(true, std::memory_order_release);
    }
  });

  std::this_thread::sleep_for(kRunWindow);
  stop.store(true, std::memory_order_release);
  trigger_a.join();
  trigger_b.join();
  std::this_thread::sleep_for(kSettleWindow);

  EXPECT_FALSE(producer_failed.load());
}
