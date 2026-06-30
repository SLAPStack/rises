/// @file latency_recorder.hpp
/// @brief Lock-free latency recorder with background file writer.
///
/// Records end-to-end latency samples (LiDAR scan timestamp to obstacle
/// detection completion) and writes them to a CSV file from a dedicated
/// writer thread. The hot path (record()) is lock-free and allocation-free.
///
/// Usage:
///   auto recorder = LatencyRecorder::create("/tmp/latency.csv");
///   // In the obstacles callback:
///   recorder->record(scan_stamp_ns, callback_start_ns, callback_end_ns, matched, unmatched);
///   // Destructor flushes and joins the writer thread.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rises::geofence
{

/// @brief A single latency measurement sample.
struct LatencySample
{
    int64_t scan_stamp_ns;      ///< Original LiDAR scan timestamp (nanoseconds).
    int64_t callback_start_ns;  ///< When the geofence callback began (nanoseconds).
    int64_t callback_end_ns;    ///< When the geofence callback finished (nanoseconds).
    int32_t matched_count;      ///< Number of matched obstacles in this scan.
    int32_t unmatched_count;    ///< Number of unmatched obstacles in this scan.
};

/// @brief Lock-free latency recorder with background CSV writer.
///
/// Uses a fixed-size ring buffer with atomic indices. The producer (ROS callback)
/// writes samples without blocking. The consumer (writer thread) reads and
/// flushes to disk periodically. If the producer outpaces the consumer,
/// samples are silently dropped (acceptable for profiling data).
class LatencyRecorder
{
public:
    /// @brief Create a recorder that writes to the given CSV file path.
    /// @param file_path Output CSV file path.
    /// @param buffer_size Ring buffer capacity (must be power of 2).
    /// @param flush_interval How often the writer thread flushes to disk.
    /// @return Unique pointer to the recorder, or nullptr on failure.
    [[nodiscard]] static std::unique_ptr<LatencyRecorder> create(
        const std::string& file_path,
        std::size_t buffer_size = 8192,
        std::chrono::milliseconds flush_interval = std::chrono::milliseconds(500));

    ~LatencyRecorder();

    LatencyRecorder(const LatencyRecorder&) = delete;
    LatencyRecorder& operator=(const LatencyRecorder&) = delete;
    LatencyRecorder(LatencyRecorder&&) = delete;
    LatencyRecorder& operator=(LatencyRecorder&&) = delete;

    /// @brief Record a latency sample. Lock-free, allocation-free.
    ///        Safe to call from the ROS callback thread.
    void record(int64_t scan_stamp_ns,
                int64_t callback_start_ns,
                int64_t callback_end_ns,
                int32_t matched_count,
                int32_t unmatched_count) noexcept;

    /// @brief Number of samples written to disk so far.
    [[nodiscard]] int64_t samplesWritten() const noexcept;

    /// @brief Number of samples dropped due to full buffer.
    [[nodiscard]] int64_t samplesDropped() const noexcept;

private:
    explicit LatencyRecorder(const std::string& file_path,
                             std::size_t buffer_size,
                             std::chrono::milliseconds flush_interval);

    [[nodiscard]] bool initialize();
    void writerLoop();

    std::string file_path_;
    std::chrono::milliseconds flush_interval_;

    // Ring buffer (power-of-2 sized for fast modulo via bitmask)
    std::vector<LatencySample> buffer_;
    std::size_t mask_;  // buffer_size - 1
    alignas(64) std::atomic<std::size_t> write_idx_{0};
    alignas(64) std::atomic<std::size_t> read_idx_{0};

    // Writer thread
    std::thread writer_thread_;
    std::atomic<bool> running_{false};

    // Stats
    std::atomic<int64_t> samples_written_{0};
    std::atomic<int64_t> samples_dropped_{0};
};

} // namespace rises::geofence
