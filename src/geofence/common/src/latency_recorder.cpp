/// @file latency_recorder.cpp
/// @brief Lock-free latency recorder with background CSV file writer.

#include "geofence/common/latency_recorder.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace rises::geofence
{

std::unique_ptr<LatencyRecorder> LatencyRecorder::create(
    const std::string& file_path,
    std::size_t buffer_size,
    std::chrono::milliseconds flush_interval)
{
    // Enforce power-of-2 buffer size (minimum 64)
    if (buffer_size < 64) {
        buffer_size = 64;
    }
    if ((buffer_size & (buffer_size - 1)) != 0) {
        // Round up to next power of 2
        std::size_t v = buffer_size - 1;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        buffer_size = v + 1;
    }

    // std::make_unique cannot be used here because the constructor is private.
    // The factory pattern enforces initialization via create() which validates
    // the file path before returning a usable recorder.
    std::unique_ptr<LatencyRecorder> recorder(
        new LatencyRecorder(file_path, buffer_size, flush_interval));

    if (!recorder->initialize()) {
        return nullptr;
    }

    return recorder;
}

LatencyRecorder::LatencyRecorder(
    const std::string& file_path,
    std::size_t buffer_size,
    std::chrono::milliseconds flush_interval)
    : file_path_(file_path)
    , flush_interval_(flush_interval)
    , buffer_(buffer_size)
    , mask_(buffer_size - 1)
{
}

LatencyRecorder::~LatencyRecorder()
{
    this->running_.store(false, std::memory_order_release);
    if (this->writer_thread_.joinable()) {
        this->writer_thread_.join();
    }
}

bool LatencyRecorder::initialize()
{
    std::FILE* file = std::fopen(this->file_path_.c_str(), "w");
    if (!file) {
        std::fprintf(stderr,
            "[latency_recorder::initialize] Failed to open output file '%s': %s\n",
            this->file_path_.c_str(), std::strerror(errno));
        return false;
    }

    // Write CSV header
    std::fprintf(file,
        "scan_stamp_ns,callback_start_ns,callback_end_ns,"
        "e2e_latency_ms,callback_duration_ms,"
        "matched_count,unmatched_count\n");
    std::fclose(file);

    this->running_.store(true, std::memory_order_release);
    this->writer_thread_ = std::thread(&LatencyRecorder::writerLoop, this);

    return true;
}

void LatencyRecorder::record(
    int64_t scan_stamp_ns,
    int64_t callback_start_ns,
    int64_t callback_end_ns,
    int32_t matched_count,
    int32_t unmatched_count) noexcept
{
    const std::size_t write_pos = this->write_idx_.load(std::memory_order_relaxed);
    const std::size_t read_pos = this->read_idx_.load(std::memory_order_acquire);

    // Check if buffer is full (leave one slot empty to distinguish full from empty)
    if (((write_pos + 1) & this->mask_) == (read_pos & this->mask_)) {
        this->samples_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    LatencySample& slot = this->buffer_[write_pos & this->mask_];
    slot.scan_stamp_ns = scan_stamp_ns;
    slot.callback_start_ns = callback_start_ns;
    slot.callback_end_ns = callback_end_ns;
    slot.matched_count = matched_count;
    slot.unmatched_count = unmatched_count;

    this->write_idx_.store(write_pos + 1, std::memory_order_release);
}

void LatencyRecorder::writerLoop()
{
    // Open file in append mode (header already written)
    std::FILE* file = std::fopen(this->file_path_.c_str(), "a");
    if (!file) {
        std::fprintf(stderr,
            "[latency_recorder::writerLoop] Failed to reopen output file '%s': %s\n",
            this->file_path_.c_str(), std::strerror(errno));
        return;
    }

    char line_buf[256];

    while (this->running_.load(std::memory_order_acquire)) {
        const std::size_t read_pos = this->read_idx_.load(std::memory_order_relaxed);
        const std::size_t write_pos = this->write_idx_.load(std::memory_order_acquire);

        if (read_pos == write_pos) {
            // Buffer empty — sleep
            std::this_thread::sleep_for(this->flush_interval_);
            continue;
        }

        // Drain all available samples
        std::size_t current = read_pos;
        while (current != write_pos) {
            const LatencySample& sample = this->buffer_[current & this->mask_];

            const double e2e_ms = static_cast<double>(
                sample.callback_end_ns - sample.scan_stamp_ns) / 1e6;
            const double callback_ms = static_cast<double>(
                sample.callback_end_ns - sample.callback_start_ns) / 1e6;

            const int len = std::snprintf(line_buf, sizeof(line_buf),
                "%ld,%ld,%ld,%.3f,%.3f,%d,%d\n",
                static_cast<long>(sample.scan_stamp_ns),
                static_cast<long>(sample.callback_start_ns),
                static_cast<long>(sample.callback_end_ns),
                e2e_ms, callback_ms,
                sample.matched_count, sample.unmatched_count);

            if (len > 0) {
                std::fwrite(line_buf, 1, static_cast<std::size_t>(len), file);
            }

            this->samples_written_.fetch_add(1, std::memory_order_relaxed);
            ++current;
        }

        this->read_idx_.store(current, std::memory_order_release);
        std::fflush(file);
    }

    // Final drain on shutdown
    const std::size_t read_pos = this->read_idx_.load(std::memory_order_relaxed);
    const std::size_t write_pos = this->write_idx_.load(std::memory_order_acquire);
    std::size_t current = read_pos;
    while (current != write_pos) {
        const LatencySample& sample = this->buffer_[current & this->mask_];
        const double e2e_ms = static_cast<double>(
            sample.callback_end_ns - sample.scan_stamp_ns) / 1e6;
        const double callback_ms = static_cast<double>(
            sample.callback_end_ns - sample.callback_start_ns) / 1e6;

        std::fprintf(file, "%ld,%ld,%ld,%.3f,%.3f,%d,%d\n",
            static_cast<long>(sample.scan_stamp_ns),
            static_cast<long>(sample.callback_start_ns),
            static_cast<long>(sample.callback_end_ns),
            e2e_ms, callback_ms,
            sample.matched_count, sample.unmatched_count);

        this->samples_written_.fetch_add(1, std::memory_order_relaxed);
        ++current;
    }
    this->read_idx_.store(current, std::memory_order_release);

    std::fclose(file);
}

int64_t LatencyRecorder::samplesWritten() const noexcept
{
    return this->samples_written_.load(std::memory_order_relaxed);
}

int64_t LatencyRecorder::samplesDropped() const noexcept
{
    return this->samples_dropped_.load(std::memory_order_relaxed);
}

} // namespace rises::geofence
