/// @file scoped_timer.hpp
/// @brief Lightweight scoped timer for profiling hot-path sections.
///
/// Usage:
///   {
///     ScopedTimer timer("batch_check", logger);
///     // ... code to time ...
///   } // prints elapsed time on destruction
///
/// Enable with GEOFENCE_PROFILE=1 cmake option or define.
/// When disabled, compiles to nothing (zero overhead).

#pragma once

// Third-party (ROS 2)
#include <rclcpp/rclcpp.hpp>

// Standard library
#include <chrono>

namespace rises::geofence {

#ifdef GEOFENCE_PROFILE

class ScopedTimer {
public:
    ScopedTimer(const char* name, rclcpp::Logger logger)
        : name_(name), logger_(logger),
          start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        const auto end = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(end - this->start_).count();
        RCLCPP_INFO(this->logger_, "[PROFILE] %s: %.3f ms", this->name_, ms);
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    const char* name_;
    rclcpp::Logger logger_;
    std::chrono::steady_clock::time_point start_;
};

#else

// Zero-cost when profiling is disabled
class ScopedTimer {
public:
    ScopedTimer(const char*, rclcpp::Logger) {}
};

#endif

} // namespace rises::geofence
