#pragma once
#include "geofence/utils/compiler_hints.hpp"
#include <cassert>
#include <memory>
#include <urcu.h>

namespace rcu {

// RAII guard that registers the calling thread with liburcu on construction
// and unregisters it on destruction. Used via a thread_local instance so
// every thread that touches RCU primitives auto-registers exactly once and
// auto-unregisters on thread exit. Without this, rcu_read_lock/unlock
// operate on a thread invisible to synchronize_rcu, which can free a
// snapshot still being read (use-after-free, silent corruption).
struct RcuThreadGuard {
  RcuThreadGuard() noexcept { rcu_register_thread(); }
  ~RcuThreadGuard() noexcept { rcu_unregister_thread(); }
  RcuThreadGuard(const RcuThreadGuard &) = delete;
  RcuThreadGuard &operator=(const RcuThreadGuard &) = delete;
  RcuThreadGuard(RcuThreadGuard &&) = delete;
  RcuThreadGuard &operator=(RcuThreadGuard &&) = delete;
};

inline void ensureRcuRegistered() noexcept {
  thread_local RcuThreadGuard guard{};
  (void)guard;
}

/**
 * @brief RCU-based atomic shared_ptr for lock-free reads
 *
 * Uses liburcu for true lock-free reads. Combines RCU pointer protection
 * with shared_ptr reference counting for safe memory management.
 *
 * Thread Safety:
 * - load(): Lock-free, wait-free reads (RCU read-side critical section)
 * - store(): Lock-free writes (RCU grace period for cleanup)
 * - Concurrent writes require external synchronization
 *
 * How it works:
 * 1. RCU protects the SnapshotWrapper* from being freed during reads
 * 2. shared_ptr refcounting protects the actual data T
 * 3. Readers copy shared_ptr inside RCU critical section
 * 4. Old wrappers deleted after RCU grace period
 *
 * @tparam T The type of snapshot data
 */

// Internal wrapper to hold the snapshot and the rcu_head
template <typename T> struct SnapshotWrapper {
  rcu_head rcu;            // Required by liburcu
  std::shared_ptr<T> snap; // The actual snapshot

  explicit SnapshotWrapper(std::shared_ptr<T> s) : snap(std::move(s)) {}
};

// Atomic RCU pointer with proper read-side critical sections
template <typename T> class AtomicSharedPtr {
public:
  AtomicSharedPtr() : ptr_(nullptr) {}

  explicit AtomicSharedPtr(std::shared_ptr<T> snap) {
    ptr_ = new SnapshotWrapper<T>(std::move(snap));
  }

  ~AtomicSharedPtr() {
    // Synchronous cleanup on destruction
    SnapshotWrapper<T> *old = ptr_;
    if (old) {
      synchronize_rcu(); // Wait for all readers to finish
      delete old;
    }
  }

  // Deleted copy/move to prevent multiple ownership
  AtomicSharedPtr(const AtomicSharedPtr &) = delete;
  AtomicSharedPtr &operator=(const AtomicSharedPtr &) = delete;
  AtomicSharedPtr(AtomicSharedPtr &&) = delete;
  AtomicSharedPtr &operator=(AtomicSharedPtr &&) = delete;

  /**
   * @brief Lock-free read with proper RCU protection
   *
   * Enters RCU read-side critical section, dereferences the pointer,
   * copies the shared_ptr (incrementing refcount), then exits critical
   * section. The shared_ptr keeps the data alive even after exiting.
   *
   * @return Shared pointer to current snapshot, or nullptr if not set
   *
   * @note LOCK-FREE: No mutexes, no blocking
   * @note Thread auto-registers on first call via thread_local guard.
   */
  std::shared_ptr<T> load() const {
    ensureRcuRegistered();
    rcu_read_lock(); // Enter read-side critical section

    // Safe to dereference while in critical section
    SnapshotWrapper<T> *w = rcu_dereference(ptr_);

    // Copy the shared_ptr (increments refcount) - this is the key!
    // The shared_ptr keeps the data alive even after we exit the critical
    // section
    std::shared_ptr<T> result = LIKELY(w) ? w->snap : nullptr;

    rcu_read_unlock(); // Exit critical section

    return result; // Safe: shared_ptr refcount prevents deletion
  }

  /**
   * @brief Lock-free update to new snapshot
   *
   * Installs new snapshot and schedules old wrapper for deferred deletion
   * after RCU grace period. Readers that started before the update can
   * still safely access the old data.
   *
   * @param new_snap New snapshot to install
   *
   * @note Lock-free for single writer
   * @note Concurrent writes require external synchronization
   * @note Old wrapper deleted after grace period (all readers done)
   */
  void store(std::shared_ptr<T> new_snap) {
    ensureRcuRegistered();
    SnapshotWrapper<T> *new_wrapper =
        new SnapshotWrapper<T>(std::move(new_snap));
    SnapshotWrapper<T> *old_wrapper = ptr_;

    // Atomic pointer update - readers see old or new, never garbage
    rcu_assign_pointer(ptr_, new_wrapper);

    // Schedule old wrapper for deferred deletion
    // It will be deleted after all current readers finish (grace period)
    if (old_wrapper) {
      call_rcu(&old_wrapper->rcu, [](rcu_head *head) {
        // Calculate wrapper address from rcu_head offset
        auto *wrapper = reinterpret_cast<SnapshotWrapper<T> *>(
            reinterpret_cast<char *>(head) - offsetof(SnapshotWrapper<T>, rcu));
        delete wrapper; // Safe: no readers can access it anymore
      });
    }
  }

private:
  mutable SnapshotWrapper<T> *ptr_; // RCU-protected pointer
};

} // namespace rcu
