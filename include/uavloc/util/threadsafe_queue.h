#pragma once

//! util::ThreadsafeQueue<T> — bounded FIFO hand-off between two threads, with
//! the two push policies the system needs (see
//! .docs/designs/system_manager_design.md §4.2):
//!   * push_blocking_if_full()  — replay / offline: never lose a frame, the
//!     producer waits (with a deadline) until the consumer makes room.
//!   * push_dropping_if_full()  — live camera: never block the producer, throw
//!     the OLDEST item away and keep processing the newest.
//!
//! Header-only on purpose (template, leaf module).
//!
//! Shutdown semantics: shutdown() wakes BOTH waiting producers and waiting
//! consumers so every thread can leave its loop. After shutdown, pop_blocking()
//! keeps handing out the items still queued and only returns false once the
//! queue is empty — i.e. a consumer loop drains cleanly instead of dropping
//! whatever was in flight. Pushes after shutdown are refused.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace uavloc {
namespace util {

template <typename T>
class ThreadsafeQueue {
public:
    //! `capacity` == UNBOUNDED_CAPACITY (0) means unbounded: the queue is never
    //! considered full, so no push ever blocks and nothing is ever dropped.
    static constexpr std::size_t UNBOUNDED_CAPACITY = 0;

    explicit ThreadsafeQueue(std::size_t capacity) : capacity_(capacity) {}

    ~ThreadsafeQueue() = default;

    ThreadsafeQueue(const ThreadsafeQueue&)            = delete;
    ThreadsafeQueue& operator=(const ThreadsafeQueue&) = delete;
    ThreadsafeQueue(ThreadsafeQueue&&)                 = delete;
    ThreadsafeQueue& operator=(ThreadsafeQueue&&)      = delete;

    //! Blocks until there is room or `timeout` elapses.
    //! true  = pushed.
    //! false = TIMED OUT (nothing pushed, `value` untouched) or shut down.
    bool push_blocking_if_full(T value, std::chrono::milliseconds timeout) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (shutdown_) {
                return false;
            }
            if (is_full_locked()) {
                // Predicate form: immune to spurious wakeups, and returns the
                // predicate value (false == the deadline expired).
                const bool has_room = cv_not_full_.wait_for(
                    lock, timeout, [this] { return shutdown_ || !is_full_locked(); });
                if (!has_room || shutdown_) {
                    return false;
                }
            }
            queue_.push_back(std::move(value));
        }
        cv_not_empty_.notify_one();
        return true;
    }

    //! Never blocks.
    //! true  = pushed with no loss.
    //! false = the OLDEST item was dropped to make room, or the queue is shut
    //!         down (in which case nothing is pushed).
    bool push_dropping_if_full(T value) {
        bool no_loss = true;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) {
                return false;
            }
            while (is_full_locked()) {
                queue_.pop_front();  // drop-oldest
                ++dropped_count_;
                no_loss = false;
            }
            queue_.push_back(std::move(value));
        }
        cv_not_empty_.notify_one();
        return no_loss;
    }

    //! Blocks until an item is available.
    //! false = shut down AND drained (every queued item has been handed out).
    bool pop_blocking(T& out) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_not_empty_.wait(lock, [this] { return shutdown_ || !queue_.empty(); });
            if (queue_.empty()) {
                return false;  // only reachable after shutdown
            }
            out = std::move(queue_.front());
            queue_.pop_front();
        }
        cv_not_full_.notify_one();
        return true;
    }

    //! Never blocks. false = the queue is empty right now.
    bool try_pop(T& out) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                return false;
            }
            out = std::move(queue_.front());
            queue_.pop_front();
        }
        cv_not_full_.notify_one();
        return true;
    }

    //! Wakes every waiter (producers and consumers) so their loops can exit.
    //! Idempotent; queued items are kept so consumers can drain them.
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        cv_not_empty_.notify_all();
        cv_not_full_.notify_all();
    }

    bool is_shutdown() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return shutdown_;
    }

    //! Discards every queued item. These are NOT counted as drops:
    //! dropped_count() reports losses caused by the drop-oldest policy only.
    void clear() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.clear();
        }
        cv_not_full_.notify_all();
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    //! Configured capacity; UNBOUNDED_CAPACITY means no limit.
    std::size_t capacity() const { return capacity_; }

    //! Cumulative number of items thrown away by push_dropping_if_full().
    std::size_t dropped_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_count_;
    }

private:
    //! Must be called with `mutex_` held.
    bool is_full_locked() const {
        return capacity_ != UNBOUNDED_CAPACITY && queue_.size() >= capacity_;
    }

    const std::size_t capacity_;

    mutable std::mutex      mutex_;
    std::condition_variable cv_not_full_;   // waited on by producers
    std::condition_variable cv_not_empty_;  // waited on by consumers

    std::deque<T> queue_;
    bool          shutdown_      = false;  // guarded by mutex_
    std::size_t   dropped_count_ = 0;      // guarded by mutex_
};

}  // namespace util
}  // namespace uavloc
