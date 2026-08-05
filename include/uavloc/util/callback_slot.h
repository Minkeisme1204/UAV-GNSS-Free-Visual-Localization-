#pragma once

//! util::CallbackSlot<void(Args...)> — the ONE publish/subscribe primitive of
//! the project (S5b, .docs/designs/system_manager_design.md §3.7). It replaces
//! util::Publisher<T> (S1) and backs debug_viewer::CallbackSlot.
//!
//! Specialized on a FUNCTION SIGNATURE, not on a payload type, so a channel that
//! carries "(timestamp, payload)" needs no wrapper struct. Named event structs
//! (core::SystemCallbacks, debug_viewer::DebugViewerCallbacks) group the slots.
//!
//! Header-only on purpose: it is a template, and `util` is the leaf module both
//! `core` and the (deliberately libuavloc-free) debug_viewer depend on.
//!
//! Semantics — all covered by tests/test_util_callback.cpp:
//!   1. Subscribers are invoked SYNCHRONOUSLY, on the caller's thread, in
//!      registration order.
//!   2. NO mutex is held while a subscriber runs: operator() copies the
//!      subscriber list under the lock and releases it before calling. A
//!      subscriber may therefore call add() / remove() / operator() on the same
//!      slot without deadlocking. (The debug_viewer version this replaces held
//!      its mutex across the call — an add() from inside a callback deadlocked;
//!      the kcb version took no lock at all — a real data race.)
//!   3. Because of that snapshot, a subscriber removed DURING an in-flight call
//!      may still be invoked for that call (it is already in the snapshot). It
//!      is guaranteed never to be invoked by any LATER call. CallbackSlot does
//!      NOT manage the lifetime of whatever the closure points at: a caller
//!      whose callback captures state must keep that state alive until the
//!      in-flight call returns (capture a shared_ptr, or remove from a thread
//!      that is not racing an emission). That is a contract on the caller.
//!   4. An exception escaping a subscriber is caught and logged; the remaining
//!      subscribers still run, and operator() itself never throws.

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace util {

template <typename Signature>
class CallbackSlot;

template <typename... Args>
class CallbackSlot<void(Args...)> {
public:
    using Callback = std::function<void(Args...)>;
    using Id       = std::uint64_t;

    //! Reserved id meaning "not a subscription". Never returned by a successful
    //! add(); never accepted by remove().
    static constexpr Id INVALID_ID = 0;

    CallbackSlot()  = default;
    ~CallbackSlot() = default;

    // Non-copyable / non-movable: it owns a mutex and hands out ids that must
    // stay meaningful for the lifetime of the object.
    CallbackSlot(const CallbackSlot&)            = delete;
    CallbackSlot& operator=(const CallbackSlot&) = delete;
    CallbackSlot(CallbackSlot&&)                 = delete;
    CallbackSlot& operator=(CallbackSlot&&)      = delete;

    //! Registers a subscriber. Returns its id, or INVALID_ID if `cb` is empty
    //! (an empty std::function would throw when invoked).
    Id add(Callback cb) {
        if (!cb) {
            spdlog::warn("util::CallbackSlot::add: empty callback ignored");
            return INVALID_ID;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const Id id = ++next_id_;  // first handed-out id is 1
        callbacks_.push_back(Entry{id, std::move(cb)});
        return id;
    }

    //! Removes a subscriber. Returns false if `id` is unknown (already removed,
    //! cleared, or INVALID_ID). See note 3 in the header comment for what
    //! happens during an in-flight emission.
    bool remove(Id id) {
        if (id == INVALID_ID) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = callbacks_.begin(); it != callbacks_.end(); ++it) {
            if (it->id == id) {
                callbacks_.erase(it);  // vector erase keeps registration order
                return true;
            }
        }
        return false;
    }

    //! Invokes every subscriber, in registration order, on the calling thread.
    //! Never throws: subscriber exceptions are logged.
    void operator()(Args... args) const {
        std::vector<Entry> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (callbacks_.empty()) {
                return;
            }
            snapshot = callbacks_;
        }
        // Lock released: a subscriber may re-enter this slot freely.
        for (const auto& entry : snapshot) {
            try {
                entry.callback(args...);
            } catch (const std::exception& ex) {
                spdlog::error("util::CallbackSlot: subscriber {} threw: {}", entry.id,
                              ex.what());
            } catch (...) {
                spdlog::error("util::CallbackSlot: subscriber {} threw a non-std exception",
                              entry.id);
            }
        }
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return callbacks_.size();
    }

    bool empty() const { return size() == 0; }

    //! Removes every subscriber. Ids handed out so far stay invalid forever (the
    //! counter is not rewound), so a stale remove() cannot hit a future
    //! subscriber.
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_.clear();
    }

private:
    struct Entry {
        Id       id = INVALID_ID;
        Callback callback;
    };

    mutable std::mutex mutex_;
    std::vector<Entry> callbacks_;
    Id                 next_id_ = INVALID_ID;
};

}  // namespace util
}  // namespace uavloc
