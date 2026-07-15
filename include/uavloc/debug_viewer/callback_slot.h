#pragma once
#include <functional>
#include <mutex>
#include <vector>

namespace uavloc::debug_viewer {

template<typename Signature>
class CallbackSlot;

template<typename... Args>
class CallbackSlot<void(Args...)> {
public:
    using Func = std::function<void(Args...)>;

    void add(Func cb) {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_.push_back(std::move(cb));
    }

    void operator()(Args... args) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& cb : callbacks_) cb(args...);
    }

private:
    mutable std::mutex mutex_;
    std::vector<Func> callbacks_;
};

} // namespace uavloc::debug_viewer
