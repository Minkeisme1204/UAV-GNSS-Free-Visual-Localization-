#include "frame_queue.h"

namespace uavloc::vo {

FrameQueue::FrameQueue(std::size_t capacity, int pop_timeout_ms)
    : capacity_(capacity == 0 ? 1 : capacity),
      pop_timeout_(std::chrono::milliseconds(pop_timeout_ms < 0 ? 0 : pop_timeout_ms)) {}

bool FrameQueue::push(const sensor::FrameData& frame) {
    bool dropped = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (queue_.size() >= capacity_) {
            queue_.pop_front();  // drop-oldest: keep the freshest frames
            ++dropped_;
            dropped = true;
        }
        queue_.push_back(frame);
    }
    cv_.notify_one();
    return !dropped;
}

bool FrameQueue::pop(sensor::FrameData& out) {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!cv_.wait_for(lk, pop_timeout_,
                      [this] { return !queue_.empty() || stopped_; })) {
        return false;  // timeout — let the consumer re-check its running flag
    }
    if (queue_.empty()) {
        return false;  // stopped with nothing left to drain
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void FrameQueue::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stopped_ = true;
    }
    cv_.notify_all();
}

std::size_t FrameQueue::size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return queue_.size();
}

std::uint64_t FrameQueue::droppedCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return dropped_;
}

}  // namespace uavloc::vo
