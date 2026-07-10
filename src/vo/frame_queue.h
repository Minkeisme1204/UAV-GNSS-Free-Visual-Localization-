#pragma once

// FrameQueue — a bounded, drop-oldest single-producer / single-consumer queue
// that decouples the frame source (VideoReader / camera pump thread) from the
// VO Tracking thread in the asynchronous VOModule mode.
//
// push() never blocks: when the queue is full the OLDEST frame is discarded so
// the consumer always works on the freshest imagery (real-time policy). pop()
// blocks up to a configurable timeout so the consumer loop can periodically
// re-check its shutdown flag. This queue is used only in async mode; the default
// synchronous processFrame() path does not touch it.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "uavloc/sensor/frame_data.h"

namespace uavloc::vo {

class FrameQueue {
public:
    // capacity        : maximum buffered frames before drop-oldest kicks in.
    // pop_timeout_ms  : how long pop() waits for a frame before returning false.
    FrameQueue(std::size_t capacity, int pop_timeout_ms);

    FrameQueue(const FrameQueue&)            = delete;
    FrameQueue& operator=(const FrameQueue&) = delete;

    // Enqueue a frame. Non-blocking. Returns true when the frame was buffered
    // without loss, false when an older frame had to be dropped to make room.
    bool push(const sensor::FrameData& frame);

    // Wait up to pop_timeout_ms for a frame. Returns true and moves it into
    // `out`; returns false on timeout or after stop() (with an empty queue) so
    // the consumer can re-check its running flag.
    bool pop(sensor::FrameData& out);

    // Wake any blocked pop() so the consumer thread can exit.
    void stop();

    std::size_t size() const;
    std::uint64_t droppedCount() const;

private:
    const std::size_t               capacity_;
    const std::chrono::milliseconds pop_timeout_;

    mutable std::mutex              mtx_;
    std::condition_variable         cv_;
    std::deque<sensor::FrameData>   queue_;
    bool                            stopped_ = false;
    std::uint64_t                   dropped_ = 0;
};

}  // namespace uavloc::vo
