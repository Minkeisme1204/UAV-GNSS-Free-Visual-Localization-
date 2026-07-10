#pragma once

// Implementation-only spdlog sink that captures formatted log lines into a
// bounded, shared ring buffer. The render thread reads the same buffer (under
// its mutex) to draw the scrolling log terminal panel. Never installed.

#include <spdlog/sinks/base_sink.h>

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace uavloc::debug_viewer {

// Shared bounded ring of formatted log lines. The sink (any logging thread)
// appends; the render thread copies it out, both under `mutex`.
struct LogRing {
    std::mutex              mutex;
    std::deque<std::string> lines;
    std::size_t             capacity = 500;
};

// spdlog sink writing each formatted record into a LogRing. base_sink already
// serialises sink_it_ with its own mutex; LogRing::mutex additionally guards
// against the concurrent render-thread reader.
class LogRingSink : public spdlog::sinks::base_sink<std::mutex> {
public:
    explicit LogRingSink(std::shared_ptr<LogRing> ring) : ring_(std::move(ring)) {}

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        spdlog::memory_buf_t formatted;
        this->formatter_->format(msg, formatted);
        std::string line(formatted.data(), formatted.size());
        // Drop a single trailing newline so ImGui list items aren't double-spaced.
        if (!line.empty() && line.back() == '\n') {
            line.pop_back();
        }
        std::lock_guard<std::mutex> lock(ring_->mutex);
        ring_->lines.push_back(std::move(line));
        while (ring_->lines.size() > ring_->capacity) {
            ring_->lines.pop_front();
        }
    }

    void flush_() override {}

private:
    std::shared_ptr<LogRing> ring_;
};

} // namespace uavloc::debug_viewer
