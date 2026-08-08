#include "uavloc/util/scoped_timer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>

namespace uavloc {
namespace util {

namespace {

constexpr std::size_t NUM_STAGES = static_cast<std::size_t>(ProfileStage::COUNT);

// Per-stage accumulator cell. The owning thread is the only writer; snapshots
// read it from another thread, so the fields are relaxed atomics (a plain load/
// store on every platform we build for) instead of racy plain doubles.
struct AtomicStat {
    std::atomic<unsigned long long> count{0};
    std::atomic<double>             total_ms{0.0};
    std::atomic<double>             last_ms{0.0};
    std::atomic<double>             max_ms{0.0};
};

// One accumulator per thread. Kept alive by a shared_ptr held BOTH by the
// thread_local slot and by the global registry, so a snapshot taken after the
// producing thread has exited (e.g. the mapping thread stopped before the
// driver dumps its CSV) still sees that thread's samples.
struct ThreadEntry {
    std::atomic<bool> label_set{false};
    std::string       label{"unlabeled"};  // guarded by label_mutex
    std::mutex        label_mutex;
    std::array<AtomicStat, NUM_STAGES> stats;  // NSDMI-initialized per element
};

std::atomic<bool>& enabled_flag() {
    static std::atomic<bool> flag{false};
    return flag;
}

std::mutex& registry_mutex() {
    static std::mutex m;
    return m;
}

std::vector<std::shared_ptr<ThreadEntry>>& registry() {
    static std::vector<std::shared_ptr<ThreadEntry>> r;
    return r;
}

// Thread-local accumulator, lazily created and registered on first use.
ThreadEntry& acquire_entry() {
    thread_local std::shared_ptr<ThreadEntry> entry;
    if (!entry) {
        entry = std::make_shared<ThreadEntry>();
        std::lock_guard<std::mutex> lock(registry_mutex());
        registry().push_back(entry);
    }
    return *entry;
}

StageStat read_stat(const AtomicStat& s) {
    StageStat out;
    out.count    = s.count.load(std::memory_order_relaxed);
    out.total_ms = s.total_ms.load(std::memory_order_relaxed);
    out.last_ms  = s.last_ms.load(std::memory_order_relaxed);
    out.max_ms   = s.max_ms.load(std::memory_order_relaxed);
    return out;
}

bool has_any_sample(const ThreadEntry& e) {
    for (const auto& s : e.stats) {
        if (s.count.load(std::memory_order_relaxed) != 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

const char* to_string(ProfileStage stage) {
    switch (stage) {
        case ProfileStage::PROCESS_FRAME_TOTAL: return "PROCESS_FRAME_TOTAL";
        case ProfileStage::FRAME_LOAD:          return "FRAME_LOAD";
        case ProfileStage::ORB_EXTRACT:         return "ORB_EXTRACT";
        case ProfileStage::INIT:                return "INIT";
        case ProfileStage::TRACK_FRAME:         return "TRACK_FRAME";
        case ProfileStage::TRACK_MATCH:         return "TRACK_MATCH";
        case ProfileStage::TRACK_POSE_OPT:      return "TRACK_POSE_OPT";
        case ProfileStage::LOCAL_MAP_UPDATE:    return "LOCAL_MAP_UPDATE";
        case ProfileStage::LOCAL_MAP_SEARCH:    return "LOCAL_MAP_SEARCH";
        case ProfileStage::LOCAL_MAP_POSE_OPT:  return "LOCAL_MAP_POSE_OPT";
        case ProfileStage::KF_GATE:             return "KF_GATE";
        case ProfileStage::KF_INSERT:           return "KF_INSERT";
        case ProfileStage::MAPPING_SUBMIT_WAIT: return "MAPPING_SUBMIT_WAIT";
        case ProfileStage::PUBLISH:             return "PUBLISH";
        case ProfileStage::MAP_QUEUE_WAIT:      return "MAP_QUEUE_WAIT";
        case ProfileStage::MAP_STORE_KF:        return "MAP_STORE_KF";
        case ProfileStage::MAP_CULL_LM:         return "MAP_CULL_LM";
        case ProfileStage::MAP_CREATE_LM:       return "MAP_CREATE_LM";
        case ProfileStage::MAP_FUSE_LM:         return "MAP_FUSE_LM";
        case ProfileStage::MAP_UPDATE_CONN:     return "MAP_UPDATE_CONN";
        case ProfileStage::MAP_LOCAL_BA:        return "MAP_LOCAL_BA";
        case ProfileStage::MAP_CULL_KF:         return "MAP_CULL_KF";
        case ProfileStage::VIDEO_DECODE:        return "VIDEO_DECODE";
        case ProfileStage::FUSION_PUSH:         return "FUSION_PUSH";
        case ProfileStage::FUSION_GRAPH_UPDATE: return "FUSION_GRAPH_UPDATE";
        case ProfileStage::COUNT:               break;
    }
    return "UNKNOWN";
}

ProfileStage parent_stage(ProfileStage stage) {
    switch (stage) {
        // Measured inside PROCESS_FRAME_TOTAL (level-1 children).
        case ProfileStage::FRAME_LOAD:
        case ProfileStage::INIT:
        case ProfileStage::TRACK_FRAME:
        case ProfileStage::LOCAL_MAP_UPDATE:
        case ProfileStage::LOCAL_MAP_SEARCH:
        case ProfileStage::LOCAL_MAP_POSE_OPT:
        case ProfileStage::KF_GATE:
        case ProfileStage::KF_INSERT:
        case ProfileStage::MAPPING_SUBMIT_WAIT:
        case ProfileStage::PUBLISH:
            return ProfileStage::PROCESS_FRAME_TOTAL;
        // Level-2 children.
        case ProfileStage::ORB_EXTRACT:
            return ProfileStage::FRAME_LOAD;
        case ProfileStage::TRACK_MATCH:
        case ProfileStage::TRACK_POSE_OPT:
            return ProfileStage::TRACK_FRAME;
        case ProfileStage::FUSION_GRAPH_UPDATE:
            return ProfileStage::FUSION_PUSH;
        // Roots: their durations are disjoint and may be summed.
        default:
            return stage;
    }
}

void Profiler::set_enabled(bool enabled) {
    enabled_flag().store(enabled, std::memory_order_relaxed);
}

bool Profiler::enabled() {
    return enabled_flag().load(std::memory_order_relaxed);
}

void Profiler::set_thread_label(const char* label) {
    if (label == nullptr) {
        return;
    }
    ThreadEntry& e = acquire_entry();
    if (e.label_set.load(std::memory_order_relaxed)) {
        return;  // first call wins (see header)
    }
    {
        std::lock_guard<std::mutex> lock(e.label_mutex);
        e.label = label;
    }
    e.label_set.store(true, std::memory_order_relaxed);
}

void Profiler::add_sample(ProfileStage stage, double ms) {
    if (!enabled() || stage >= ProfileStage::COUNT) {
        return;
    }
    AtomicStat& s = acquire_entry().stats[static_cast<std::size_t>(stage)];
    // Single writer (the owning thread): read-modify-write without CAS is safe.
    s.count.store(s.count.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    s.total_ms.store(s.total_ms.load(std::memory_order_relaxed) + ms, std::memory_order_relaxed);
    s.last_ms.store(ms, std::memory_order_relaxed);
    if (ms > s.max_ms.load(std::memory_order_relaxed)) {
        s.max_ms.store(ms, std::memory_order_relaxed);
    }
}

std::vector<StageStat> Profiler::snapshot_total() {
    std::vector<StageStat> out(NUM_STAGES);
    std::lock_guard<std::mutex> lock(registry_mutex());
    for (const auto& entry : registry()) {
        for (std::size_t i = 0; i < NUM_STAGES; ++i) {
            const StageStat s = read_stat(entry->stats[i]);
            if (s.count == 0) {
                continue;
            }
            out[i].count += s.count;
            out[i].total_ms += s.total_ms;
            out[i].last_ms = s.last_ms;
            out[i].max_ms = std::max(out[i].max_ms, s.max_ms);
        }
    }
    return out;
}

std::vector<std::pair<std::string, std::vector<StageStat>>> Profiler::snapshot_by_thread() {
    std::vector<std::pair<std::string, std::vector<StageStat>>> out;
    std::lock_guard<std::mutex> lock(registry_mutex());
    for (const auto& entry : registry()) {
        if (!has_any_sample(*entry)) {
            continue;
        }
        std::string label;
        {
            std::lock_guard<std::mutex> label_lock(entry->label_mutex);
            label = entry->label;
        }
        std::vector<StageStat> stats(NUM_STAGES);
        for (std::size_t i = 0; i < NUM_STAGES; ++i) {
            stats[i] = read_stat(entry->stats[i]);
        }
        out.emplace_back(std::move(label), std::move(stats));
    }
    return out;
}

void Profiler::reset() {
    std::lock_guard<std::mutex> lock(registry_mutex());
    for (const auto& entry : registry()) {
        for (auto& s : entry->stats) {
            s.count.store(0, std::memory_order_relaxed);
            s.total_ms.store(0.0, std::memory_order_relaxed);
            s.last_ms.store(0.0, std::memory_order_relaxed);
            s.max_ms.store(0.0, std::memory_order_relaxed);
        }
    }
}

ScopedTimer::ScopedTimer(ProfileStage stage)
    : stage_(stage), active_(Profiler::enabled()) {
    if (active_) {
        t0_ = std::chrono::steady_clock::now();
    }
}

ScopedTimer::~ScopedTimer() {
    if (!active_) {
        return;
    }
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0_).count();
    Profiler::add_sample(stage_, ms);
}

}  // namespace util
}  // namespace uavloc
