#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace uavloc {
namespace util {

// Coarse-grained wall-clock stages measured by the profiler. The enumeration
// order is also the canonical display order (CSV rows / viewer table). Each
// stage is meant to wrap a COARSE block only — never a per-keypoint or
// per-landmark loop body.
enum class ProfileStage {
    // ── tracking thread (the thread that calls VOModule::process_frame) ──────
    PROCESS_FRAME_TOTAL,  // whole VOModule::process_frame body (root)
    FRAME_LOAD,           // VoFrameLoader::load (grayscale + ORB + undistort + grid)
    ORB_EXTRACT,          // ORB extraction inside FRAME_LOAD
    INIT,                 // Initializer::initialize
    TRACK_FRAME,          // frame-to-frame tracking (motion / robust match)
    TRACK_MATCH,          // matching step inside TRACK_FRAME
    TRACK_POSE_OPT,       // pose optimization inside TRACK_FRAME
    LOCAL_MAP_UPDATE,     // LocalMapUpdater::acquire_local_map
    LOCAL_MAP_SEARCH,     // projection search against the local map
    LOCAL_MAP_POSE_OPT,   // pose optimization with the local map
    KF_GATE,              // KeyframeInserter::new_keyframe_is_needed
    KF_INSERT,            // KeyframeInserter::insert_new_keyframe
    MAPPING_SUBMIT_WAIT,  // MappingModule::submit (may block on the local BA)
    PUBLISH,              // data-out callbacks

    // ── mapping thread ──────────────────────────────────────────────────────
    MAP_QUEUE_WAIT,       // blocking wait at the mapping loop top (root)
    MAP_STORE_KF,         // LocalMapper::store_new_keyframe (root)
    MAP_CULL_LM,          // remove_invalid_landmarks (root)
    MAP_CREATE_LM,        // create_new_landmarks (root)
    MAP_FUSE_LM,          // fuse_landmark_duplication (root)
    MAP_UPDATE_CONN,      // covisibility graph refresh (root)
    MAP_LOCAL_BA,         // local bundle adjustment (root)
    MAP_CULL_KF,          // temporal + redundant keyframe culling (root)

    // ── driver / fusion ─────────────────────────────────────────────────────
    VIDEO_DECODE,         // frame read from the sensor source (root)
    FUSION_PUSH,          // FusionModule::push (root)
    FUSION_GRAPH_UPDATE,  // GTSAM smoother update inside FUSION_PUSH

    COUNT
};

// Stable, human-readable stage name (used by the CSV dump and the viewer
// table). Never returns nullptr; unknown values yield "UNKNOWN".
const char* to_string(ProfileStage stage);

// Nesting metadata: the stage this one is measured INSIDE of. Root stages (the
// ones whose durations are disjoint and may be summed) map to themselves. Used
// by consumers to compute percentages and the "unaccounted" residual without
// re-encoding the call hierarchy.
ProfileStage parent_stage(ProfileStage stage);

// Accumulated timing of one stage. Plain POD so consumers (drivers, the debug
// viewer) never touch the profiler's internal atomics.
struct StageStat {
    unsigned long long count    = 0;    // number of samples added
    double             total_ms = 0.0;  // sum of sample durations
    double             last_ms  = 0.0;  // most recent sample
    double             max_ms   = 0.0;  // largest sample
};

// Process-wide, thread-partitioned stage accumulator.
//
// Samples are added into a THREAD-LOCAL accumulator with no locking, so the hot
// path is a relaxed atomic read (the enabled flag) plus a handful of relaxed
// atomic stores. Every thread-local accumulator is also owned by a global
// registry (shared_ptr), so a snapshot taken after a worker thread has exited
// still sees that thread's numbers.
//
// Disabled by default: with profiling off, ScopedTimer costs one relaxed atomic
// load and a branch.
class Profiler {
public:
    // Global on/off switch (read on every ScopedTimer construction).
    static void set_enabled(bool enabled);
    static bool enabled();

    // Names the calling thread in the snapshots. FIRST CALL WINS: later calls
    // from the same thread are ignored, so a per-frame call (e.g. at the top of
    // process_frame) cannot overwrite a label the owner set at start-up.
    // Threads that never call this are reported as "unlabeled".
    static void set_thread_label(const char* label);

    // Adds one sample to the calling thread's accumulator. No-op when disabled.
    static void add_sample(ProfileStage stage, double ms);

    // All threads merged, indexed by static_cast<size_t>(ProfileStage)
    // (size == ProfileStage::COUNT). count/total_ms are summed and max_ms is
    // the maximum across threads; last_ms is taken from the last registered
    // thread that has samples for the stage (each stage has a single producer
    // thread in this pipeline, so that is the producer's last sample).
    static std::vector<StageStat> snapshot_total();

    // Per-thread breakdown: (thread label, per-stage stats) in registration
    // order. Threads with no samples at all are omitted.
    static std::vector<std::pair<std::string, std::vector<StageStat>>> snapshot_by_thread();

    // Zeroes every registered accumulator (labels and registration are kept).
    static void reset();
};

// RAII stage timer: samples steady_clock on construction and reports the
// elapsed milliseconds to the Profiler on destruction. Does nothing at all when
// the profiler is disabled (checked once, at construction).
class ScopedTimer {
public:
    explicit ScopedTimer(ProfileStage stage);
    ~ScopedTimer();

    ScopedTimer(const ScopedTimer&)            = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    ProfileStage                          stage_;
    bool                                  active_;
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace util
}  // namespace uavloc
