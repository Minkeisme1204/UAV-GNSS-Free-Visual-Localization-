#pragma once

// MappingModule — orchestrator for the new_vo local-mapping step.
//
// Owns a module::LocalMapper and dispatches freshly inserted keyframes to it.
// Two modes, selected by VOConfig::async_enabled:
//   * synchronous (default): submit() runs LocalMapper::map inline on the
//     calling (tracking) thread and returns the fusion replacements — the
//     deterministic baseline.
//   * asynchronous: submit() enqueues the keyframe and returns immediately; a
//     dedicated mapping thread consumes the bounded queue and runs
//     LocalMapper::map. A newer keyframe aborts an in-progress local bundle
//     adjustment (abort_local_BA_) to bound latency.
//
// KeyframeInserter holds a MappingModule* and queries the pause / skip-localBA
// interface; the actual submission is driven by VOModule (so it can apply the
// fusion replacements to the tracker's last frame).

#include "uavloc/new_vo/module/local_mapper.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace uavloc {
namespace vo {

namespace data {
class Keyframe;
class MapDatabase;
} // namespace data

struct VOConfig;

class MappingModule {
public:
    using ReplacedLandmarks = module::LocalMapper::ReplacedLandmarks;

    MappingModule(data::MapDatabase* map_db, const VOConfig& config);

    ~MappingModule();

    //! Spawn the mapping thread (async mode only; no-op in sync mode).
    void start();

    //! Terminate and join the mapping thread (safe to call multiple times).
    void stop();

    //! Dispatch a new keyframe to local mapping.
    //! Sync: run LocalMapper::map inline; replaced_lms holds the fusion merges.
    //! Async: enqueue and return immediately; replaced_lms is left empty (use
    //! drain_replaced_lms() to collect merges produced on the mapping thread).
    //! Async + wait_for_local_bundle_adjustment: block until the mapping thread
    //! has fully processed this keyframe (stella keyframe_inserter.cc:223
    //! future_add_keyframe.get()), then drain the fusion replacements into
    //! replaced_lms so the caller can apply them immediately.
    void submit(const std::shared_ptr<data::Keyframe>& keyfrm, ReplacedLandmarks& replaced_lms);

    //! Move out the fusion replacements accumulated by the async mapping thread.
    void drain_replaced_lms(ReplacedLandmarks& out);

    //! Add a Keyframe to the mapping queue asynchronously
    std::shared_future<void> async_add_keyframe(const std::shared_ptr<data::Keyframe>& keyfrm);

    //! Whether the mapping module is skipping local bundle adjustment
    bool is_skipping_localBA() const;

    //! Whether at least one keyframe is waiting in the mapping queue. Polled by
    //! the async landmark-generation loop as a backpressure signal so it can
    //! interrupt itself and let the queue drain.
    bool keyframe_is_queued() const;

    //! Request an asynchronous pause and get a future that resolves when paused
    std::shared_future<void> async_pause();

    //! Request a pause and BLOCK until the mapping thread is genuinely idle
    //! (parked at the loop top, not inside LocalMapper::map). Any in-flight
    //! local BA is aborted and the pending queue is discarded (promises
    //! fulfilled). No-op in sync mode / when the thread is not started. Used by
    //! VOModule to safely reset the map on LOST -> re-init.
    void pause_and_wait();

    //! Drop internal mapping state that references map objects about to be
    //! erased: the LocalMapper's pending "fresh landmark" queue and the
    //! accumulated fusion replacements. MUST be called while the mapping thread
    //! is idle (i.e. after pause_and_wait()). No-op in sync mode so the
    //! deterministic baseline is preserved.
    void reset_state();

    //! Whether a pause has been requested
    bool pause_is_requested() const;

    //! Whether the mapping module is currently paused
    bool is_paused() const;

    //! Resume the mapping module
    void resume();

private:
    //! Mapping-thread loop: consume the keyframe queue.
    void run();

    //! Merge async fusion replacements into pending_replaced_lms_ (flattened).
    void merge_replaced_lms(const ReplacedLandmarks& replaced);

    module::LocalMapper local_mapper_;

    const bool         async_enabled_;
    const unsigned int queue_threshold_;
    //! stella wait_for_local_bundle_adjustment: per-keyframe tracking↔mapping
    //! handshake — submit() blocks until the mapping thread finished the KF.
    const bool         wait_for_lba_;

    using QueueItem = std::pair<std::shared_ptr<data::Keyframe>, std::shared_ptr<std::promise<void>>>;
    std::deque<QueueItem>   queue_;
    mutable std::mutex      mtx_queue_;
    std::condition_variable cv_;
    //! Signalled by the mapping thread when it becomes idle/parked so that
    //! pause_and_wait() can block until mapping is genuinely quiescent.
    std::condition_variable pause_cv_;
    std::unique_ptr<std::thread> thread_;

    ReplacedLandmarks  pending_replaced_lms_;
    std::mutex         mtx_replaced_;

    //! aborts an in-progress local bundle adjustment (read by g2o via bool*)
    bool               abort_local_BA_ = false;

    std::atomic<bool>  pause_requested_{false};
    std::atomic<bool>  is_paused_{false};
    //! true only while the mapping thread is inside LocalMapper::map(); guarded
    //! by mtx_queue_ (written by run(), read by pause_and_wait()'s predicate).
    bool               processing_ = false;
    std::atomic<bool>  terminate_{false};
    std::atomic<bool>  started_{false};
};

}} // namespace vo // namespace uavloc
