#include "uavloc/new_vo/module/mapping_module.h"

#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/util/scoped_timer.h"

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {

MappingModule::MappingModule(data::MapDatabase* map_db, const VOConfig& config)
    : local_mapper_(map_db, config),
      async_enabled_(config.async_enabled),
      queue_threshold_(config.mapping_queue_threshold),
      wait_for_lba_(config.wait_for_local_bundle_adjustment) {}

MappingModule::~MappingModule() {
    stop();
}

void MappingModule::start() {
    if (!async_enabled_) {
        return;
    }
    if (started_.exchange(true)) {
        return;
    }
    terminate_ = false;
    thread_ = std::make_unique<std::thread>(&MappingModule::run, this);
    spdlog::info("MappingModule: async mapping thread started");
}

void MappingModule::stop() {
    if (!started_.exchange(false)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_queue_);
        terminate_ = true;
    }
    cv_.notify_all();
    // Wake any pause_and_wait() blocked on the pause condition (its predicate
    // also checks terminate_) so shutdown can never deadlock against a pause.
    pause_cv_.notify_all();
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
    thread_.reset();
    // Fulfill any promises left in the queue so no waiter blocks forever.
    std::lock_guard<std::mutex> lock(mtx_queue_);
    for (auto& item : queue_) {
        if (item.second) {
            item.second->set_value();
        }
    }
    queue_.clear();
    spdlog::info("MappingModule: async mapping thread stopped");
}

void MappingModule::submit(const std::shared_ptr<data::Keyframe>& keyfrm, ReplacedLandmarks& replaced_lms) {
    replaced_lms.clear();
    if (!keyfrm) {
        return;
    }
    if (async_enabled_) {
        auto future_add_keyframe = async_add_keyframe(keyfrm);
        if (wait_for_lba_) {
            // stella keyframe_inserter.cc:223-225: tracking blocks until the
            // mapping thread has fully processed this keyframe. The promise is
            // fulfilled on every path (run() after map(), stop() queue-flush,
            // pause_and_wait() queue-discard), so this cannot deadlock.
            future_add_keyframe.get();
            // The keyframe's fusion merges are now pending; hand them to the
            // caller immediately so the tracker applies them this frame
            // instead of one frame late.
            drain_replaced_lms(replaced_lms);
        }
        return;
    }
    // Synchronous: run inline. abort_local_BA_ stays false (no concurrent load).
    // No backpressure applies in sync mode, so both new mechanisms are no-ops:
    // never skip local BA and never abort landmark generation.
    abort_local_BA_ = false;
    local_mapper_.map(keyfrm, replaced_lms, &abort_local_BA_,
                      /*skip_local_ba=*/false, /*abort_landmark_gen=*/{});
}

void MappingModule::drain_replaced_lms(ReplacedLandmarks& out) {
    std::lock_guard<std::mutex> lock(mtx_replaced_);
    out = std::move(pending_replaced_lms_);
    pending_replaced_lms_.clear();
}

std::shared_future<void> MappingModule::async_add_keyframe(const std::shared_ptr<data::Keyframe>& keyfrm) {
    auto promise = std::make_shared<std::promise<void>>();
    std::shared_future<void> future = promise->get_future().share();
    {
        std::lock_guard<std::mutex> lock(mtx_queue_);
        queue_.emplace_back(keyfrm, promise);
        // A newer keyframe aborts an in-progress local bundle adjustment.
        abort_local_BA_ = true;
    }
    cv_.notify_one();
    return future;
}

bool MappingModule::is_skipping_localBA() const {
    if (!async_enabled_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mtx_queue_);
    return queue_.size() > queue_threshold_;
}

bool MappingModule::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_queue_);
    return !queue_.empty();
}

std::shared_future<void> MappingModule::async_pause() {
    pause_requested_ = true;
    auto promise = std::make_shared<std::promise<void>>();
    // In sync mode (or when no thread is running) the module is effectively
    // paused immediately.
    if (!async_enabled_ || !started_) {
        is_paused_ = true;
    }
    promise->set_value();
    return promise->get_future().share();
}

void MappingModule::pause_and_wait() {
    // Sync mode / thread not started: mapping runs inline on the caller, so it
    // is already idle. Keep this a strict no-op to preserve deterministic sync
    // behaviour.
    if (!async_enabled_ || !started_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_queue_);
        pause_requested_ = true;
        // Abort any in-flight local BA so map() returns promptly.
        abort_local_BA_ = true;
        // Discard pending keyframes and fulfil their promises (mirror stop()).
        for (auto& item : queue_) {
            if (item.second) {
                item.second->set_value();
            }
        }
        queue_.clear();
    }
    cv_.notify_all();

    // Block until the mapping thread is genuinely idle: parked at the loop top
    // (is_paused_) and not inside LocalMapper::map() (!processing_).
    std::unique_lock<std::mutex> lock(mtx_queue_);
    pause_cv_.wait(lock, [this] { return terminate_ || (is_paused_ && !processing_); });
}

void MappingModule::reset_state() {
    // Sync mode: mapping runs inline and never accumulates cross-reset scratch
    // state on a background thread — keep this a strict no-op to preserve the
    // deterministic baseline.
    if (!async_enabled_ || !started_) {
        return;
    }
    // Caller must have made the mapping thread idle (pause_and_wait) first, so
    // touching local_mapper_ here is race-free.
    local_mapper_.reset();
    std::lock_guard<std::mutex> lock(mtx_replaced_);
    pending_replaced_lms_.clear();
}

bool MappingModule::pause_is_requested() const {
    return pause_requested_;
}

bool MappingModule::is_paused() const {
    return is_paused_;
}

void MappingModule::resume() {
    pause_requested_ = false;
    is_paused_ = false;
    cv_.notify_one();
}

void MappingModule::run() {
    util::Profiler::set_thread_label("mapping");
    while (true) {
        QueueItem item;
        {
            std::unique_lock<std::mutex> lock(mtx_queue_);
            // Park at the loop top while there is nothing to do or a pause is
            // requested. Signal pause_cv_ once we are genuinely idle so that
            // pause_and_wait() can proceed knowing no map() is in flight.
            while (!terminate_ && (queue_.empty() || pause_requested_)) {
                if (pause_requested_ && !is_paused_) {
                    is_paused_ = true;
                    pause_cv_.notify_all();
                }
                util::ScopedTimer _t(util::ProfileStage::MAP_QUEUE_WAIT);
                cv_.wait(lock);
            }
            if (terminate_) {
                // Respond to termination even while paused; fulfil the
                // outstanding waiter (if any) below via stop()'s drain.
                break;
            }
            item = std::move(queue_.front());
            queue_.pop_front();
            // Fresh keyframe about to be mapped: allow its local BA to run.
            abort_local_BA_ = false;
            processing_ = true;
        }

        // Async backpressure: skip local BA when the queue is backed up, and let
        // landmark generation interrupt itself once a newer keyframe is queued so
        // the mapping queue can drain (mirror stella_vslam::mapping_module).
        const bool skip_local_ba = is_skipping_localBA();

        ReplacedLandmarks replaced_lms;
        try {
            local_mapper_.map(item.first, replaced_lms, &abort_local_BA_,
                              skip_local_ba,
                              [this] { return keyframe_is_queued(); });
        }
        catch (const std::exception& e) {
            spdlog::error("MappingModule: local mapping threw: {}", e.what());
        }

        {
            std::lock_guard<std::mutex> lock(mtx_queue_);
            processing_ = false;
        }
        // A pause may have been requested mid-map(); wake pause_and_wait() so it
        // re-checks the predicate (it will then wait for is_paused_ at the top).
        pause_cv_.notify_all();

        merge_replaced_lms(replaced_lms);

        if (item.second) {
            item.second->set_value();
        }
    }
}

void MappingModule::merge_replaced_lms(const ReplacedLandmarks& replaced) {
    if (replaced.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_replaced_);
    for (const auto& pair : replaced) {
        const auto& old_lm = pair.first;
        auto final_lm = pair.second;
        // Follow existing chains so pending stays single-step (flattened).
        auto it = pending_replaced_lms_.find(final_lm);
        while (it != pending_replaced_lms_.end() && it->second != final_lm) {
            final_lm = it->second;
            it = pending_replaced_lms_.find(final_lm);
        }
        pending_replaced_lms_[old_lm] = final_lm;
        // Redirect existing entries that pointed at old_lm.
        for (auto& kv : pending_replaced_lms_) {
            if (kv.second == old_lm) {
                kv.second = final_lm;
            }
        }
    }
}

}} // namespace vo // namespace uavloc
