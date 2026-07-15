#include "uavloc/fusion/fusion_module.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <spdlog/spdlog.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace uavloc::fusion {

namespace {

// F1 skeleton placeholders — named constants (config-free by design; the real
// health state machine and backpressure policy land with the real factors).

//! Placeholder health rule: INITIALIZING until this many keyframes are in the
//! graph, then CONVERGED (kcb §2.3 state machine arrives in a later step).
constexpr unsigned int HEALTH_CONVERGED_MIN_KEYFRAMES = 5;

//! Soft queue-depth threshold: the queue never drops items (keyframes are
//! required by the graph), but growth beyond this depth is logged (once per
//! doubling — rate-limited).
constexpr std::size_t QUEUE_SOFT_WARN_DEPTH = 32;

constexpr double MSEC_TO_SEC = 1e-3;
constexpr double DEG_TO_RAD  = M_PI / 180.0;

using gtsam::symbol_shorthand::X;

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

struct FusionModule::Impl {
    explicit Impl(const FusionConfig& config)
        : config_(config) {}

    ~Impl() {
        stop();
    }

    // ── Threading (mirrors MappingModule) ─────────────────────────────────

    void start() {
        if (!config_.async_enabled) {
            return;
        }
        if (started_.exchange(true)) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mtx_queue_);
            terminate_ = false;
        }
        thread_ = std::make_unique<std::thread>(&Impl::run, this);
        spdlog::info("FusionModule: async fusion thread started");
    }

    void stop() {
        if (!started_.exchange(false)) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mtx_queue_);
            terminate_ = true;
        }
        cv_.notify_all();
        if (thread_ && thread_->joinable()) {
            thread_->join();
        }
        thread_.reset();
        spdlog::info("FusionModule: async fusion thread stopped");
    }

    void push(const vo::VOResult& res, const sensor::TelemetryData& telem) {
        WorkItem item{res, telem};
        if (config_.async_enabled && started_.load()) {
            std::size_t depth = 0;
            {
                std::lock_guard<std::mutex> lock(mtx_queue_);
                queue_.push_back(std::move(item));
                depth = queue_.size();
                if (depth >= next_queue_warn_depth_) {
                    spdlog::warn("FusionModule: work queue depth {} (soft threshold {}) — "
                                 "fusion thread is falling behind",
                                 depth, next_queue_warn_depth_);
                    next_queue_warn_depth_ *= 2;
                }
            }
            cv_.notify_one();
            return;
        }
        // Synchronous mode (or async not started): run inline on the caller's
        // thread — the deterministic baseline.
        process(item);
    }

    void add_result_callback(std::function<void(const FusionResult&)> cb) {
        if (!cb) {
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_callbacks_);
        callbacks_.push_back(std::move(cb));
    }

    FusionResult latest() const {
        std::lock_guard<std::mutex> lock(mtx_latest_);
        return latest_;
    }

private:
    //! Fusion-thread loop: consume the work queue. On terminate, drain the
    //! remaining items before exiting (bounded: just finish the queue).
    void run() {
        for (;;) {
            WorkItem item;
            {
                std::unique_lock<std::mutex> lock(mtx_queue_);
                cv_.wait(lock, [this] { return terminate_ || !queue_.empty(); });
                if (queue_.empty()) {
                    if (terminate_) {
                        break;  // drained — safe to exit
                    }
                    continue;   // spurious wakeup
                }
                item = std::move(queue_.front());
                queue_.pop_front();
            }
            process(item);
        }
    }

    // ── Worker step (shared by sync inline and async thread) ──────────────

    struct WorkItem {
        vo::VOResult          res;
        sensor::TelemetryData telem;
    };

    void process(const WorkItem& item) {
        FusionResult out;
        out.frame_id       = item.res.frame_id;
        out.timestamp_msec = item.res.timestamp_msec;
        // Placeholders until S(k) / θ / b(k) become graph variables (later F1 step).
        out.scale      = 1.0;
        out.theta_deg  = config_.theta_init_deg;
        out.agl_bias_m = 0.0;
        out.health     = currentHealth();

        // LOST / re-init frames: skip the graph, still emit an invalid result.
        if (!item.res.has_pose) {
            out.has_pose = false;
            publish(out);
            return;
        }

        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.matrix() = item.res.T_wc;

        if (!initialized_) {
            // First keyframe with valid telemetry anchors the graph.
            if (item.res.is_keyframe && item.telem.valid) {
                initialize(item, T_wc);
                out.has_pose         = true;
                out.T_enu_c          = T_enu_c_last_kf_;
                out.base_keyframe_id = last_kf_frame_id_;
                out.graph_updated    = true;
            } else {
                out.has_pose = false;
            }
        } else if (item.res.is_keyframe) {
            addKeyframe(item, T_wc);
            out.has_pose         = true;
            out.T_enu_c          = T_enu_c_last_kf_;
            out.base_keyframe_id = last_kf_frame_id_;
            out.graph_updated    = true;
        } else {
            // Non-keyframe: propagate from the last optimized state — cheap,
            // no graph touch. T_enu_c(i) = X(k_last) * T_rel(k → i).
            const Eigen::Isometry3d T_rel = T_wc_last_kf_.inverse() * T_wc;
            out.T_enu_c          = T_enu_c_last_kf_ * T_rel;
            out.has_pose         = true;
            out.base_keyframe_id = last_kf_frame_id_;
        }

        out.health = currentHealth();
        publish(out);
    }

    //! First keyframe with valid telemetry: anchor X(0) and prove GTSAM
    //! linkage end-to-end with one smoother update.
    void initialize(const WorkItem& item, const Eigen::Isometry3d& T_wc) {
        T_enu_from_w_ = enuFromVoWorld(item.telem);

        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.lag_seconds);

        const Eigen::Isometry3d T_enu_c = T_enu_from_w_ * T_wc;
        const gtsam::Pose3      pose0(T_enu_c.matrix());
        const gtsam::Key        key = X(0);

        // Anchor prior on X(0): weak X/Y (prevents indeterminacy pre-VPR),
        // AGL-grade Z, telemetry-attitude-grade rotation.
        // Pose3 tangent order: (rx, ry, rz, x, y, z).
        gtsam::Vector6 sigmas;
        sigmas << config_.rollpitch_sigma_rad, config_.rollpitch_sigma_rad,
                  config_.rollpitch_sigma_rad, config_.anchor_xy_sigma_m,
                  config_.anchor_xy_sigma_m, config_.agl_sigma_m;

        gtsam::NonlinearFactorGraph graph;
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            key, pose0, gtsam::noiseModel::Diagonal::Sigmas(sigmas));

        gtsam::Values values;
        values.insert(key, pose0);

        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        stamps[key] = item.res.timestamp_msec * MSEC_TO_SEC;

        smoother_->update(graph, values, stamps);

        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(key).matrix();
        T_wc_last_kf_             = T_wc;
        last_kf_frame_id_         = item.res.frame_id;
        next_key_                 = 1;
        keyframe_count_           = 1;
        initialized_              = true;

        spdlog::info("FusionModule: initialized — X(0) anchored at frame {} (t={:.3f}s)",
                     item.res.frame_id, item.res.timestamp_msec * MSEC_TO_SEC);
    }

    //! Subsequent keyframes: X(k) chained to X(k-1) by the raw VO relative
    //! pose. PLACEHOLDER for ScaledVOFactor(X(k-1), X(k), S(k)) — the real
    //! factor set (scale, delta-yaw, roll/pitch, AGL) lands in a later F1 step.
    void addKeyframe(const WorkItem& item, const Eigen::Isometry3d& T_wc) {
        const unsigned int k = next_key_;

        const Eigen::Isometry3d T_rel = T_wc_last_kf_.inverse() * T_wc;
        const gtsam::Pose3      between(T_rel.matrix());

        gtsam::Vector6 sigmas;
        const double rot_sigma = config_.vo_rot_sigma_deg * DEG_TO_RAD;
        sigmas << rot_sigma, rot_sigma, rot_sigma,
                  config_.vo_trans_sigma_m, config_.vo_trans_sigma_m,
                  config_.vo_trans_sigma_m;

        gtsam::NonlinearFactorGraph graph;
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(k - 1), X(k), between, gtsam::noiseModel::Diagonal::Sigmas(sigmas));

        gtsam::Values values;
        const gtsam::Pose3 guess = gtsam::Pose3(T_enu_c_last_kf_.matrix()) * between;
        values.insert(X(k), guess);

        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        stamps[X(k)] = item.res.timestamp_msec * MSEC_TO_SEC;

        smoother_->update(graph, values, stamps);

        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(X(k)).matrix();
        T_wc_last_kf_             = T_wc;
        last_kf_frame_id_         = item.res.frame_id;
        next_key_                 = k + 1;
        ++keyframe_count_;

        spdlog::debug("FusionModule: keyframe X({}) inserted (frame {}, {} KFs in graph history)",
                      k, item.res.frame_id, keyframe_count_);
    }

    //! VO-world → ENU alignment from telemetry at initialization.
    //! TODO(F1 later step): build the real rotation chain here —
    //! R_enu_w = Rz(heading + mount-azimuth θ) ∘ gimbal tilt ∘ camera-axes
    //! permutation (see .docs/theory/heading_agl_prior_tactics.md §7 and
    //! .docs/designs/geo_referencing_latlon.md). Identity placeholder for the
    //! skeleton so the plumbing/threading can be verified independently.
    static Eigen::Isometry3d enuFromVoWorld(const sensor::TelemetryData& /*telem*/) {
        return Eigen::Isometry3d::Identity();
    }

    FusionHealth currentHealth() const {
        if (keyframe_count_ >= HEALTH_CONVERGED_MIN_KEYFRAMES) {
            return FusionHealth::CONVERGED;
        }
        return FusionHealth::INITIALIZING;
    }

    //! Store the newest result and fire the callbacks in registration order
    //! (on the calling thread of process()).
    void publish(const FusionResult& out) {
        {
            std::lock_guard<std::mutex> lock(mtx_latest_);
            latest_ = out;
        }
        std::vector<std::function<void(const FusionResult&)>> cbs;
        {
            std::lock_guard<std::mutex> lock(mtx_callbacks_);
            cbs = callbacks_;
        }
        for (const auto& cb : cbs) {
            cb(out);
        }
    }

    // ── Members ────────────────────────────────────────────────────────────

    const FusionConfig config_;

    // Work queue (async mode). Never drops items — keyframes are required by
    // the graph.
    std::deque<WorkItem>         queue_;
    mutable std::mutex           mtx_queue_;
    std::condition_variable      cv_;
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool>            started_{false};
    bool                         terminate_ = false;  // guarded by mtx_queue_
    std::size_t                  next_queue_warn_depth_ = QUEUE_SOFT_WARN_DEPTH;

    // Subscribers.
    std::vector<std::function<void(const FusionResult&)>> callbacks_;
    std::mutex                                            mtx_callbacks_;

    // Newest result snapshot.
    FusionResult       latest_;
    mutable std::mutex mtx_latest_;

    // Graph state — touched only by process() (single consumer: the fusion
    // thread in async mode, the caller's thread in sync mode).
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    bool              initialized_    = false;
    unsigned int      next_key_       = 0;  //!< index k of the next X(k)
    unsigned int      keyframe_count_ = 0;
    unsigned int      last_kf_frame_id_ = 0;
    Eigen::Isometry3d T_wc_last_kf_    = Eigen::Isometry3d::Identity();  //!< VO pose at last KF
    Eigen::Isometry3d T_enu_c_last_kf_ = Eigen::Isometry3d::Identity();  //!< optimized X(k_last)
    Eigen::Isometry3d T_enu_from_w_    = Eigen::Isometry3d::Identity();  //!< VO world → ENU
};

// ─────────────────────────────────────────────────────────────────────────────

FusionModule::FusionModule(const FusionConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

FusionModule::~FusionModule() = default;

void FusionModule::start() {
    impl_->start();
}

void FusionModule::stop() {
    impl_->stop();
}

void FusionModule::push(const vo::VOResult& res, const sensor::TelemetryData& telem) {
    impl_->push(res, telem);
}

void FusionModule::add_result_callback(std::function<void(const FusionResult&)> cb) {
    impl_->add_result_callback(std::move(cb));
}

FusionResult FusionModule::latest() const {
    return impl_->latest();
}

} // namespace uavloc::fusion
