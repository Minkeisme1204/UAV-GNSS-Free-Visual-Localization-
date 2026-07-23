#include "uavloc/fusion/fusion_module.h"

#include "factors.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
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

//! Placeholder health rule: INITIALIZING until this many keyframes are in the
//! graph, then CONVERGED (kcb §2.3 state machine arrives in a later step).
constexpr unsigned int HEALTH_CONVERGED_MIN_KEYFRAMES = 5;

//! Soft queue-depth threshold: the queue never drops items (keyframes are
//! required by the graph), but growth beyond this depth is logged (once per
//! doubling — rate-limited).
constexpr std::size_t QUEUE_SOFT_WARN_DEPTH = 32;

//! Effectively-unconstrained sigma for the tangent components a partial prior
//! must not touch (kcb pattern: AHRS quat prior uses rot sigmas + 1e9 trans).
//! A convention of "infinity", not a mission tunable.
constexpr double LOOSE_SIGMA = 1e9;

constexpr double MSEC_TO_SEC = 1e-3;
constexpr double DEG_TO_RAD  = M_PI / 180.0;
constexpr double RAD_TO_DEG  = 180.0 / M_PI;

// State keys (gtsam::Symbol): x(k) Pose3 T_enu_camera, s(k) VO scale
// multiplier, b(k) AGL-vs-ENU-Z bias, t(0) mount azimuth θ [rad] (single
// constant key — rides on its prior until VPR makes it observable, F2).
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::S;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::X;

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

struct FusionModule::Impl {
    explicit Impl(const FusionConfig& config)
        : config_(config),
          theta_est_rad_(config.theta_init_deg * DEG_TO_RAD) {}

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

    std::vector<FusionLagPose> getLagWindow() const {
        std::lock_guard<std::mutex> lock(mtx_latest_);
        return lag_window_;
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

        // Re-init boundary tracking: any non-TRACKING result (LOST or
        // NOT_INITIALIZED) seen since the previous keyframe marks the next
        // keyframe as a post-reinit boundary. VOModule welds pose continuity
        // across the gap and re-seeds its metric scale from altitude, so the
        // cross-boundary factors must be handled specially (see addKeyframe).
        if (item.res.state != vo::VOTrackingState::TRACKING) {
            saw_non_tracking_since_kf_ = true;
        }

        // LOST / re-init frames: skip the graph, still emit an invalid result.
        if (!item.res.has_pose) {
            out.has_pose = false;
            fillAuxStates(out);
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
            // no graph touch. T_enu_c(i) = X(k_last) ∘ (R_rel, s·t_rel) —
            // the current scale estimate applies to the VO translation
            // increment (same model as the ScaledVOFactor).
            Eigen::Isometry3d T_rel = T_wc_last_kf_.inverse() * T_wc;
            T_rel.translation()  *= scale_est_;
            out.T_enu_c          = T_enu_c_last_kf_ * T_rel;
            out.has_pose         = true;
            out.base_keyframe_id = last_kf_frame_id_;
        }

        fillAuxStates(out);
        publish(out);
    }

    //! Fill the auxiliary graph states (current estimates) + health.
    void fillAuxStates(FusionResult& out) const {
        out.scale      = scale_est_;
        out.agl_bias_m = bias_est_;
        out.theta_deg  = theta_est_rad_ * RAD_TO_DEG;
        out.health     = currentHealth();
    }

    //! Measured camera orientation in ENU from a telemetry sample (shared by
    //! the X(0) anchor, the per-keyframe attitude prior and the ENU init —
    //! one chain, one convention; see factors.h).
    static Eigen::Matrix3d measuredRotationEnuCamera(const sensor::TelemetryData& telem) {
        return rotation_enu_camera(telem.roll_deg, telem.pitch_deg, telem.heading_deg,
                                   telem.gimbal_pan_deg, telem.gimbal_tilt_deg);
    }

    //! Heading + gimbal-pan azimuth ψ [rad] used by the delta-yaw factor.
    static double psiRad(const sensor::TelemetryData& telem) {
        return (telem.heading_deg + telem.gimbal_pan_deg) * DEG_TO_RAD;
    }

    //! First keyframe with valid telemetry: anchor the graph.
    //! X(0) sits at ENU position (0, 0, agl_0) with the telemetry-measured
    //! rotation; s(0), b(0) and the constant mount-azimuth θ = t(0) get their
    //! priors here.
    void initialize(const WorkItem& item, const Eigen::Isometry3d& T_wc) {
        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.lag_seconds);

        const Eigen::Matrix3d R_enu_c = measuredRotationEnuCamera(item.telem);
        const double          agl_0   = item.telem.altitude_m;
        const double          t0_sec  = item.res.timestamp_msec * MSEC_TO_SEC;
        const double theta_rad        = config_.theta_init_deg * DEG_TO_RAD;
        const double theta_sigma_rad  = config_.theta_sigma_deg * DEG_TO_RAD;

        Eigen::Isometry3d T_anchor = Eigen::Isometry3d::Identity();
        T_anchor.linear()      = R_enu_c;
        T_anchor.translation() = Eigen::Vector3d(0.0, 0.0, agl_0);

        // VO world → ENU alignment: chosen so the initial estimate of X(0)
        // equals the anchor pose. The VO world is the first camera frame
        // (T_wc = I at the VO origin), for which this reduces to
        // T_enu_w = (R_enu_c_meas, (0, 0, agl_0)).
        T_enu_from_w_ = T_anchor * T_wc.inverse();

        const gtsam::Pose3 pose0(T_anchor.matrix());

        // Anchor prior on X(0). Pose3 tangent order: (rx, ry, rz, x, y, z).
        // Rotation: roll/pitch at telemetry-attitude grade; the yaw component
        // (index 2 — camera z is near-vertical for a near-nadir camera) is as
        // uncertain as the mount azimuth θ, since heading + θ set absolute yaw.
        // Translation: weak X/Y (prevents indeterminacy pre-VPR), AGL-grade Z.
        gtsam::Vector6 sigmas;
        sigmas << config_.rollpitch_sigma_rad, config_.rollpitch_sigma_rad,
                  theta_sigma_rad, config_.anchor_xy_sigma_m,
                  config_.anchor_xy_sigma_m, config_.agl_sigma_m;

        gtsam::NonlinearFactorGraph graph;
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            X(0), pose0, gtsam::noiseModel::Diagonal::Sigmas(sigmas));

        // s(0): VO is metric-seeded from altitude at init, so the scale
        // multiplier starts at exactly 1.
        graph.emplace_shared<gtsam::PriorFactor<double>>(
            S(0), 1.0, gtsam::noiseModel::Isotropic::Sigma(1, config_.scale_prior_sigma));

        // b(0): AGL-vs-ENU-Z bias starts at 0 (the ENU origin height is
        // defined by the init AGL itself).
        graph.emplace_shared<gtsam::PriorFactor<double>>(
            B(0), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, config_.agl_bias_prior_sigma_m));

        // θ = t(0): mount azimuth, constant. Participates in NO other factor
        // in F1 (F2 wires it once VPR X/Y fixes make it observable).
        graph.emplace_shared<gtsam::PriorFactor<double>>(
            T(0), theta_rad, gtsam::noiseModel::Isotropic::Sigma(1, theta_sigma_rad));

        // AGL measurement on the very first state (gated like every keyframe).
        if (item.telem.altitude_m > 0.0) {
            graph.emplace_shared<AglFactor>(
                X(0), B(0), item.telem.altitude_m,
                gtsam::noiseModel::Isotropic::Sigma(1, config_.agl_sigma_m));
        }

        gtsam::Values values;
        values.insert(X(0), pose0);
        values.insert(S(0), 1.0);
        values.insert(B(0), 0.0);
        values.insert(T(0), theta_rad);

        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        stamps[X(0)] = t0_sec;
        stamps[S(0)] = t0_sec;
        stamps[B(0)] = t0_sec;
        stamps[T(0)] = t0_sec;

        smoother_->update(graph, values, stamps);

        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(X(0)).matrix();
        scale_est_                = smoother_->calculateEstimate<double>(S(0));
        bias_est_                 = smoother_->calculateEstimate<double>(B(0));
        theta_est_rad_            = smoother_->calculateEstimate<double>(T(0));

        T_wc_last_kf_        = T_wc;
        last_kf_frame_id_    = item.res.frame_id;
        next_key_            = 1;
        keyframe_count_      = 1;
        last_kf_psi_rad_     = psiRad(item.telem);
        last_kf_telem_valid_ = true;  // initialize() requires valid telemetry
        saw_non_tracking_since_kf_ = false;
        initialized_         = true;

        kf_registry_.clear();
        kf_registry_.push_back({0, item.res.frame_id});
        rebuildLagWindow();

        spdlog::info("FusionModule: initialized — X(0) anchored at frame {} "
                     "(t={:.3f}s, agl={:.1f} m, heading={:.1f}°)",
                     item.res.frame_id, t0_sec, agl_0, item.telem.heading_deg);
    }

    //! Subsequent keyframes: the F1 factor set —
    //!   1. ScaledVOFactor(x(k-1), x(k), s(k)) — VO relative pose, Huber;
    //!   2. scale + bias random walks (always added: keeps the chains
    //!      connected even when telemetry factors are gated out);
    //!   3. DeltaYawFactor(x(k-1), x(k)) — offset-immune heading delta,
    //!      gated on telemetry validity at both ends + slew rate;
    //!   4. rotation-only attitude prior (roll/pitch tight, yaw loose —
    //!      absolute heading enters ONLY via the delta factor, θ unknown);
    //!   5. AglFactor(x(k), b(k)) — gated on validity + altitude > 0.
    //!
    //! Post-reinit boundary keyframes (a non-TRACKING VO result was seen since
    //! the previous keyframe) deviate from the table: the ScaledVOFactor
    //! translation sigma is inflated (the welded pose across the LOST gap is
    //! an assumption, not a measurement), the s(k-1)→s(k) walk is replaced by
    //! a fresh s(k)=1 prior (VO re-seeded metric scale from altitude), and the
    //! delta-yaw factor is skipped (heading during the blind gap is unknown
    //! motion — the wrapped Δψ could alias). The b(k) bias chain is kept: the
    //! AGL-vs-ENU-Z bias is a telemetry property, unaffected by VO re-init.
    void addKeyframe(const WorkItem& item, const Eigen::Isometry3d& T_wc) {
        const unsigned int k     = next_key_;
        const double       t_sec = item.res.timestamp_msec * MSEC_TO_SEC;
        const bool reinit_boundary = saw_non_tracking_since_kf_;

        const Eigen::Isometry3d T_rel = T_wc_last_kf_.inverse() * T_wc;
        const gtsam::Pose3      rel(T_rel.matrix());

        gtsam::NonlinearFactorGraph graph;

        // 1. Scaled VO relative-pose factor (Huber-robust). At a boundary the
        //    translation sigma is inflated (rotation sigma unchanged).
        {
            gtsam::Vector6 sigmas;
            const double rot_sigma   = config_.vo_rot_sigma_deg * DEG_TO_RAD;
            const double trans_sigma = config_.vo_trans_sigma_m *
                (reinit_boundary ? config_.reinit_trans_inflation : 1.0);
            sigmas << rot_sigma, rot_sigma, rot_sigma,
                      trans_sigma, trans_sigma, trans_sigma;
            const auto vo_noise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k),
                gtsam::noiseModel::Diagonal::Sigmas(sigmas));
            graph.emplace_shared<ScaledVOFactor>(X(k - 1), X(k), S(k), rel, vo_noise);
        }

        // 2. Scale: random walk within a VO segment; at a boundary the VO
        //    scale may jump discontinuously, so restart with a fresh s(k)=1
        //    prior instead of chaining across the weld.
        if (reinit_boundary) {
            graph.emplace_shared<gtsam::PriorFactor<double>>(
                S(k), 1.0,
                gtsam::noiseModel::Isotropic::Sigma(1, config_.scale_prior_sigma));
        } else {
            graph.emplace_shared<gtsam::BetweenFactor<double>>(
                S(k - 1), S(k), 0.0,
                gtsam::noiseModel::Isotropic::Sigma(1, config_.scale_walk_sigma));
        }
        //    AGL-bias walk: kept across boundaries (telemetry property).
        graph.emplace_shared<gtsam::BetweenFactor<double>>(
            B(k - 1), B(k), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, config_.agl_bias_walk_m));

        // 3. Delta-yaw factor (offset-immune heading constraint). Skipped at
        //    a boundary: the heading step spans the blind LOST gap (unknown
        //    motion) and the angle wrap could alias.
        if (!reinit_boundary && item.telem.valid && last_kf_telem_valid_) {
            const double psi_k  = psiRad(item.telem);
            const double d_psi  = wrap_angle(psi_k - last_kf_psi_rad_);
            const double max_step = config_.delta_yaw_max_step_deg * DEG_TO_RAD;
            if (std::abs(d_psi) <= max_step) {
                // heading is clockwise-from-North, ENU yaw is CCW-from-East:
                // the measured ENU-yaw increment is -Δψ (see factors.h).
                const auto dy_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k),
                    gtsam::noiseModel::Isotropic::Sigma(1, config_.delta_yaw_sigma_rad));
                graph.emplace_shared<DeltaYawFactor>(X(k - 1), X(k),
                                                     wrap_angle(-d_psi), dy_noise);
            } else {
                spdlog::debug("FusionModule: delta-yaw gated out at X({}) — "
                              "|Δψ|={:.1f}° exceeds {:.1f}°",
                              k, std::abs(d_psi) * RAD_TO_DEG,
                              config_.delta_yaw_max_step_deg);
            }
        }

        // 4. Roll/pitch attitude prior (rotation-only; yaw + translation
        //    loose — Pose3 tangent order (rx, ry, rz, x, y, z), index 2 is
        //    the near-vertical camera z for a near-nadir camera).
        if (item.telem.valid) {
            const Eigen::Matrix3d R_meas = measuredRotationEnuCamera(item.telem);
            gtsam::Vector6 sigmas;
            sigmas << config_.rollpitch_sigma_rad, config_.rollpitch_sigma_rad,
                      LOOSE_SIGMA, LOOSE_SIGMA, LOOSE_SIGMA, LOOSE_SIGMA;
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                X(k), gtsam::Pose3(gtsam::Rot3(R_meas), gtsam::Point3(0.0, 0.0, 0.0)),
                gtsam::noiseModel::Diagonal::Sigmas(sigmas));
        }

        // 5. AGL altitude factor.
        if (item.telem.valid && item.telem.altitude_m > 0.0) {
            graph.emplace_shared<AglFactor>(
                X(k), B(k), item.telem.altitude_m,
                gtsam::noiseModel::Isotropic::Sigma(1, config_.agl_sigma_m));
        }

        // Initial estimates: propagate the pose with the current scale. At a
        // boundary the fresh prior mean (1.0) is the better scale guess.
        const double scale_guess = reinit_boundary ? 1.0 : scale_est_;
        gtsam::Values values;
        const gtsam::Pose3 guess =
            gtsam::Pose3(T_enu_c_last_kf_.matrix())
                .compose(gtsam::Pose3(rel.rotation(), rel.translation() * scale_guess));
        values.insert(X(k), guess);
        values.insert(S(k), scale_guess);
        values.insert(B(k), bias_est_);

        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        stamps[X(k)] = t_sec;
        stamps[S(k)] = t_sec;
        stamps[B(k)] = t_sec;
        // Refresh the constant θ key so the fixed-lag window never
        // marginalizes it out.
        stamps[T(0)] = t_sec;

        smoother_->update(graph, values, stamps);

        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(X(k)).matrix();
        scale_est_                = smoother_->calculateEstimate<double>(S(k));
        bias_est_                 = smoother_->calculateEstimate<double>(B(k));
        theta_est_rad_            = smoother_->calculateEstimate<double>(T(0));

        T_wc_last_kf_        = T_wc;
        last_kf_frame_id_    = item.res.frame_id;
        next_key_            = k + 1;
        last_kf_telem_valid_ = item.telem.valid;
        if (item.telem.valid) {
            last_kf_psi_rad_ = psiRad(item.telem);
        }
        saw_non_tracking_since_kf_ = false;
        ++keyframe_count_;

        kf_registry_.push_back({k, item.res.frame_id});
        rebuildLagWindow();

        if (reinit_boundary) {
            spdlog::info("FusionModule: post-reinit boundary at X({}) (frame {}) — "
                         "scale restarted at 1.0, VO trans sigma ×{:.1f}, "
                         "delta-yaw skipped",
                         k, item.res.frame_id, config_.reinit_trans_inflation);
        }
        spdlog::debug("FusionModule: keyframe X({}) inserted (frame {}, s={:.4f}, "
                      "b={:.2f} m, {} KFs in graph history)",
                      k, item.res.frame_id, scale_est_, bias_est_, keyframe_count_);
    }

    //! Rebuild the lag-window snapshot after a successful smoother update.
    //! A key that the fixed-lag window has marginalized out no longer exists
    //! in calculateEstimate()'s Values (values.exists(X(k)) is false) — those
    //! registry entries are pruned for good (a marginalized key never comes
    //! back). Surviving keyframes are stored in registry order, which is
    //! insertion order = ascending k = ascending frame_id. Runs on the
    //! process() thread only; the snapshot swap shares mtx_latest_ with
    //! latest()/getLagWindow().
    void rebuildLagWindow() {
        const gtsam::Values values = smoother_->calculateEstimate();

        std::vector<KeyframeEntry>  live;
        std::vector<FusionLagPose>  window;
        live.reserve(kf_registry_.size());
        window.reserve(kf_registry_.size());
        for (const auto& entry : kf_registry_) {
            if (!values.exists(X(entry.k))) {
                continue;  // marginalized out of the fixed-lag window — prune
            }
            live.push_back(entry);
            FusionLagPose lp;
            lp.frame_id         = entry.frame_id;
            lp.T_enu_c.matrix() = values.at<gtsam::Pose3>(X(entry.k)).matrix();
            window.push_back(std::move(lp));
        }
        kf_registry_.swap(live);

        std::lock_guard<std::mutex> lock(mtx_latest_);
        lag_window_ = std::move(window);
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

    // Newest result snapshot + corrected lag-window snapshot (same mutex).
    FusionResult               latest_;
    std::vector<FusionLagPose> lag_window_;
    mutable std::mutex         mtx_latest_;

    //! One inserted keyframe: smoother key index k ↔ source frame id. The
    //! registry (process()-thread only) tracks which X(k) are still alive in
    //! the fixed-lag window; pruned in rebuildLagWindow().
    struct KeyframeEntry {
        unsigned int k        = 0;
        unsigned int frame_id = 0;
    };
    std::vector<KeyframeEntry> kf_registry_;

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

    // Latest auxiliary-state estimates (read back after every smoother update).
    double scale_est_     = 1.0;  //!< s(k_last)
    double bias_est_      = 0.0;  //!< b(k_last)
    double theta_est_rad_ = 0.0;  //!< θ = t(0); init from config in the ctor

    // Telemetry azimuth at the last keyframe (delta-yaw factor endpoints).
    double last_kf_psi_rad_     = 0.0;
    bool   last_kf_telem_valid_ = false;

    //! True when a non-TRACKING VO result (LOST or NOT_INITIALIZED) was seen
    //! since the previous keyframe → the next keyframe is a post-reinit
    //! boundary. Cleared on initialize() and after every keyframe.
    bool saw_non_tracking_since_kf_ = false;
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

std::vector<FusionLagPose> FusionModule::getLagWindow() const {
    return impl_->getLagWindow();
}

} // namespace uavloc::fusion
