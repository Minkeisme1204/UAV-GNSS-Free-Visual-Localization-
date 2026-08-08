#include "uavloc/fusion/fusion_module.h"

#include "factors.h"

#include "uavloc/util/scoped_timer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <spdlog/spdlog.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace uavloc::fusion {

namespace {

//! Soft queue-depth threshold: the queue never drops items (keyframes are
//! required by the graph), but growth beyond this depth is logged (once per
//! doubling — rate-limited).
constexpr std::size_t QUEUE_SOFT_WARN_DEPTH = 32;

//! Effectively-unconstrained sigma for the tangent components a partial prior
//! must not touch (kcb pattern: AHRS quat prior uses rot sigmas + 1e9 trans).
//! A convention of "infinity", not a mission tunable.
constexpr double LOOSE_SIGMA = 1e9;

// Gimbal-tilt convention, mirrored EXACTLY from new_vo's
// VOModule::seed_metric_scale(): tilt is measured from the horizontal plane,
// 90 deg = nadir, so the off-nadir angle is (90 - tilt); a tilt outside the
// plausible band falls back to a nadir assumption (off-nadir = 0). Frame
// conventions, not mission tunables — same status as LOOSE_SIGMA.
constexpr double GIMBAL_TILT_NADIR_DEG     = 90.0;
constexpr double GIMBAL_TILT_MIN_VALID_DEG = 45.0;
//! Below this cos(off-nadir) the depth↔AGL relation degenerates (grazing view).
constexpr double MIN_COS_OFF_NADIR = 1e-6;

//! Capacity of the absolute-fix intake queue. Unlike queue_ (keyframes, which
//! the graph cannot do without and which are therefore never dropped), a fix
//! that the pipeline could not consume in time is worthless: the queue drops
//! the OLDEST entry so a burst of fixes can neither stall the producer nor
//! grow without bound. Sized far above the expected in-flight count (at most
//! one fix per keyframe interval).
constexpr std::size_t FIX_QUEUE_CAPACITY = 64;

//! Index of the translation block inside the Pose3 tangent ordering
//! (rx, ry, rz, x, y, z) — used to slice marginalCovariance().
constexpr int POSE3_TANGENT_TRANS_INDEX = 3;

constexpr double MSEC_TO_SEC = 1e-3;
constexpr double DEG_TO_RAD  = M_PI / 180.0;
constexpr double RAD_TO_DEG  = 180.0 / M_PI;

// State keys (gtsam::Symbol): x(k) Pose3 T_enu_camera, s(k) VO scale
// multiplier, b(k) AGL-vs-ENU-Z bias.
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::S;
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

    //! Producer side of the absolute-fix intake (any thread). Only validation
    //! and queueing happen here — matching, gating and graph insertion run on
    //! the process() thread inside addKeyframe().
    void push_absolute_fix(const anchor::AbsoluteFix& fix) {
        if (!fix.valid) {
            spdlog::debug("FusionModule: absolute fix ignored — valid == false");
            return;
        }
        if (!fix.xy_enu.allFinite() || !isUsableCovariance(fix.cov)) {
            spdlog::warn("FusionModule: absolute fix rejected — non-finite "
                         "position or non-SPD covariance");
            return;
        }
        fix_injected_.fetch_add(1, std::memory_order_relaxed);

        if (!initialized_.load(std::memory_order_acquire)) {
            // No X(k) exists yet: the ENU frame the fix is expressed in is
            // only defined once the graph is anchored.
            fix_no_graph_.fetch_add(1, std::memory_order_relaxed);
            spdlog::debug("FusionModule: absolute fix dropped — graph not "
                          "initialized yet");
            return;
        }

        std::lock_guard<std::mutex> lock(mtx_fix_);
        if (fix_queue_.size() >= FIX_QUEUE_CAPACITY) {
            fix_queue_.pop_front();  // drop-oldest
            fix_queue_dropped_.fetch_add(1, std::memory_order_relaxed);
            spdlog::warn("FusionModule: absolute-fix queue full ({}) — "
                         "oldest fix dropped", FIX_QUEUE_CAPACITY);
        }
        fix_queue_.push_back(fix);
    }

    FusionFixStats fix_stats() const {
        FusionFixStats st;
        st.injected      = fix_injected_.load(std::memory_order_relaxed);
        st.applied       = fix_applied_.load(std::memory_order_relaxed);
        st.gated         = fix_gated_.load(std::memory_order_relaxed);
        st.low_confidence = fix_low_confidence_.load(std::memory_order_relaxed);
        st.age_expired   = fix_age_expired_.load(std::memory_order_relaxed);
        st.unmatched     = fix_unmatched_.load(std::memory_order_relaxed);
        st.marginalized  = fix_marginalized_.load(std::memory_order_relaxed);
        st.queue_dropped = fix_queue_dropped_.load(std::memory_order_relaxed);
        st.no_graph      = fix_no_graph_.load(std::memory_order_relaxed);
        st.distance_since_fix_m =
            fix_distance_since_m_.load(std::memory_order_relaxed);
        st.last_gated_residual_m =
            fix_last_gated_residual_m_.load(std::memory_order_relaxed);
        st.last_gated_gate_radius_m =
            fix_last_gated_radius_m_.load(std::memory_order_relaxed);
        return st;
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

        if (!initialized_.load(std::memory_order_acquire)) {
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

    //! Fill the auxiliary graph states (current estimates) + health +
    //! horizontal covariance. The covariance is the one computed at the last
    //! smoother update, i.e. the covariance of X(base_keyframe_id) — on a
    //! non-keyframe it is carried over unchanged (documented on
    //! FusionResult::xy_covariance).
    void fillAuxStates(FusionResult& out) const {
        out.scale            = scale_est_;
        out.agl_bias_m       = bias_est_;
        out.health           = health_;
        out.xy_covariance    = xy_cov_last_kf_;
        out.covariance_valid = xy_cov_valid_;
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

    //! cos(β) of the off-nadir angle from the gimbal tilt — same derivation as
    //! new_vo's seed_metric_scale(), so the map depth this module receives and
    //! the depth VO used to seed its metric scale are directly comparable.
    static double cosOffNadir(const sensor::TelemetryData& telem) {
        double off_nadir_rad = 0.0;
        const double tilt = telem.gimbal_tilt_deg;
        if (tilt > GIMBAL_TILT_MIN_VALID_DEG && tilt <= GIMBAL_TILT_NADIR_DEG) {
            off_nadir_rad = (GIMBAL_TILT_NADIR_DEG - tilt) * DEG_TO_RAD;
        }
        return std::cos(off_nadir_rad);
    }

    //! First keyframe with valid telemetry: anchor the graph.
    //! X(0) sits at ENU position (0, 0, agl_0) with the telemetry-measured
    //! rotation; s(0) and b(0) get their priors here.
    void initialize(const WorkItem& item, const Eigen::Isometry3d& T_wc) {
        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.lag_seconds);

        const Eigen::Matrix3d R_enu_c = measuredRotationEnuCamera(item.telem);
        const double          agl_0   = item.telem.altitude_m;
        const double          t0_sec  = item.res.timestamp_msec * MSEC_TO_SEC;
        const double anchor_yaw_sigma_rad = config_.anchor_yaw_sigma_deg * DEG_TO_RAD;

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
        // (index 2 — camera z is near-vertical for a near-nadir camera) is left
        // loose because absolute yaw is unknown at init (only heading deltas
        // constrain it, pre-VPR). Translation: weak X/Y (prevents indeterminacy
        // pre-VPR), AGL-grade Z.
        gtsam::Vector6 sigmas;
        sigmas << config_.rollpitch_sigma_rad, config_.rollpitch_sigma_rad,
                  anchor_yaw_sigma_rad, config_.anchor_xy_sigma_m,
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

        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        stamps[X(0)] = t0_sec;
        stamps[S(0)] = t0_sec;
        stamps[B(0)] = t0_sec;

        {
            util::ScopedTimer _t(util::ProfileStage::FUSION_GRAPH_UPDATE);
            smoother_->update(graph, values, stamps);
        }

        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(X(0)).matrix();
        scale_est_                = smoother_->calculateEstimate<double>(S(0));
        bias_est_                 = smoother_->calculateEstimate<double>(B(0));

        setDistanceSinceFix(0.0);  // the drift budget starts at the anchor
        T_wc_last_kf_        = T_wc;
        last_kf_frame_id_    = item.res.frame_id;
        next_key_            = 1;
        keyframe_count_      = 1;
        last_kf_psi_rad_     = psiRad(item.telem);
        last_kf_telem_valid_ = true;  // initialize() requires valid telemetry
        saw_non_tracking_since_kf_ = false;
        initialized_.store(true, std::memory_order_release);

        kf_registry_.clear();
        kf_registry_.push_back({0, item.res.frame_id, item.res.timestamp_msec});
        rebuildLagWindow();
        refreshCovarianceAndHealth(0);

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

        // 6. Map-depth scale measurement (Huber-robust) — the second factor
        //    touching s(k), which is what makes the map scale observable at
        //    all (the ScaledVOFactor alone is invariant under s → α·s). Added
        //    at re-init boundaries too: it is an independent ABSOLUTE
        //    measurement, unlike the scale random walk. Gated on a keyframe
        //    median over enough landmarks and a plausible flight AGL — near
        //    the ground d_vo/AGL explodes and would poison the scale state.
        if (config_.map_depth_enabled && item.res.median_map_depth > 0.0 &&
            item.res.median_depth_num_lms >= config_.map_depth_min_lms &&
            item.telem.valid &&
            item.telem.altitude_m >= config_.map_depth_min_agl_m) {
            const double cos_beta = cosOffNadir(item.telem);
            if (cos_beta > MIN_COS_OFF_NADIR) {
                const auto md_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k),
                    gtsam::noiseModel::Isotropic::Sigma(1, config_.map_depth_sigma));
                graph.emplace_shared<MapDepthFactor>(
                    S(k), item.res.median_map_depth, item.telem.altitude_m,
                    cos_beta, md_noise);
                spdlog::debug("FusionModule: map-depth factor at S({}) — "
                              "d_vo={:.3f} ({} lms), agl={:.2f} m, "
                              "cosβ={:.4f}, s_meas={:.4f}",
                              k, item.res.median_map_depth,
                              item.res.median_depth_num_lms,
                              item.telem.altitude_m, cos_beta,
                              item.telem.altitude_m /
                                  (item.res.median_map_depth * cos_beta));
            }
        }

        // 7. Absolute horizontal fixes (anchor::AbsoluteFix) queued since the
        //    previous keyframe. Consumed BEFORE the update so an accepted fix
        //    joins this same graph; it attaches to the keyframe state closest
        //    in time, which may be an EARLIER X(m) — the fixed-lag smoother
        //    then corrects the past as well, something a forward-only filter
        //    cannot do. No-op (graph untouched) when the queue is empty.
        consumeAbsoluteFixes(graph, t_sec);

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

        {
            util::ScopedTimer _t(util::ProfileStage::FUSION_GRAPH_UPDATE);
            smoother_->update(graph, values, stamps);
        }

        const Eigen::Vector3d p_prev_kf = T_enu_c_last_kf_.translation();
        T_enu_c_last_kf_.matrix() = smoother_->calculateEstimate<gtsam::Pose3>(X(k)).matrix();
        scale_est_                = smoother_->calculateEstimate<double>(S(k));
        bias_est_                 = smoother_->calculateEstimate<double>(B(k));

        // Grow the drift budget by the leg just flown. Measured on the graph's
        // own optimized positions — the same quantity the gate is testing — and
        // reset to 0 by applyAbsoluteFix() when a fix is accepted.
        setDistanceSinceFix(dist_since_fix_m_ +
                            (T_enu_c_last_kf_.translation() - p_prev_kf).norm());

        T_wc_last_kf_        = T_wc;
        last_kf_frame_id_    = item.res.frame_id;
        next_key_            = k + 1;
        last_kf_telem_valid_ = item.telem.valid;
        if (item.telem.valid) {
            last_kf_psi_rad_ = psiRad(item.telem);
        }
        saw_non_tracking_since_kf_ = false;
        ++keyframe_count_;

        kf_registry_.push_back({k, item.res.frame_id, item.res.timestamp_msec});
        rebuildLagWindow();
        refreshCovarianceAndHealth(k);

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

    // ── Absolute-fix intake (consumer side) ───────────────────────────────

    //! Symmetric-positive-definite check on a fix covariance: gtsam's
    //! Gaussian::Covariance() and the gate's solve both require it.
    static bool isUsableCovariance(const Eigen::Matrix2d& cov) {
        if (!cov.allFinite()) {
            return false;
        }
        const Eigen::LDLT<Eigen::Matrix2d> ldlt(cov);
        return ldlt.info() == Eigen::Success && ldlt.isPositive() &&
               cov(0, 0) > 0.0 && cov(1, 1) > 0.0;
    }

    //! Noise model for an accepted fix: the measurement covariance, optionally
    //! wrapped in a robust kernel. With fix_gate_enabled the Mahalanobis gate
    //! has already run at this point and the kernel only shapes the influence
    //! of measurements that were admitted; with the gate off (the default) the
    //! kernel is the ONLY thing standing between a bad residual and the graph —
    //! which is why the default kernel is Huber and not the redescending Tukey
    //! (see FusionConfig::fix_robust_kernel).
    gtsam::SharedNoiseModel makeFixNoiseModel(const Eigen::Matrix2d& cov) const {
        const auto base = gtsam::noiseModel::Gaussian::Covariance(cov);
        switch (config_.fix_robust_kernel) {
            case FixRobustKernel::HUBER:
                return gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k),
                    base);
            case FixRobustKernel::TUKEY:
                return gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Tukey::Create(config_.fix_tukey_c),
                    base);
            case FixRobustKernel::NONE:
                break;
        }
        return base;
    }

    //! Drain the fix queue and append the surviving AbsoluteXYFactors to
    //! `graph` — the SAME graph the caller is about to hand to
    //! smoother_->update(), so an accepted fix takes effect in the very
    //! update of the keyframe that consumed it. Process()-thread only.
    void consumeAbsoluteFixes(gtsam::NonlinearFactorGraph& graph, double now_sec) {
        std::vector<anchor::AbsoluteFix> fixes;
        {
            std::lock_guard<std::mutex> lock(mtx_fix_);
            if (fix_queue_.empty()) {
                return;  // fast path: nothing to do, graph untouched
            }
            fixes.assign(fix_queue_.begin(), fix_queue_.end());
            fix_queue_.clear();
        }
        for (const auto& fix : fixes) {
            applyAbsoluteFix(graph, fix, now_sec);
        }
    }

    //! Match one fix to a live keyframe state, gate it, and (if accepted) add
    //! the factor. `now_sec` is the timestamp of the keyframe driving this
    //! update.
    //!
    //! Candidates are the keyframes ALREADY in the smoother (kf_registry_,
    //! self-pruned on marginalization) — not the keyframe currently being
    //! inserted, whose marginal covariance does not exist yet and which could
    //! therefore only be gated on cov_fix alone.
    //!
    //! The timestamp of the matched key is deliberately NOT refreshed:
    //! re-stamping would extend that key's lifetime inside the fixed-lag
    //! window as a side effect of receiving a fix.
    void applyAbsoluteFix(gtsam::NonlinearFactorGraph& graph,
                          const anchor::AbsoluteFix& fix, double now_sec) {
        // Producer-side quality filter FIRST: it is the only rejection stage
        // that does not consult the estimate the fix is meant to correct, so it
        // cannot participate in a self-feeding drift loop (see
        // FusionConfig::fix_min_confidence / fix_gate_enabled).
        //
        // Stage 3 of the three-stage absolute-measurement trace
        // (anchor[REQ] → anchor[EMIT] → anchor[APPLY]), at `info`: EVERY fix
        // that enters here leaves exactly one anchor[APPLY] line carrying its
        // outcome, so no measurement can disappear without a trace. The
        // correlation key across the three stages is `ts` (= the timestamp of
        // the query the fix was made from); `kf_frame` is the frame of the
        // keyframe the fix was ATTACHED to, and `dt_match` how far that
        // keyframe sits from the fix — dt_match > 0 means the fix landed on a
        // keyframe OTHER than the frame that requested it.
        if (fix.confidence < config_.fix_min_confidence) {
            fix_low_confidence_.fetch_add(1, std::memory_order_relaxed);
            spdlog::info("anchor[APPLY] result=LOW_CONFIDENCE ts={:.1f} "
                         "kf_frame=n/a confidence={:.3f} < {:.3f}",
                         fix.timestamp_msec, fix.confidence,
                         config_.fix_min_confidence);
            return;
        }

        const double fix_sec = fix.timestamp_msec * MSEC_TO_SEC;
        const double age_budget_sec = fixAgeBudgetSec();
        if (now_sec - fix_sec > age_budget_sec) {
            fix_age_expired_.fetch_add(1, std::memory_order_relaxed);
            spdlog::info("anchor[APPLY] result=AGE_EXPIRED ts={:.1f} "
                         "kf_frame=n/a age={:.1f} ms > budget {:.1f} ms",
                         fix.timestamp_msec, (now_sec - fix_sec) / MSEC_TO_SEC,
                         age_budget_sec / MSEC_TO_SEC);
            return;
        }

        const KeyframeEntry* best = nullptr;
        double               best_dt = 0.0;
        for (const auto& entry : kf_registry_) {
            const double dt = std::abs(entry.timestamp_msec * MSEC_TO_SEC - fix_sec);
            if (best == nullptr || dt < best_dt) {
                best    = &entry;
                best_dt = dt;
            }
        }
        if (best == nullptr || best_dt > config_.fix_match_tolerance_sec) {
            fix_unmatched_.fetch_add(1, std::memory_order_relaxed);
            spdlog::info("anchor[APPLY] result=UNMATCHED ts={:.1f} kf_frame=n/a "
                         "nearest_kf={} dt_match={:.1f} ms > tol {:.1f} ms "
                         "(live keyframes={})",
                         fix.timestamp_msec,
                         best != nullptr ? static_cast<long long>(best->frame_id)
                                         : -1,
                         best_dt / MSEC_TO_SEC,
                         config_.fix_match_tolerance_sec / MSEC_TO_SEC,
                         kf_registry_.size());
            return;
        }
        const unsigned int k_match = best->k;
        const unsigned int kf_frame = best->frame_id;
        const gtsam::Key   key     = X(k_match);

        gtsam::Pose3 pose_pred;
        try {
            pose_pred = smoother_->calculateEstimate<gtsam::Pose3>(key);
        } catch (const std::exception& e) {
            fix_marginalized_.fetch_add(1, std::memory_order_relaxed);
            spdlog::info("anchor[APPLY] result=MARGINALIZED ts={:.1f} "
                         "kf_frame={} X({}) dt_match={:.1f} ms — no longer in "
                         "the smoother ({})",
                         fix.timestamp_msec, kf_frame, k_match,
                         best_dt / MSEC_TO_SEC, e.what());
            return;
        }

        const Eigen::Vector2d r(pose_pred.translation().x() - fix.xy_enu.x(),
                                pose_pred.translation().y() - fix.xy_enu.y());

        // ── Mahalanobis gate (OPT-IN, FusionConfig::fix_gate_enabled) ────────
        //
        // Gate covariance S = cov_fix + cov_pred + (rho·s)²·I.
        //
        // cov_pred grows with the VO drift, which is what keeps a CORRECT fix
        // admissible after the estimate has wandered far (a cov_fix-only gate
        // would reject exactly the measurement the estimator needs most — see
        // FusionConfig). The third term repairs what cov_pred gets WRONG: the
        // graph models VO error as independent noise (√n) while the real drift
        // is systematic (n), so cov_pred alone was measured ~118× too tight on
        // YenBai (.docs/reports/m1_fake_anchor.md §6.2). rho is a MEASURED
        // per-dataset drift rate supplied by the config (0 ⇒ term disabled and
        // the gate is byte-identical to M1), s is the distance flown since the
        // last APPLIED fix.
        //
        // The whole test is self-referential — it asks the estimate whether the
        // measurement that should correct it is plausible — and measurement
        // showed that this closes a positive-feedback loop (FusionConfig). It
        // is kept, off by default, so the M1 numbers stay reproducible.
        double d2            = 0.0;
        double drift_sigma_m = 0.0;
        if (config_.fix_gate_enabled) {
            Eigen::Matrix2d S = fix.cov;
            {
                Eigen::Matrix2d cov_pred;
                std::string     err;
                if (marginalXyCovarianceEnu(key, pose_pred.rotation().matrix(),
                                            cov_pred, err)) {
                    S += cov_pred;
                } else {
                    warnCovarianceFallbackOnce(k_match, err.c_str());
                }
            }
            if (config_.fix_drift_rate_m_per_m > 0.0) {
                drift_sigma_m = config_.fix_drift_rate_m_per_m * dist_since_fix_m_;
                S += (drift_sigma_m * drift_sigma_m) * Eigen::Matrix2d::Identity();
            }

            d2 = r.dot(S.ldlt().solve(r));
            if (!(d2 <= config_.fix_gate_chi2)) {
                // Gate radius ALONG this residual direction: the residual
                // rescaled to where it would have sat exactly on the
                // chi-square threshold.
                const double radius_m =
                    (d2 > 0.0) ? r.norm() * std::sqrt(config_.fix_gate_chi2 / d2) : 0.0;
                fix_gated_.fetch_add(1, std::memory_order_relaxed);
                fix_last_gated_residual_m_.store(r.norm(), std::memory_order_relaxed);
                fix_last_gated_radius_m_.store(radius_m, std::memory_order_relaxed);
                spdlog::info("anchor[APPLY] result=GATED ts={:.1f} kf_frame={} "
                             "X({}) dt_match={:.1f} ms age={:.1f}/{:.1f} ms "
                             "residual={:.1f} m d²={:.2f} > {:.2f} "
                             "(gate radius {:.1f} m, drift sigma {:.1f} m over "
                             "s={:.1f} m)",
                             fix.timestamp_msec, kf_frame, k_match,
                             best_dt / MSEC_TO_SEC,
                             (now_sec - fix_sec) / MSEC_TO_SEC,
                             age_budget_sec / MSEC_TO_SEC, r.norm(), d2,
                             config_.fix_gate_chi2, radius_m, drift_sigma_m,
                             dist_since_fix_m_);
                return;
            }
        }

        graph.emplace_shared<AbsoluteXYFactor>(key, fix.xy_enu,
                                               makeFixNoiseModel(fix.cov));
        fix_applied_.fetch_add(1, std::memory_order_relaxed);
        // d² and the drift sigma are only meaningful when the gate ran; say so
        // instead of printing a 0 that reads like a measured value.
        //
        // `age` is the budget figure asked of the lag window: how far back the
        // smoother had to reach for this fix, against what the window allows.
        spdlog::info("anchor[APPLY] result=APPLIED ts={:.1f} kf_frame={} X({}) "
                     "dt_match={:.1f} ms age={:.1f}/{:.1f} ms residual={:.1f} m "
                     "gate={} (d²={:.2f}, drift sigma {:.1f} m over s={:.1f} m)",
                     fix.timestamp_msec, kf_frame, k_match,
                     best_dt / MSEC_TO_SEC, (now_sec - fix_sec) / MSEC_TO_SEC,
                     age_budget_sec / MSEC_TO_SEC, r.norm(),
                     config_.fix_gate_enabled ? "on" : "off", d2, drift_sigma_m,
                     dist_since_fix_m_);
        // The drift budget restarts at an ACCEPTED fix only: a gated, expired
        // or unmatched fix leaves the estimate exactly as uncertain as it was.
        setDistanceSinceFix(0.0);
    }

    //! Age budget of a fix [s]: the whole lag window minus a safety margin —
    //! the window IS how far back the smoother can still correct, and the
    //! margin keeps a fix from landing on a state that marginalizes during this
    //! very update (kcb_slam: last_stamp − lag_window + 0.5). Clamped at 0 so a
    //! misconfigured margin can only reject, never admit everything.
    double fixAgeBudgetSec() const {
        return std::max(0.0, config_.lag_seconds - config_.fix_age_margin_sec);
    }

    //! Single writer of the flown-distance state (process() thread); keeps the
    //! atomic mirror that fix_stats() publishes in step with it.
    void setDistanceSinceFix(double meters) {
        dist_since_fix_m_ = meters;
        fix_distance_since_m_.store(meters, std::memory_order_relaxed);
    }

    //! Rate-limited (once per module lifetime) warning for the degraded gate:
    //! when the marginal covariance is unavailable the gate falls back to the
    //! fix covariance alone, which is conservative (it rejects more).
    void warnCovarianceFallbackOnce(unsigned int k, const char* what) {
        if (warned_cov_fallback_) {
            spdlog::debug("FusionModule: marginalCovariance(X({})) failed again "
                          "({}) — gate still on cov_fix only", k, what);
            return;
        }
        warned_cov_fallback_ = true;
        spdlog::warn("FusionModule: marginalCovariance(X({})) failed ({}) — "
                     "absolute-fix gate falls back to the fix covariance alone "
                     "(this warning is not repeated)", k, what);
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

    // ── Marginal covariance + health (S7) ─────────────────────────────────

    //! ENU horizontal (East/North) marginal covariance of the pose state
    //! `key` [m²]. The Pose3 translation block gtsam returns is expressed in
    //! the LOCAL (camera) frame — Pose3::retract translates by R·v — so it is
    //! rotated into ENU with the pose's own rotation before the E/N corner is
    //! taken. Returns false and fills `err` when the smoother cannot produce a
    //! usable covariance; `out` is then left untouched.
    //!
    //! Single source of this computation: both the absolute-fix gate and the
    //! per-keyframe accuracy figure go through it.
    bool marginalXyCovarianceEnu(gtsam::Key key, const Eigen::Matrix3d& R_enu_c,
                                 Eigen::Matrix2d& out, std::string& err) const {
        try {
            const gtsam::Matrix   cov = smoother_->marginalCovariance(key);
            const Eigen::Matrix3d cov_t_local =
                cov.block<3, 3>(POSE3_TANGENT_TRANS_INDEX, POSE3_TANGENT_TRANS_INDEX);
            const Eigen::Matrix3d cov_t_enu =
                R_enu_c * cov_t_local * R_enu_c.transpose();
            const Eigen::Matrix2d xy = cov_t_enu.topLeftCorner<2, 2>();
            if (!xy.allFinite()) {
                err = "non-finite marginal covariance";
                return false;
            }
            out = xy;
            return true;
        } catch (const std::exception& e) {
            // Covers gtsam::IndeterminantLinearSystemException (a
            // ThreadsafeException, hence a std::exception) and everything else
            // the elimination can throw.
            err = e.what();
            return false;
        }
    }

    //! Called right after every smoother update: refresh the horizontal
    //! covariance of the freshly optimized keyframe state X(k) and step the
    //! health state machine with it.
    void refreshCovarianceAndHealth(unsigned int k) {
        Eigen::Matrix2d cov;
        std::string     err;
        if (marginalXyCovarianceEnu(X(k), T_enu_c_last_kf_.linear(), cov, err)) {
            xy_cov_last_kf_ = cov;
            xy_cov_valid_   = true;
        } else {
            // The PREVIOUS keyframe's covariance is deliberately dropped:
            // publishing a stale (and smaller) uncertainty as if it were
            // current is worse than publishing none at all.
            xy_cov_last_kf_.setZero();
            xy_cov_valid_ = false;
            warnStateCovarianceOnce(k, err.c_str());
        }

        const double sigma = horizontal_accuracy_m(xy_cov_last_kf_, xy_cov_valid_);
        const FusionHealth prev = health_;
        health_ = next_fusion_health(health_, sigma, xy_cov_valid_,
                                     config_.health_converged_sigma_m,
                                     config_.health_drifting_sigma_m);
        if (health_ != prev) {
            spdlog::info("FusionModule: health {} → {} at X({}) "
                         "(sigma_xy={:.2f} m, thresholds {:.2f}/{:.2f} m)",
                         healthName(prev), healthName(health_), k, sigma,
                         config_.health_converged_sigma_m,
                         config_.health_drifting_sigma_m);
        }
        spdlog::debug("FusionModule: X({}) sigma_xy={:.3f} m (health {})",
                      k, sigma, healthName(health_));
    }

    static const char* healthName(FusionHealth h) {
        switch (h) {
            case FusionHealth::INITIALIZING: return "INITIALIZING";
            case FusionHealth::CONVERGED:    return "CONVERGED";
            case FusionHealth::DRIFTING:     return "DRIFTING";
        }
        return "?";
    }

    //! Rate-limited (once per module lifetime) warning for a missing STATE
    //! covariance: FusionResult::covariance_valid then goes false and
    //! accuracy_m becomes NaN downstream — never 0.
    void warnStateCovarianceOnce(unsigned int k, const char* what) {
        if (warned_state_cov_) {
            spdlog::debug("FusionModule: marginalCovariance(X({})) failed again "
                          "({}) — accuracy reported as unavailable", k, what);
            return;
        }
        warned_state_cov_ = true;
        spdlog::warn("FusionModule: marginalCovariance(X({})) failed ({}) — "
                     "FusionResult carries no covariance and accuracy_m is NaN "
                     "(this warning is not repeated)", k, what);
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
        //! Source-frame timestamp — the key an absolute fix is matched on.
        double       timestamp_msec = 0.0;
    };
    std::vector<KeyframeEntry> kf_registry_;

    // Absolute-fix intake. Separate from queue_ and drop-oldest: fixes are
    // optional evidence, keyframes are not.
    std::deque<anchor::AbsoluteFix> fix_queue_;
    mutable std::mutex              mtx_fix_;
    std::atomic<unsigned long long> fix_injected_{0};
    std::atomic<unsigned long long> fix_applied_{0};
    std::atomic<unsigned long long> fix_gated_{0};
    std::atomic<unsigned long long> fix_low_confidence_{0};
    // One counter per drop reason — see FusionFixStats: M1 merged three of
    // these and the 84 % rejection rate went unnoticed for a whole milestone.
    std::atomic<unsigned long long> fix_age_expired_{0};
    std::atomic<unsigned long long> fix_unmatched_{0};
    std::atomic<unsigned long long> fix_marginalized_{0};
    std::atomic<unsigned long long> fix_queue_dropped_{0};
    std::atomic<unsigned long long> fix_no_graph_{0};
    //! Publication mirrors of the gate observables (written by process(), read
    //! by fix_stats() from any thread).
    std::atomic<double> fix_distance_since_m_{0.0};
    std::atomic<double> fix_last_gated_residual_m_{0.0};
    std::atomic<double> fix_last_gated_radius_m_{0.0};
    //! Rate-limit flag for the degraded-gate warning (process() thread only).
    bool warned_cov_fallback_ = false;
    //! Rate-limit flag for the missing-state-covariance warning (same thread).
    bool warned_state_cov_ = false;

    // Graph state — touched only by process() (single consumer: the fusion
    // thread in async mode, the caller's thread in sync mode).
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    //! Atomic: written once by process(), read by push_absolute_fix() from
    //! the producer thread (a fix that predates the anchor has no frame).
    std::atomic<bool> initialized_{false};
    unsigned int      next_key_       = 0;  //!< index k of the next X(k)
    unsigned int      keyframe_count_ = 0;
    unsigned int      last_kf_frame_id_ = 0;
    Eigen::Isometry3d T_wc_last_kf_    = Eigen::Isometry3d::Identity();  //!< VO pose at last KF
    Eigen::Isometry3d T_enu_c_last_kf_ = Eigen::Isometry3d::Identity();  //!< optimized X(k_last)
    Eigen::Isometry3d T_enu_from_w_    = Eigen::Isometry3d::Identity();  //!< VO world → ENU

    // Latest auxiliary-state estimates (read back after every smoother update).
    double scale_est_     = 1.0;  //!< s(k_last)
    double bias_est_      = 0.0;  //!< b(k_last)

    //! Distance flown [m] since the last APPLIED absolute fix (since graph
    //! initialization while no fix has been applied yet) — the `s` of the
    //! drift term of the fix gate. Accumulated over the OPTIMIZED keyframe
    //! positions (the graph's own trajectory), so it needs no extra input and
    //! stays consistent with the state the gate is testing. process() thread
    //! only; mirrored into fix_distance_since_m_ for publication.
    double dist_since_fix_m_ = 0.0;

    //! ENU horizontal covariance of X(k_last) and its validity, refreshed at
    //! every smoother update by refreshCovarianceAndHealth(). Zero + false
    //! before the graph is anchored (and after a failed marginalization).
    Eigen::Matrix2d xy_cov_last_kf_ = Eigen::Matrix2d::Zero();
    bool            xy_cov_valid_   = false;
    //! Health state — advanced only by refreshCovarianceAndHealth().
    FusionHealth    health_         = FusionHealth::INITIALIZING;

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

void FusionModule::push_absolute_fix(const anchor::AbsoluteFix& fix) {
    impl_->push_absolute_fix(fix);
}

FusionFixStats FusionModule::fix_stats() const {
    return impl_->fix_stats();
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
