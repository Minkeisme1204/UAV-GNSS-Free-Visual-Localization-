// debug_viewer_bridge.cpp — SystemManager -> DebugViewer, and the two threads
// that own a live run. See debug_viewer_bridge.h for the design notes and for
// the bit-identical constraint this file is under.

#include "debug_viewer_bridge.h"

#include "uavloc/util/scoped_timer.h"
#include "uavloc/util/sys_monitor.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace uavloc::debug_viewer {
namespace {

// Poll period [ms] of the supervisor thread that watches for end-of-data /
// the frame cap. Not a pipeline parameter — only how fast teardown reacts.
constexpr int SUPERVISOR_POLL_MS = 20;

// ── HUD heading formula (mirrors the old test_vo_pipeline) ───────────────────
// Degrees per full turn — used to wrap heading angles into [0, 360).
constexpr double DEGREES_PER_TURN = 360.0;
// Radians -> degrees for the course-over-ground heading.
constexpr double DEG_PER_RAD = 180.0 / M_PI;

// Wrap an angle in degrees into [0, 360).
double wrap360(double deg) {
    deg = std::fmod(deg, DEGREES_PER_TURN);
    if (deg < 0.0) deg += DEGREES_PER_TURN;
    return deg;
}

// Tracking-overlay presentation constants (debug-viewer sub-window only — NOT
// pipeline thresholds): circle radius for a tracked landmark keypoint, and the
// annotation text placement/scale.
constexpr int    TRACK_KP_RADIUS_PX   = 4;   // tracked-landmark circle radius (px)
constexpr int    TRACK_KP_THICKNESS   = 2;   // tracked-landmark circle stroke (px)
constexpr int    TRACK_TEXT_X_PX      = 10;
constexpr int    TRACK_TEXT_Y_PX      = 30;
constexpr double TRACK_TEXT_SCALE     = 0.8;
constexpr int    TRACK_TEXT_THICKNESS = 2;

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
SystemBridge::SystemBridge(DebugViewer& viewer, const DebugViewer::Config& cfg)
    : viewer_(viewer),
      cfg_(cfg),
      video_stride_(std::max(1, cfg.video_stride)),
      hud_min_baseline_m_(cfg.hud_min_baseline_m),
      perf_sample_ms_(cfg.perf_sample_ms),
      profile_t0_(std::chrono::steady_clock::now()),
      aligner_(TrajectoryAligner::Config{cfg.alignment_window_frames,
                                         cfg.alignment_min_spread_m}) {}

SystemBridge::~SystemBridge() {
    // Defensive: a caller that never reached end() (an exception out of the
    // render loop) must not leave two detached threads touching a dead object.
    stop_requested_.store(true);
    if (supervisor_.joinable())   supervisor_.join();
    if (perf_sampler_.joinable()) perf_sampler_.join();
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------
bool SystemBridge::attach(core::SystemManager& sys) {
    if (sys_ != nullptr) {
        spdlog::error("DebugViewer::attach: a system is already attached — "
                      "ignoring the second one");
        return false;
    }
    sys_ = &sys;

    // ── channel 1: image + telemetry of the frame just processed ─────────────
    sys.callbacks().on_frame_processed.add(
        [this](double /*t_msec*/, const core::FrameProcessed& fp) {
            onFrameProcessed(fp);
        });

    // ── channel 2: the VO payload — the bulk of the visualization ────────────
    sys.callbacks().on_vo_data.add(
        [this](double /*t_msec*/, const vo::VOData& data) { onVoData(data); });

    // ── channel 3: the fused pose ────────────────────────────────────────────
    sys.callbacks().on_fusion_result.add(
        [this](double /*t_msec*/, const fusion::FusionResult& fres) {
            onFusionResult(fres);
        });

    // ── channel 4: the smoothed lag window ───────────────────────────────────
    sys.callbacks().on_lag_window.add(
        [this](const std::vector<fusion::FusionLagPose>& window) {
            onLagWindow(window);
        });

    // ── channel 5: the periodic monitoring figures ───────────────────────────
    // Unlike the four above this one fires on the MONITOR thread, on a fixed
    // cadence (SystemConfig::stats_period_ms) that is INDEPENDENT of processing
    // load — which is exactly what lets the HUD show the rate falling to 0 when
    // the pipeline stalls instead of freezing at the last processed frame.
    sys.callbacks().on_stats.add(
        [this](const core::SystemStats& st) { onStats(st); });

    // ── The Start/Stop bridge ────────────────────────────────────────────────
    // The handler runs on the viewer's render thread — never on the source's
    // reading thread, which is the one caller pauseSource() would have to
    // refuse — and returns the VERDICT, so a refused request leaves the button
    // label where it was instead of lying.
    viewer_.setRunControl([this](bool want_run) {
        if (!want_run) {
            return sys_->pauseSource();
        }
        if (system_started_.load()) {
            return sys_->resumeSource();
        }
        if (!sys_->start()) {
            return false;
        }
        system_started_.store(true);
        return true;
    });
    return true;
}

void SystemBridge::setFusedPoseSink(DebugViewer::FusedPoseSink sink) {
    std::lock_guard<std::mutex> lk(fused_mutex_);
    fused_sink_ = std::move(sink);
}

void SystemBridge::enableAnchorFixTracking(const std::string& warning) {
    std::lock_guard<std::mutex> lk(anchor_mutex_);
    anchor_tracking_ = true;
    anchor_warning_  = warning;
}

AnchorFixCounters SystemBridge::anchorFixCounters() const {
    std::lock_guard<std::mutex> lk(anchor_mutex_);
    AnchorFixCounters c;
    c.generated        = anchor_generated_;
    c.accepted         = anchor_accepted_;
    c.rejected         = anchor_rejected_;
    c.pending          = anchor_pending_.size();
    c.reinit_generated = anchor_reinit_generated_;
    c.reinit_accepted  = anchor_reinit_accepted_;
    return c;
}

bool SystemBridge::beyondCapIndex(std::size_t index) const {
    return cfg_.max_frames > 0 && index > cfg_.max_frames;
}

bool SystemBridge::beyondCapFrame(unsigned long long frame_id) const {
    return cap_reached_.load() && frame_id > cap_last_frame_id_.load();
}

// ---------------------------------------------------------------------------
// Helpers (moved verbatim from tests/test_vo_viewer.cpp)
// ---------------------------------------------------------------------------

// Build the "Profiling" table snapshot from the merged profiler stats
// (mean = total_ms / count) and hand it to the viewer (latest-wins).
void SystemBridge::pushProfileSnapshot(int frame_id) {
    namespace ut = uavloc::util;
    const auto   stats = ut::Profiler::snapshot_total();
    const double frame_total_ms =
        stats[static_cast<std::size_t>(ut::ProfileStage::PROCESS_FRAME_TOTAL)].total_ms;
    ProfileSnapshot snap;
    snap.t_sec = std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - profile_t0_).count();
    snap.frame_id = frame_id;
    for (std::size_t i = 0; i < stats.size(); ++i) {
        const ut::StageStat& s = stats[i];
        if (s.count == 0) {
            continue;  // stage never ran — keeps the table compact
        }
        ProfileRow row;
        row.name    = ut::to_string(static_cast<ut::ProfileStage>(i));
        row.mean_ms = static_cast<float>(s.total_ms / static_cast<double>(s.count));
        row.last_ms = static_cast<float>(s.last_ms);
        row.max_ms  = static_cast<float>(s.max_ms);
        row.count   = s.count;
        row.percent = (frame_total_ms > 0.0)
            ? static_cast<float>(100.0 * s.total_ms / frame_total_ms) : 0.0f;
        snap.rows.push_back(std::move(row));
    }
    viewer_.pushProfile(snap);
}

// Transform a VO-world position through the frozen alignment and push it
// as a discrete LOST-state marker (red, "Lost state poses" checkbox).
void SystemBridge::pushLost(int fid, const Eigen::Vector3d& p_world) {
    const Eigen::Vector3d est = aligner_.applyPoint(p_world);
    InferredPose lp;
    lp.frame_id = fid;
    lp.x = est.x(); lp.y = est.y(); lp.z = est.z();
    viewer_.pushLostPose(lp);
}

void SystemBridge::pushEstimate(int fid, const Eigen::Matrix4d& T_wc) {
    const Eigen::Vector3d p_world = T_wc.block<3, 1>(0, 3);
    const Eigen::Vector3d est     = aligner_.applyPoint(p_world);
    InferredPose ep;
    ep.frame_id = fid;
    ep.x = est.x(); ep.y = est.y(); ep.z = est.z();
    // Orientation for the viewer's "EstOdom" pose axes: rotate the VO camera
    // rotation into ENU through the frozen alignment, then extract ZYX (yaw,
    // pitch, roll) — the same convention the viewer's gizmo helper composes
    // back as Rz(yaw) * Ry(pitch) * Rx(roll).
    const Eigen::Matrix3d R_enu = aligner_.applyRotation(T_wc.block<3, 3>(0, 0));
    const Eigen::Vector3d ypr   = R_enu.eulerAngles(2, 1, 0);
    ep.yaw_deg   = ypr.x() * DEG_PER_RAD;
    ep.pitch_deg = ypr.y() * DEG_PER_RAD;
    ep.roll_deg  = ypr.z() * DEG_PER_RAD;
    viewer_.pushPose(ep);
}

// Feed the viewer's "Error (m)" window: |predicted - groundtruth| for the SAME
// frame, in metres. `pred` is the position already handed to pushFusedPose
// (i.e. in the groundtruth ENU frame through the g0 + (f - f0) anchor
// translation), so the two are directly comparable.
//
// MUST be called with fused_mutex_ held — it reads the gt_exact_* slot the
// pipeline thread writes.
//
// Frames whose groundtruth id does not match are SKIPPED rather than paired
// approximately: that includes every pose flushed out of fused_pending_
// (emitted before the first GT existed). A skipped frame is a missing bar in
// the histogram, which is honest; a mis-paired one would be a wrong bar.
//
// ⚠ This error is measured against TELEMETRY groundtruth, and it may still
// contain an unresolved mount-azimuth offset θ (F1: θ is prior-only), so it is
// an upper bound, not a 4-DoF-aligned ATE. When the absolute fixes are
// manufactured from the same telemetry, the number is CONTAMINATED — it may
// not be quoted as system accuracy.
void SystemBridge::pushErrorSample(uint64_t fid, const Eigen::Vector3d& pred) {
    if (!gt_exact_valid_ || gt_exact_frame_id_ != fid) {
        ++error_samples_skipped_;
        return;
    }
    ++error_samples_pushed_;
    const Eigen::Vector3d d = pred - gt_exact_enu_;
    ErrorSample s;
    s.frame_id = static_cast<int>(fid);
    s.err_2d_m = static_cast<float>(d.head<2>().norm());
    s.err_3d_m = static_cast<float>(d.norm());
    viewer_.pushErrorSample(s);
}

// ---------------------------------------------------------------------------
// Channel handlers
// ---------------------------------------------------------------------------
void SystemBridge::onFrameProcessed(const core::FrameProcessed& fp) {
    const std::size_t index = ++frames_processed_;
    if (cfg_.max_frames > 0 && index == cfg_.max_frames) {
        cap_last_frame_id_.store(fp.frame_id);
        cap_reached_.store(true);
    }
    if (beyondCapIndex(index)) {
        return;
    }
    frame_image_         = fp.image;  // cv::Mat is refcounted
    frame_telemetry_     = fp.telemetry;
    frame_has_telemetry_ = fp.has_telemetry;

    // Stream every `video_stride` frame into the viewer's video panel.
    if (index % static_cast<std::size_t>(video_stride_) == 0) {
        viewer_.pushFrame(fp.image);
    }
}

void SystemBridge::onVoData(const vo::VOData& data) {
    const vo::VOResult& r = data.result;
    if (beyondCapIndex(frames_processed_.load())) {
        return;
    }

    // Publish the PREVIOUS frame's groundtruth (see the staging note in
    // debug_viewer_bridge.h / below).
    if (staged_gt_valid_) {
        std::lock_guard<std::mutex> lk(fused_mutex_);
        latest_gt_enu_   = staged_gt_enu_;
        latest_gt_valid_ = true;
        staged_gt_valid_ = false;
    }

    if (data.map_updated && !data.map_points.empty()) {
        latest_map_points_ = data.map_points;
        map_dirty_         = true;
    }

    // Refresh the "Profiling" table (opt-in; no-op cost when disabled).
    if (util::Profiler::enabled()) {
        pushProfileSnapshot(static_cast<int>(r.frame_id));
    }

    // ── LOST markers: mark the last valid position — the point where tracking
    //    broke — so the marker sits on the estimate polyline. Push through the
    //    alignment once frozen, otherwise buffer. ─────────────────────────────
    if (r.state == vo::VOTrackingState::LOST && has_last_pos_) {
        if (aligner_.isFrozen()) {
            pushLost(static_cast<int>(r.frame_id), last_pos_);
        } else {
            lost_buffer_.emplace_back(static_cast<int>(r.frame_id), last_pos_);
        }
    }

    // ── HUD bookkeeping + per-frame metrics (has_pose frames) ───────────────
    if (r.has_pose) {
        const Eigen::Vector3d p = r.T_wc.block<3, 1>(0, 3);
        // Path length: sum of consecutive GLOBAL position deltas. The increment
        // is identity on the first step of a (re-)initialised map; at those
        // boundaries the position can jump, so skip them.
        const bool reinit_boundary =
            r.T_prev_curr.isApprox(Eigen::Matrix4d::Identity(), 1e-9);
        const Eigen::Vector3d dp = has_last_pos_
            ? Eigen::Vector3d(p - last_pos_) : Eigen::Vector3d::Zero();
        if (has_last_pos_ && !reinit_boundary) {
            trajectory_length_m_ += dp.norm();
            // VO course-over-ground heading (ENU, 0 = North CW): rotate the
            // per-frame displacement into ENU through the frozen alignment.
            // Held when the baseline is too small to trust the direction.
            if (aligner_.isFrozen() && dp.norm() >= hud_min_baseline_m_) {
                // Direction, not a point — rotation only, no translation.
                const Eigen::Vector3d dp_enu = aligner_.T().block<3, 3>(0, 0) * dp;
                last_cog_heading_deg_ = wrap360(
                    std::atan2(dp_enu.x(), dp_enu.y()) * DEG_PER_RAD);
            }
        }
        last_pos_     = p;
        has_last_pos_ = true;

        // Per-frame metrics: inliers/landmarks line charts + HUD scalars.
        const float hud_heading_tel =
            frame_has_telemetry_
                ? static_cast<float>(wrap360(frame_telemetry_.heading_deg +
                                             frame_telemetry_.gimbal_pan_deg))
                : 0.0f;
        FrameMetrics fm;
        fm.frame_id        = static_cast<int>(r.frame_id);
        fm.inliers         = static_cast<float>(r.num_inliers);
        fm.landmarks       = static_cast<float>(r.num_landmarks);
        fm.heading_deg     = static_cast<float>(last_cog_heading_deg_);
        fm.heading_tel_deg = hud_heading_tel;
        fm.distance_m      = static_cast<float>(trajectory_length_m_);
        viewer_.pushMetrics(fm);

        // Tracking sub-window: overlay the tracked landmark keypoints on the
        // frame and annotate the counts (same stride as the plain video panel).
        if (frames_processed_.load() % static_cast<std::size_t>(video_stride_) == 0 &&
            !frame_image_.empty()) {
            cv::Mat overlay = frame_image_.clone();
            for (const Eigen::Vector2d& obs : r.tracked_observations) {
                cv::circle(overlay,
                           cv::Point(static_cast<int>(std::lround(obs.x())),
                                     static_cast<int>(std::lround(obs.y()))),
                           TRACK_KP_RADIUS_PX,
                           cv::Scalar(0, 255, 0),  // green (BGR)
                           TRACK_KP_THICKNESS, cv::LINE_AA);
            }
            const std::string label =
                "Landmarks: " + std::to_string(r.num_landmarks) +
                "  tracked: " + std::to_string(r.tracked_observations.size());
            cv::putText(overlay, label,
                        cv::Point(TRACK_TEXT_X_PX, TRACK_TEXT_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, TRACK_TEXT_SCALE,
                        cv::Scalar(0, 255, 255), TRACK_TEXT_THICKNESS,
                        cv::LINE_AA);
            viewer_.pushTrackingFrame(overlay);
        }
    }

    // ── Overlay: estimate vs groundtruth (TRACKING frames only) ─────────────
    if (r.has_pose && r.state == vo::VOTrackingState::TRACKING &&
        frame_has_telemetry_ && frame_telemetry_.valid &&
        frame_telemetry_.altitude_m > 0.0) {
        const sensor::TelemetryData& t = frame_telemetry_;
        // First call wins inside the aligner — the origin is the first TRACKING
        // frame's fix.
        aligner_.setEnuOrigin(t.latitude_deg, t.longitude_deg, t.altitude_m);
        const Eigen::Vector3d gt_enu =
            aligner_.gpsToEnu(t.latitude_deg, t.longitude_deg, t.altitude_m);
        const Eigen::Vector3d p_cam = r.T_wc.block<3, 1>(0, 3);

        // Groundtruth is independent of alignment — push it every frame.
        InferredPose gp;
        gp.frame_id = static_cast<int>(r.frame_id);
        gp.x = gt_enu.x(); gp.y = gt_enu.y(); gp.z = gt_enu.z();
        viewer_.pushGroundtruthPose(gp);

        // ── ONE-FRAME-DEFERRED groundtruth publication ──────────────────────
        // The fusion callback of frame k must see the GT of frame k-1: that is
        // what picks g0, the anchor of the whole fused display line.
        // on_vo_data is emitted BEFORE on_fusion_result, so the GT is staged
        // here and published at the START of the next frame instead.
        // Publishing eagerly would shift the entire fused line by one frame of
        // motion.
        staged_gt_enu_   = gt_enu;
        staged_gt_valid_ = true;

        // Publish THIS frame's GT for the error window. Separate from the
        // staged/anchor path on purpose: the anchor wants the previous frame,
        // the error wants this one.
        {
            std::lock_guard<std::mutex> lk(fused_mutex_);
            gt_exact_enu_      = gt_enu;
            gt_exact_frame_id_ = r.frame_id;
            gt_exact_valid_    = true;
        }

        if (!aligner_.isFrozen()) {
            // Accumulate correspondences; buffer estimates until the frozen
            // transform is known, then flush them aligned.
            aligner_.addCorrespondence(p_cam, gt_enu);
            est_buffer_.emplace_back(static_cast<int>(r.frame_id), r.T_wc);

            if (aligner_.readyToFreeze()) {
                // The latest keyframe map-point cloud (VO world, metres) — the
                // landmark carpet defines the ground plane the aligner levels
                // roll/pitch against.
                aligner_.setPlanePoints(std::vector<Eigen::Vector3d>(
                    latest_map_points_.begin(), latest_map_points_.end()));
                const std::size_t n_corr = aligner_.correspondenceCount();
                aligner_.tryFreeze();
                spdlog::info("alignment frozen at frame {} over {} TRACKING "
                             "correspondences", r.frame_id, n_corr);
                for (const auto& fp : est_buffer_) {
                    pushEstimate(fp.first, fp.second);
                }
                est_buffer_.clear();
                // Flush LOST markers gathered before alignment froze.
                for (const auto& fp : lost_buffer_) {
                    pushLost(fp.first, fp.second);
                }
                lost_buffer_.clear();
            }
        } else {
            pushEstimate(static_cast<int>(r.frame_id), r.T_wc);
        }
    }

    // ── Map-point cloud: push the latest keyframe cloud through the frozen
    //    alignment (only once the transform exists). ──────────────────────────
    if (aligner_.isFrozen() && map_dirty_) {
        map_dirty_ = false;
        std::vector<Eigen::Vector3f> pts_enu;
        pts_enu.reserve(latest_map_points_.size());
        for (const Vec3_t& p : latest_map_points_) {
            const Eigen::Vector3d en = aligner_.applyPoint(p);
            pts_enu.emplace_back(static_cast<float>(en.x()),
                                 static_cast<float>(en.y()),
                                 static_cast<float>(en.z()));
        }
        if (!pts_enu.empty()) {
            viewer_.pushMapPoints(pts_enu);
        }
    }
}

// Fires on the FUSION thread in async mode — the DebugViewer push methods are
// thread-safe. The orientation is passed through as-is: the constant yaw offset
// θ (mount azimuth) baked into T_enu_c is EXPECTED — do NOT correct it; seeing
// it on the fused pose axes is diagnostic.
void SystemBridge::onFusionResult(const fusion::FusionResult& fres) {
    if (!fres.has_pose || beyondCapFrame(fres.frame_id)) {
        return;
    }
    const Eigen::Vector3d    f = fres.T_enu_c.translation();
    const Eigen::Quaterniond q(fres.T_enu_c.rotation());
    std::lock_guard<std::mutex> lk(fused_mutex_);
    latest_fused_pos_ = f;
    latest_fused_set_ = true;
    if (!fused_f0_set_) {
        fused_f0_     = f;
        fused_f0_set_ = true;
    }
    if (!fused_g0_set_) {
        if (!latest_gt_valid_) {
            // no GT yet — buffer until g0 exists
            fused_pending_.emplace_back(fres.frame_id, f, q);
            return;
        }
        fused_g0_     = latest_gt_enu_;
        fused_g0_set_ = true;
        for (const auto& [fid, fp, fq] : fused_pending_) {
            const Eigen::Vector3d d = fused_g0_ + (fp - fused_f0_);
            viewer_.pushFusedPose(d.cast<float>(), fq.cast<float>(), fid);
            if (fused_sink_) {
                fused_sink_(static_cast<unsigned int>(fid), d);
            }
            ++fused_pushed_;
            // Offered, not forced: these predate the first groundtruth, so the
            // id will not match and they land in the skipped counter instead of
            // becoming mis-paired samples.
            pushErrorSample(fid, d);
        }
        fused_pending_.clear();
    }
    const Eigen::Vector3d d = fused_g0_ + (f - fused_f0_);
    viewer_.pushFusedPose(d.cast<float>(), q.cast<float>(), fres.frame_id);
    if (fused_sink_) {
        fused_sink_(fres.frame_id, d);
    }
    ++fused_pushed_;
    pushErrorSample(fres.frame_id, d);
}

// Keyframe smoother update: redraw the lag window with the corrected poses.
// SystemManager emits it right after the fusion result it belongs to, on the
// same thread, and only when the graph was actually updated. Corrections before
// the first GT anchor are meaningless — skipped. Every lag pose goes through the
// SAME anchor mapping as the live pushes: g0 + (p - f0), quaternion passthrough.
void SystemBridge::onLagWindow(const std::vector<fusion::FusionLagPose>& window) {
    std::lock_guard<std::mutex> lk(fused_mutex_);
    if (!fused_g0_set_) {
        return;
    }
    std::vector<FusedCorrection> corrected;
    corrected.reserve(window.size());
    for (const auto& lp : window) {
        const Eigen::Vector3d    p = lp.T_enu_c.translation();
        const Eigen::Quaterniond lq(lp.T_enu_c.rotation());
        const Eigen::Vector3d    dp = fused_g0_ + (p - fused_f0_);
        corrected.push_back({static_cast<uint64_t>(lp.frame_id),
                             dp.cast<float>(), lq.cast<float>()});
    }
    if (!corrected.empty()) {
        viewer_.pushFusedCorrection(corrected);
        ++fused_corrections_;
    }
}

// Channel 5. Runs on the MONITOR thread (not the pipeline thread), on a fixed
// cadence independent of processing load — so it keeps arriving, and keeps the
// HUD honest, while the pipeline is stalled. Does nothing but repackage: no
// alignment, no drawing, no allocation beyond the POD, because a slow handler
// here would delay the monitor thread's next tick.
void SystemBridge::onStats(const core::SystemStats& st) {
    ThroughputSample ts;
    ts.fps_windowed = st.fps_windowed;
    ts.fps_mean     = st.fps_processed;
    ts.proc_ms_mean = st.proc_ms_mean;
    ts.window_sec   = st.fps_window_sec;
    viewer_.pushThroughput(ts);
}

// ---------------------------------------------------------------------------
// Absolute-fix markers
// ---------------------------------------------------------------------------
void SystemBridge::pushAnchorFix(const Eigen::Vector2d& xy_fusion_enu,
                                 bool from_reinit) {
    AnchorFixRecord r;
    r.pos_fusion.head<2>() = xy_fusion_enu;
    r.from_reinit          = from_reinit;
    {
        // A fix is horizontal-only; draw it at the altitude the back-end
        // currently believes in, so the marker sits ON the fused line instead
        // of on the ground plane.
        std::lock_guard<std::mutex> lk(fused_mutex_);
        if (latest_fused_set_) {
            r.pos_fusion.z() = latest_fused_pos_.z();
        }
    }
    std::lock_guard<std::mutex> lk(anchor_mutex_);
    anchor_fixes_.push_back(r);
    anchor_pending_.push_back(anchor_fixes_.size() - 1);
    ++anchor_generated_;
    if (r.from_reinit) ++anchor_reinit_generated_;
    anchor_dirty_ = true;
}

void SystemBridge::anchorPoll() {
    {
        std::lock_guard<std::mutex> lk(anchor_mutex_);
        if (!anchor_tracking_) {
            return;
        }
    }
    const fusion::FusionFixStats st = sys_->fixStats();

    std::string markers_stats;
    std::string markers_warning;
    std::vector<AnchorFixMarker> markers;
    {
        std::lock_guard<std::mutex> lk(anchor_mutex_);
        // Verdicts arrive as counter DELTAS, attributed to the oldest
        // unresolved fixes in emission order — the module consumes them FIFO.
        auto take = [&](unsigned long long n, bool accepted) {
            for (unsigned long long i = 0; i < n && !anchor_pending_.empty(); ++i) {
                const std::size_t idx = anchor_pending_.front();
                anchor_pending_.pop_front();
                anchor_fixes_[idx].resolved = true;
                anchor_fixes_[idx].accepted = accepted;
                if (accepted) ++anchor_accepted_; else ++anchor_rejected_;
                if (accepted && anchor_fixes_[idx].from_reinit) {
                    ++anchor_reinit_accepted_;
                }
                anchor_dirty_ = true;
            }
        };
        take(st.applied - anchor_prev_stats_.applied, true);
        // Everything that is not "applied" is a rejection as far as the picture
        // is concerned; the log and test_full_flight's CSV carry the exact
        // reason, which a 3D marker cannot show anyway.
        take((st.gated          - anchor_prev_stats_.gated) +
             (st.low_confidence - anchor_prev_stats_.low_confidence) +
             (st.age_expired    - anchor_prev_stats_.age_expired) +
             (st.unmatched      - anchor_prev_stats_.unmatched) +
             (st.marginalized   - anchor_prev_stats_.marginalized) +
             (st.queue_dropped  - anchor_prev_stats_.queue_dropped) +
             (st.no_graph       - anchor_prev_stats_.no_graph), false);
        anchor_prev_stats_ = st;

        if (!anchor_dirty_) {
            return;
        }
        anchor_dirty_ = false;

        // Same display mapping as the fused line — g0 + (p - f0) — so a marker
        // lands where it belongs relative to the other trajectories.
        {
            std::lock_guard<std::mutex> lk_fused(fused_mutex_);
            if (!fused_f0_set_ || !fused_g0_set_) {
                anchor_dirty_ = true;  // no display anchor yet — retry next poll
                return;
            }
            markers.reserve(anchor_fixes_.size());
            for (const AnchorFixRecord& r : anchor_fixes_) {
                const Eigen::Vector3d d = fused_g0_ + (r.pos_fusion - fused_f0_);
                markers.push_back({d.cast<float>(), r.resolved && r.accepted,
                                   r.from_reinit});
            }
        }
        markers_warning = anchor_warning_;
        markers_stats =
            "Anchor fixes  generated " + std::to_string(anchor_generated_) +
            " | accepted " + std::to_string(anchor_accepted_) +
            " | rejected " + std::to_string(anchor_rejected_) +
            " | pending "  + std::to_string(anchor_pending_.size()) +
            "   ||  re-anchor (square) " +
            std::to_string(anchor_reinit_generated_) + " | accepted " +
            std::to_string(anchor_reinit_accepted_);
    }
    viewer_.setAnchorFixes(markers);
    viewer_.setAnchorOverlay(markers_warning, markers_stats);
}

// ---------------------------------------------------------------------------
// The run: auto-start, supervisor, performance sampler
// ---------------------------------------------------------------------------
bool SystemBridge::begin() {
    if (sys_ == nullptr) {
        return true;  // a bare viewer (no pipeline) — nothing to supervise
    }
    if (cfg_.autostart) {
        if (!sys_->start()) {
            spdlog::error("DebugViewer: SystemManager start failed");
            return false;
        }
        system_started_.store(true);
        spdlog::info("DebugViewer: run mode = AUTO-START ({})",
                     cfg_.autostart_reason);
        viewer_.setRunState(DebugViewer::RunState::RUNNING);
    } else {
        spdlog::info("DebugViewer: run mode = WAIT FOR THE VIEWER'S 'Start' "
                     "BUTTON ({}). Start/Stop then pauses and resumes the input "
                     "as often as you like.", cfg_.autostart_reason);
    }

    stop_requested_.store(false);

    // Supervisor: the only thing that decides WHEN the data is over. It stops
    // the system (which cuts the source, drains and joins) but leaves the
    // viewer window alone.
    supervisor_ = std::thread([this]() {
        while (!stop_requested_.load()) {
            if (cfg_.max_frames > 0 && frames_processed_.load() >= cfg_.max_frames) {
                break;
            }
            if (sys_->stats().source_ended) {
                break;
            }
            anchorPoll();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SUPERVISOR_POLL_MS));
        }
        sys_->stop();
        // Final sweep: the last keyframe update resolves whatever was still in
        // flight, and the picture must show those verdicts too.
        anchorPoll();
        // TERMINAL, not paused: sys_->stop() has shut the pipeline down for
        // good, so every later resumeSource() would be refused
        // (SystemManager::setSourceStreaming requires state == RUNNING). The
        // button therefore goes grey and says "Finished" instead of offering a
        // "Start" that could only fail silently — a display must not claim an
        // action is available when it is not.
        viewer_.setRunState(DebugViewer::RunState::FINISHED);
        spdlog::info("pipeline finished ({} frames) — viewer stays open, "
                     "close the window to exit", frames_processed_.load());
    });

    // Performance sampler: a tiny dedicated thread that every `perf_sample_ms`
    // takes a util::SysMonitor sample (CPU % + RSS MB from /proc, measured
    // inside the util module) and pushes a PerfSample into the viewer's
    // "Performance" streaming charts. Producer-side measurement — the viewer
    // only plots (module decoupling). Terminates on stop_requested_.
    perf_sampler_ = std::thread([this]() {
        uavloc::util::SysMonitor mon;
        while (!stop_requested_.load()) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(perf_sample_ms_));
            uavloc::util::SysStats s;
            if (mon.sample(s)) {
                viewer_.pushPerf(PerfSample{s.t_sec, s.cpu_percent, s.rss_mb});
                ++perf_samples_;
            }
        }
    });
    return true;
}

void SystemBridge::end(bool had_display) {
    if (!supervisor_.joinable() && !perf_sampler_.joinable()) {
        return;
    }
    // With a display, run() only returns once the window is closed — whether
    // mid-run (the flag makes the supervisor tear the system down) or after the
    // data ended (the supervisor already did). Headless, run() returned at once
    // and the join below simply waits for the pipeline to finish on its own.
    if (had_display) {
        stop_requested_.store(true);
    }
    if (supervisor_.joinable()) {
        supervisor_.join();
    }
    stop_requested_.store(true);  // sole sampler-exit signal on the headless path
    if (perf_sampler_.joinable()) {
        perf_sampler_.join();
    }
    spdlog::info("perf sampler: {} samples", perf_samples_.load());
    spdlog::info("fused poses pushed: {}", fused_pushed_.load());
    spdlog::info("fused corrections applied: {}", fused_corrections_.load());
    // Feed check for the "Error (m)" window (plumbing, not accuracy): skipped
    // frames are the ones with no groundtruth of the SAME id — mostly the poses
    // flushed out of fused_pending_ before the first GT existed.
    spdlog::info("error samples pushed: {} (skipped, no same-frame GT: {})",
                 error_samples_pushed_, error_samples_skipped_);
}

} // namespace uavloc::debug_viewer
