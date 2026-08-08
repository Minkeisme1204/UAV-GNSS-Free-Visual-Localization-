#pragma once

// SystemBridge — everything that turns core::SystemManager events into what the
// DebugViewer draws (S3), plus the two service threads that own the RUN (S4).
//
// Until S3 this code lived in tests/test_vo_viewer.cpp. It moved here for one
// reason: it is not test logic, it is the display half of the system, and the
// old placement forced every driver that wants a live window to re-implement
// four callbacks, a 4-DoF alignment, a display anchor and two threads.
//
// WHAT STAYED IN THE DRIVER, on purpose:
//   * the fused-pose DUMP (DebugViewer::FusedPoseSink) — a regression artifact
//     of the test, not a feature of the viewer;
//   * building and configuring the absolute-position producer (anchor::
//     FakeAnchor): that is system assembly. Only the drawing of its fixes and
//     the polling of the back-end's verdicts are here;
//   * the auto-start POLICY (headless / dump gate / explicit override). The
//     driver decides, Config::autostart carries the decision, this class only
//     executes it.
//
// ⚠ BIT-IDENTICAL CONSTRAINT: the sequence of fused display poses produced here
// is a byte-for-byte regression gate (tests/test_vo_viewer.cpp with
// UAVLOC_VIEWER_DUMP, compared against build/tests/s0_GOLDEN.csv). Formulas,
// ORDER of operations, accumulation order into the buffers and the mutex
// semantics are therefore load-bearing — this file is a MOVE of proven code,
// not a rewrite.
//
// THREADING (unchanged from the driver): on_frame_processed and on_vo_data are
// emitted back to back on the pipeline thread for one frame, so the state they
// share needs no lock. on_fusion_result / on_lag_window may fire on the fusion
// thread, so everything they touch is guarded by fused_mutex_. The verdict
// polling runs on the supervisor thread and has its own anchor_mutex_.

#include "uavloc/debug_viewer/debug_viewer.h"
#include "uavloc/debug_viewer/trajectory_aligner.h"

#include "uavloc/common/type.h"
#include "uavloc/core/system_manager.h"
#include "uavloc/core/system_types.h"
#include "uavloc/fusion/fusion_data.h"
#include "uavloc/new_vo/vo_module.h"
#include "uavloc/sensor/telemetry_data.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace uavloc::debug_viewer {

class SystemBridge {
public:
    SystemBridge(DebugViewer& viewer, const DebugViewer::Config& cfg);
    ~SystemBridge();

    SystemBridge(const SystemBridge&)            = delete;
    SystemBridge& operator=(const SystemBridge&) = delete;

    //! Subscribe to the four display channels and install the run control.
    //! Returns false when a system is already attached.
    bool attach(core::SystemManager& sys);

    void setFusedPoseSink(DebugViewer::FusedPoseSink sink);
    void pushAnchorFix(const Eigen::Vector2d& xy_fusion_enu, bool from_reinit);
    void enableAnchorFixTracking(const std::string& warning);
    AnchorFixCounters anchorFixCounters() const;

    //! Auto-start (when configured) and spawn the supervisor + performance
    //! sampler. Returns false when the configured auto-start was REFUSED, in
    //! which case no thread was spawned and run() must not enter its loop.
    //! No-op returning true when no system is attached.
    bool begin();

    //! Join the supervisor and the sampler, in the order the driver used:
    //! a GUI run (the window has just been closed) tears the pipeline down,
    //! a headless run lets it finish by itself first.
    void end(bool had_display);

private:
    //! One absolute fix as the display tracks it: the position the producer
    //! reported (FUSION ENU — the same frame as fusion::FusionResult::T_enu_c)
    //! plus the consumer's verdict, which only arrives a few frames later.
    struct AnchorFixRecord {
        Eigen::Vector3d pos_fusion = Eigen::Vector3d::Zero();
        bool            resolved   = false;
        bool            accepted   = false;
        //! Produced by a RE-ANCHOR request (VO had just re-initialized) rather
        //! than by the regular cadence — drawn as a square marker and counted
        //! apart.
        bool            from_reinit = false;
    };

    // ── channel handlers (see the emission order in SystemCallbacks) ─────────
    void onFrameProcessed(const core::FrameProcessed& fp);
    void onVoData(const vo::VOData& data);
    void onFusionResult(const fusion::FusionResult& fres);
    void onLagWindow(const std::vector<fusion::FusionLagPose>& window);

    // ── helpers, all moved verbatim from the driver ─────────────────────────
    void pushProfileSnapshot(int frame_id);
    void pushLost(int fid, const Eigen::Vector3d& p_world);
    void pushEstimate(int fid, const Eigen::Matrix4d& T_wc);
    //! MUST be called with fused_mutex_ held.
    void pushErrorSample(uint64_t fid, const Eigen::Vector3d& pred);
    //! Poll the back-end's verdicts and repaint the markers. SUPERVISOR thread
    //! only, never from a SystemManager callback (design §R-c).
    void anchorPoll();

    bool beyondCapIndex(std::size_t index) const;
    bool beyondCapFrame(unsigned long long frame_id) const;

    DebugViewer&        viewer_;
    DebugViewer::Config cfg_;

    core::SystemManager* sys_ = nullptr;

    // Cached, clamped presentation parameters (identical to the driver's).
    const int    video_stride_;
    const double hud_min_baseline_m_;
    const int    perf_sample_ms_;

    //! Wall-clock reference for the "Profiling" table timestamps.
    const std::chrono::steady_clock::time_point profile_t0_;

    // ── run bookkeeping ─────────────────────────────────────────────────────
    std::atomic<bool>        stop_requested_{false};
    std::atomic<std::size_t> frames_processed_{0};
    //! True once SystemManager::start() has succeeded — the first Start brings
    //! the system up, every later one only resumes the reader.
    std::atomic<bool>        system_started_{false};
    //! UAVLOC_VO_MAXFRAMES, made EXACT on the output side.
    std::atomic<bool>               cap_reached_{false};
    std::atomic<unsigned long long> cap_last_frame_id_{0};

    std::atomic<std::size_t> fused_pushed_{0};
    std::atomic<std::size_t> fused_corrections_{0};
    std::atomic<std::size_t> perf_samples_{0};

    // ── PIPELINE-THREAD state (no lock: one thread, back-to-back callbacks) ──
    cv::Mat               frame_image_;
    sensor::TelemetryData frame_telemetry_;
    bool                  frame_has_telemetry_ = false;

    std::vector<Vec3_t> latest_map_points_;
    bool                map_dirty_ = false;

    TrajectoryAligner aligner_;
    std::vector<std::pair<int, Eigen::Matrix4d>,
                Eigen::aligned_allocator<std::pair<int, Eigen::Matrix4d>>>
                                                 est_buffer_;
    std::vector<std::pair<int, Eigen::Vector3d>> lost_buffer_;

    Eigen::Vector3d last_pos_             = Eigen::Vector3d::Zero();
    bool            has_last_pos_         = false;
    double          trajectory_length_m_  = 0.0;
    double          last_cog_heading_deg_ = 0.0;

    // ── Fused-trajectory display anchor (guarded by fused_mutex_) ───────────
    std::mutex      fused_mutex_;
    bool            latest_gt_valid_ = false;
    Eigen::Vector3d latest_gt_enu_   = Eigen::Vector3d::Zero();
    bool            fused_f0_set_    = false;
    bool            fused_g0_set_    = false;
    Eigen::Vector3d fused_f0_        = Eigen::Vector3d::Zero();
    Eigen::Vector3d fused_g0_        = Eigen::Vector3d::Zero();
    std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Quaterniond>>
                    fused_pending_;
    Eigen::Vector3d latest_fused_pos_ = Eigen::Vector3d::Zero();
    bool            latest_fused_set_ = false;

    // EXACT frame-id pairing for the "Error (m)" window.
    Eigen::Vector3d gt_exact_enu_      = Eigen::Vector3d::Zero();
    uint64_t        gt_exact_frame_id_ = 0;
    bool            gt_exact_valid_    = false;

    // ONE-FRAME-DEFERRED groundtruth publication (pipeline thread only).
    Eigen::Vector3d staged_gt_enu_   = Eigen::Vector3d::Zero();
    bool            staged_gt_valid_ = false;

    std::size_t error_samples_pushed_  = 0;
    std::size_t error_samples_skipped_ = 0;

    //! Set before the run starts, read under fused_mutex_ at every push.
    DebugViewer::FusedPoseSink fused_sink_;

    // ── absolute-fix markers (guarded by anchor_mutex_) ─────────────────────
    mutable std::mutex           anchor_mutex_;
    bool                         anchor_tracking_ = false;
    std::string                  anchor_warning_;
    std::vector<AnchorFixRecord> anchor_fixes_;
    std::deque<std::size_t>      anchor_pending_;  // indices, in emission order
    std::size_t                  anchor_generated_ = 0;
    std::size_t                  anchor_accepted_  = 0;
    std::size_t                  anchor_rejected_  = 0;
    std::size_t                  anchor_reinit_generated_ = 0;
    std::size_t                  anchor_reinit_accepted_  = 0;
    bool                         anchor_dirty_     = false;
    //! Supervisor-thread only: previous counter snapshot the deltas are taken
    //! against.
    fusion::FusionFixStats       anchor_prev_stats_{};

    std::thread supervisor_;
    std::thread perf_sampler_;
};

} // namespace uavloc::debug_viewer
