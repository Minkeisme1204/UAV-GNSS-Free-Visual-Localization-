#pragma once

// FakeAnchor — the M1 absolute-position producer: it does NOT recognise a
// place, it READS THE GROUNDTRUTH and adds noise.
//
// ⚠⚠ CONTAMINATED MEASUREMENT. Every fix this class emits is derived from the
// same telemetry the evaluation scores against. Any error figure produced with
// a FakeAnchor attached measures the BACK-END's ability to consume absolute
// positions and NOTHING about the accuracy of a real system. Report it with
// that sentence attached (.claude/rules/reporting.md), and put the warning
// where the numbers are shown — not in a footnote.
//
// It exists so that the fusion back-end, core::SystemManager and every driver
// can be built and measured against the FINAL interface before VPR exists;
// swapping in the real producer is then a one-line change at the call site
// (setAnchor), which is the whole point of AnchorInterface.
//
// The "recognition" step is fake_run(query):
//   1. look the groundtruth up at the query instant (telemetry CSV, by frame id)
//   2. convert it to ENU against the origin core::SystemManager handed down
//   3. add zero-mean Gaussian noise, sigma from the config, fixed seed
//   4. optionally replace the noise with a gross outlier
//   5. build the AbsoluteFix and hand it to the result callback

#include "uavloc/anchor/anchor_interface.h"
#include "uavloc/sensor/drone_telemetry_csv_reader.h"

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <functional>
#include <memory>

namespace uavloc::anchor {

//! How requestFix() delivers its result.
enum class FakeAnchorMode {
    //! fake_run() runs inline and the result callback fires BEFORE
    //! requestFix() returns. Deterministic — the mode every regression gate and
    //! every published number must use.
    SYNCHRONOUS,
    //! The query is queued and a worker thread runs fake_run(), so the callback
    //! fires on that thread. Shape of kcb's SatcomLightglue::Run(). Models the
    //! real timing of a producer that takes tens of milliseconds; NOT
    //! deterministic.
    ASYNC
};

struct FakeAnchorConfig {
    //! Groundtruth source: the same drone-log CSV the mission config feeds to
    //! sensor::VideoReader. FakeAnchor loads it a SECOND time, on purpose —
    //! that keeps it a self-contained producer instead of a privileged reader
    //! of the pipeline's telemetry.
    sensor::DroneTelemetryConfig telemetry;

    //! Added to AnchorQuery::frame_id to obtain the CSV lookup key, exactly as
    //! sensor::VideoReaderConfig::telemetry_frame_id_offset does (a cut video
    //! does not start at the log's first imageId).
    long long frame_id_offset = 0;

    //! Standard deviation [m] of the Gaussian noise added to a GOOD fix, per
    //! axis. Also what the emitted AbsoluteFix::cov declares.
    double sigma_m = 20.0;

    //! Duty cycle: emit at most one fix every N requests. A real producer runs
    //! on keyframes and cannot run on all of them, and the cadence is the
    //! PRODUCER's business — a caller must never have to know it.
    //!
    //! Applies to AnchorRequestReason::KEYFRAME requests only: a REINIT is an
    //! event, not a tick of the cadence (see AnchorRequestReason).
    int every_kf = 20;

    //! Anti-flooding limit on the REINIT bypass: a REINIT request is allowed to
    //! skip the duty cycle only when at least this many requests have passed
    //! since the last fix that DID skip it. A REINIT refused by this limit is
    //! not thrown away — it falls back to being an ordinary cadence request, so
    //! nothing is silently lost.
    //!
    //! 0 = no limit (every REINIT bypasses the cadence). That is the default on
    //! purpose: a non-zero value would silently drop measurements, and there is
    //! NO measured number to justify one yet. The right value has to come from
    //! an experiment on a flight where the VO actually flickers — until such a
    //! run exists, this key stays off.
    int reinit_min_kf_gap = 0;

    //! Probability that a fix is a gross outlier instead of a noisy-but-correct
    //! measurement, and the magnitude band [m] of that error. An outlier still
    //! declares the honest sigma in its covariance — that mismatch is exactly
    //! what a consumer-side gate is supposed to catch.
    double outlier_rate  = 0.0;
    double outlier_min_m = 200.0;
    double outlier_max_m = 1000.0;

    //! Hold a produced fix back for N further requests before emitting it. The
    //! fix keeps its ORIGINAL timestamp, so the smoother has to reach back into
    //! the lag window — which is the property being tested.
    int latency_kf = 0;

    //! Seed of the noise/outlier RNG. Same seed + same request sequence ⇒ same
    //! fixes, byte for byte.
    unsigned int seed = 42u;

    FakeAnchorMode mode = FakeAnchorMode::SYNCHRONOUS;

    //! ASYNC only: depth of the request queue. Drop-oldest — a stale query is
    //! worthless, and requestFix() must never block the pipeline.
    std::size_t queue_capacity = 8;

    //! Reads the "FakeAnchor" node (all keys optional; missing keys keep the
    //! defaults above). The groundtruth CSV is read from the nested
    //! "DroneTelemetry" node, same schema as sensor::VideoReader uses.
    static FakeAnchorConfig fromYaml(const YAML::Node& node);
};

//! What FakeAnchor knew at the moment it MANUFACTURED a fix. Exists only
//! because the fix is fake: an evaluation driver has to be able to score
//! "was this one an outlier, and did the back-end catch it", and
//! anchor::AbsoluteFix must never grow a field that only a fake producer can
//! fill.
struct FakeFixRecord {
    AbsoluteFix fix;
    //! True when the position was deliberately corrupted.
    bool is_outlier = false;
    //! The groundtruth ENU position [m] the fix was derived from.
    Eigen::Vector2d truth_enu = Eigen::Vector2d::Zero();
    //! |fix.xy_enu - truth_enu| [m].
    double error_m = 0.0;
    //! Frame the query came from (CSV row = frame_id + frame_id_offset).
    unsigned int frame_id = 0;
    //! False while the fix is held back by FakeAnchorConfig::latency_kf: the
    //! record is published at GENERATION time, the fix itself later.
    bool emitted_now = true;
    //! True when the query that produced this fix was an
    //! AnchorRequestReason::REINIT that actually bypassed the duty cycle. A
    //! REINIT that was throttled back to an ordinary cadence request reports
    //! false — the flag says how the fix was OBTAINED, not what was asked.
    bool from_reinit = false;
};

//! Cumulative accounting of the producer side. Every requestFix() call ends in
//! exactly one of the OUTCOME counters below (requested is the total); the
//! three reinit_* counters are a SEPARATE breakdown of the same calls by
//! request reason, and must not be added to the outcome ones.
struct FakeAnchorStats {
    unsigned long long requested = 0;
    //! Fixes manufactured (whether emitted immediately or held back).
    unsigned long long generated = 0;
    //! Of those, how many were deliberate outliers.
    unsigned long long outliers = 0;
    //! Fixes actually handed to the result callback.
    unsigned long long emitted = 0;
    //! Request fell between two duty-cycle slots — normal operation.
    unsigned long long skipped_cadence = 0;
    //! The query carried no usable telemetry (agl_m <= 0).
    unsigned long long skipped_no_telemetry = 0;
    //! No groundtruth row for that frame, or the row is unusable.
    unsigned long long skipped_no_groundtruth = 0;
    //! setEnuOrigin() has not been called yet — the fix would have no frame.
    unsigned long long skipped_no_origin = 0;
    //! ASYNC only: the request queue was full and an older query was dropped.
    unsigned long long dropped_busy = 0;

    // ── reason breakdown (orthogonal to the outcome counters above) ──────────
    //! Requests that arrived with AnchorRequestReason::REINIT.
    unsigned long long reinit_requested = 0;
    //! Fixes manufactured from a REINIT request that bypassed the duty cycle.
    //! `generated - reinit_generated` is therefore the ordinary-cadence yield —
    //! the two must stay separable, otherwise there is no way to tell whether
    //! re-anchoring ever fired.
    unsigned long long reinit_generated = 0;
    //! REINIT requests refused the bypass by FakeAnchorConfig::reinit_min_kf_gap
    //! and demoted to ordinary cadence requests (they may still have produced a
    //! fix through the normal duty cycle, counted as such).
    unsigned long long reinit_throttled = 0;
};

class FakeAnchor final : public AnchorInterface {
public:
    FakeAnchor() = delete;

    explicit FakeAnchor(const FakeAnchorConfig& config);

    ~FakeAnchor() override;

    FakeAnchor(const FakeAnchor&)            = delete;
    FakeAnchor& operator=(const FakeAnchor&) = delete;

    //! Loads the groundtruth CSV. False when it is missing or empty — a
    //! FakeAnchor without groundtruth has nothing to fake.
    bool setup() override;

    bool start() override;
    void stop() override;

    void setResultCallback(ResultCallback cb) override;
    void setEnuOrigin(double lat0_deg, double lon0_deg) override;
    bool requestFix(const AnchorQuery& q) override;

    //! Subscribe to the manufacturing record of every generated fix (see
    //! FakeFixRecord). Fires on the thread that runs fake_run(), immediately
    //! BEFORE the result callback when the fix is emitted at once, and at
    //! generation time when it is held back by latency_kf. Evaluation drivers
    //! only.
    using GenerationCallback = std::function<void(const FakeFixRecord&)>;
    void setGenerationCallback(GenerationCallback cb);

    FakeAnchorStats stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uavloc::anchor
