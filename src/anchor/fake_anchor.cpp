#include "uavloc/anchor/fake_anchor.h"

#include "uavloc/sensor/geo_reference.h"
#include "uavloc/util/threadsafe_queue.h"

#include <spdlog/spdlog.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <random>
#include <thread>
#include <utility>
#include <vector>

namespace uavloc::anchor {

namespace {

//! Smallest sigma the generator will use. A zero sigma would make the declared
//! covariance singular, which the back-end rejects outright — so a config that
//! asks for "no noise" gets the smallest NON-degenerate noise instead, loudly.
constexpr double MIN_SIGMA_M = 1e-6;

const char* mode_name(FakeAnchorMode m) {
    switch (m) {
        case FakeAnchorMode::SYNCHRONOUS: return "SYNCHRONOUS";
        case FakeAnchorMode::ASYNC:       return "ASYNC";
    }
    return "UNKNOWN";
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

FakeAnchorConfig FakeAnchorConfig::fromYaml(const YAML::Node& node) {
    FakeAnchorConfig c;
    if (!node || !node.IsMap()) {
        return c;
    }
    c.telemetry       = sensor::DroneTelemetryConfig::fromYaml(node["DroneTelemetry"]);
    c.frame_id_offset = node["frame_id_offset"].as<long long>(c.frame_id_offset);
    c.sigma_m         = node["sigma_m"].as<double>(c.sigma_m);
    c.every_kf        = node["every_kf"].as<int>(c.every_kf);
    c.reinit_min_kf_gap =
        node["reinit_min_kf_gap"].as<int>(c.reinit_min_kf_gap);
    c.outlier_rate    = node["outlier_rate"].as<double>(c.outlier_rate);
    c.outlier_min_m   = node["outlier_min_m"].as<double>(c.outlier_min_m);
    c.outlier_max_m   = node["outlier_max_m"].as<double>(c.outlier_max_m);
    c.latency_kf      = node["latency_kf"].as<int>(c.latency_kf);
    c.seed = static_cast<unsigned int>(node["seed"].as<long long>(
        static_cast<long long>(c.seed)));
    const std::string mode = node["mode"].as<std::string>("synchronous");
    if (mode == "async") {
        c.mode = FakeAnchorMode::ASYNC;
    } else if (mode != "synchronous") {
        spdlog::warn("FakeAnchorConfig: unknown mode '{}' — expected "
                     "synchronous|async; using synchronous", mode);
    }
    c.queue_capacity = node["queue_capacity"].as<std::size_t>(c.queue_capacity);
    return c;
}

// ─────────────────────────────────────────────────────────────────────────────

class FakeAnchor::Impl {
public:
    explicit Impl(const FakeAnchorConfig& config)
        : config_(sanitize(config)),
          reader_(config_.telemetry),
          rng_(config_.seed) {}

    ~Impl() { stop(); }

    bool setup() {
        if (setup_done_) {
            spdlog::error("FakeAnchor::setup: already set up — ignoring");
            return false;
        }
        if (config_.telemetry.csv_path.empty()) {
            spdlog::error("FakeAnchor::setup: no groundtruth CSV configured — a "
                          "fake anchor has nothing to read");
            return false;
        }
        if (!reader_.load()) {
            spdlog::error("FakeAnchor::setup: cannot load groundtruth CSV '{}'",
                          config_.telemetry.csv_path);
            return false;
        }
        setup_done_ = true;
        spdlog::warn("FakeAnchor: FIXES ARE GENERATED FROM GROUNDTRUTH — every "
                     "error figure of this run measures the back-end, NOT a real "
                     "system");
        spdlog::info("FakeAnchor: set up ({} groundtruth rows from '{}', "
                     "frame_id_offset={}, sigma={:.1f} m, every {} requests, "
                     "reinit_min_kf_gap={}, outlier_rate={:.2f} [{:.0f}, {:.0f}] "
                     "m, latency={} requests, seed={}, mode={})",
                     reader_.size(), config_.telemetry.csv_path,
                     config_.frame_id_offset, config_.sigma_m, config_.every_kf,
                     config_.reinit_min_kf_gap, config_.outlier_rate,
                     config_.outlier_min_m, config_.outlier_max_m,
                     config_.latency_kf, config_.seed, mode_name(config_.mode));
        return true;
    }

    bool start() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!setup_done_) {
            spdlog::error("FakeAnchor::start: setup() has not succeeded");
            return false;
        }
        if (running_.load()) {
            spdlog::error("FakeAnchor::start: already started");
            return false;
        }
        if (config_.mode == FakeAnchorMode::ASYNC) {
            queue_ = std::make_unique<util::ThreadsafeQueue<AnchorQuery>>(
                config_.queue_capacity);
            try {
                worker_ = std::thread([this] { run(); });
            } catch (const std::system_error& ex) {
                spdlog::error("FakeAnchor::start: cannot spawn the worker "
                              "thread: {}", ex.what());
                queue_.reset();
                return false;
            }
        }
        running_.store(true);
        return true;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!running_.exchange(false)) {
            return;
        }
        if (queue_) {
            queue_->shutdown();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        queue_.reset();
        const FakeAnchorStats st = stats();
        spdlog::info("FakeAnchor: stopped (requested={}, generated={} "
                     "[{} outliers], emitted={}, skipped: cadence={} "
                     "no_telemetry={} no_groundtruth={} no_origin={}, "
                     "dropped_busy={})",
                     st.requested, st.generated, st.outliers, st.emitted,
                     st.skipped_cadence, st.skipped_no_telemetry,
                     st.skipped_no_groundtruth, st.skipped_no_origin,
                     st.dropped_busy);
        // Kept on its own line: a re-anchor fix folded into `generated` would be
        // indistinguishable from a cadence fix, and then no run could ever say
        // whether the re-anchor path fired at all.
        spdlog::info("FakeAnchor: re-anchor (reason=REINIT): requested={}, "
                     "generated={} of {} total, throttled={} "
                     "(reinit_min_kf_gap={})",
                     st.reinit_requested, st.reinit_generated, st.generated,
                     st.reinit_throttled, config_.reinit_min_kf_gap);
    }

    void setResultCallback(ResultCallback cb) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        result_cb_ = std::move(cb);
    }

    void setGenerationCallback(GenerationCallback cb) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        generation_cb_ = std::move(cb);
    }

    void setEnuOrigin(double lat0_deg, double lon0_deg) {
        std::lock_guard<std::mutex> lock(origin_mutex_);
        if (geo_.initialized()) {
            spdlog::warn("FakeAnchor::setEnuOrigin: origin already set — "
                         "ignoring the second one (lat0={:.7f}, lon0={:.7f})",
                         lat0_deg, lon0_deg);
            return;
        }
        // alt0 = 0 and psi = 0: only the horizontal flat-earth mapping is used
        // (enu() ignores the rotation, which latlon() would apply).
        geo_.init(lat0_deg, lon0_deg, 0.0, 0.0);
        spdlog::info("FakeAnchor: ENU origin set by the owner "
                     "(lat0={:.7f}, lon0={:.7f})", lat0_deg, lon0_deg);
    }

    bool requestFix(const AnchorQuery& q) {
        if (!running_.load()) {
            spdlog::debug("FakeAnchor::requestFix: not started — request dropped");
            return false;
        }
        if (config_.mode == FakeAnchorMode::SYNCHRONOUS) {
            return fake_run(q);
        }
        if (!queue_->push_dropping_if_full(q)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.dropped_busy;
            return false;
        }
        return true;
    }

    FakeAnchorStats stats() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

private:
    //! A produced fix on its way to the result callback, together with the
    //! provenance the EMIT trace needs: WHICH query made it (the correlation
    //! key of the anchor[REQ]/anchor[EMIT]/anchor[APPLY] triple) and on which
    //! request index, so the latency can be measured instead of assumed.
    struct PendingEmit {
        AbsoluteFix         fix;
        unsigned int        frame_id         = 0;
        AnchorRequestReason reason           = AnchorRequestReason::KEYFRAME;
        //! True when the fix skipped the duty cycle (a REINIT that was not
        //! throttled) — `reason` says what was ASKED, this says what happened.
        bool                bypassed_cadence = false;
        bool                is_outlier       = false;
        //! Request index the fix was generated on.
        unsigned long long  gen_seq          = 0;
    };

    //! A produced fix waiting out its artificial latency.
    struct DelayedFix {
        PendingEmit        item;
        unsigned long long release_seq = 0;
    };

    //! Clamp a configuration into the range the generator can honour, loudly.
    static FakeAnchorConfig sanitize(const FakeAnchorConfig& in) {
        FakeAnchorConfig c = in;
        if (c.every_kf < 1) {
            spdlog::warn("FakeAnchor: every_kf={} is below 1 — using 1", c.every_kf);
            c.every_kf = 1;
        }
        if (c.sigma_m <= 0.0) {
            spdlog::warn("FakeAnchor: sigma_m={} is not positive — using {} (a "
                         "zero sigma makes the declared covariance singular, "
                         "which the back-end rejects)", c.sigma_m, MIN_SIGMA_M);
            c.sigma_m = MIN_SIGMA_M;
        }
        if (c.outlier_max_m < c.outlier_min_m) {
            spdlog::warn("FakeAnchor: outlier_max_m={} < outlier_min_m={} — "
                         "using a degenerate band at the minimum",
                         c.outlier_max_m, c.outlier_min_m);
            c.outlier_max_m = c.outlier_min_m;
        }
        if (c.latency_kf < 0) {
            spdlog::warn("FakeAnchor: latency_kf={} is negative — using 0",
                         c.latency_kf);
            c.latency_kf = 0;
        }
        if (c.reinit_min_kf_gap < 0) {
            spdlog::warn("FakeAnchor: reinit_min_kf_gap={} is negative — using 0 "
                         "(no limit on the re-anchor bypass)",
                         c.reinit_min_kf_gap);
            c.reinit_min_kf_gap = 0;
        }
        return c;
    }

    //! Body of the worker thread (ASYNC only) — the shape of kcb's
    //! SatcomLightglue::Run(): pop a request, compute, publish, repeat.
    void run() {
        spdlog::debug("FakeAnchor: worker thread started");
        AnchorQuery q;
        while (queue_->pop_blocking(q)) {
            fake_run(q);
            q.image.release();  // do not pin a frame buffer while idle
        }
        spdlog::debug("FakeAnchor: worker thread stopped");
    }

    //! THE fake "recognition". The generator state (RNG stream, duty-cycle
    //! counter, latency queue) is single-writer under gen_mutex_; the callbacks
    //! fire AFTER that lock is released, so a sink is free to call straight
    //! back into the pipeline.
    //!
    //! Returns true when the request was accepted for processing — which it
    //! always is once we got here; the return value of requestFix() reports
    //! ACCEPTANCE, not production (see AnchorInterface).
    bool fake_run(const AnchorQuery& q) {
        std::vector<PendingEmit> to_emit;
        FakeFixRecord            record;
        bool                     have_record = false;
        unsigned long long       seq_now     = 0;
        {
            std::lock_guard<std::mutex> lock(gen_mutex_);
            {
                std::lock_guard<std::mutex> slock(stats_mutex_);
                ++stats_.requested;
            }
            ++request_seq_;
            seq_now = request_seq_;

            have_record = maybeGenerate(q, to_emit, record);

            // Release whatever the artificial latency was holding: those fixes
            // keep their ORIGINAL timestamp, so the smoother has to reach back
            // into the lag window to apply them. Done AFTER generation, so a
            // fix produced on this very request with latency 0 leaves first —
            // the order the driver used before the generator moved in here.
            while (!delayed_.empty() &&
                   delayed_.front().release_seq <= request_seq_) {
                to_emit.push_back(delayed_.front().item);
                delayed_.pop_front();
            }
        }
        if (have_record) {
            publishRecord(record);
        }
        for (const PendingEmit& item : to_emit) {
            emit(item, seq_now, q.timestamp_msec);
        }
        return true;
    }

    //! Generate at most one fix. Returns true when `record` was filled; a fix
    //! that is emitted immediately is appended to `to_emit`, one held back by
    //! latency_kf goes into the delay queue instead.
    bool maybeGenerate(const AnchorQuery& q, std::vector<PendingEmit>& to_emit,
                       FakeFixRecord& record) {
        // ── the CADENCE gate, and the one reason allowed to skip it ──────────
        // A REINIT is an event, not a tick: the VO chain just restarted, so the
        // absolute measurement is worth the most exactly here — letting a duty
        // cycle decide would throw away the best instant of the whole flight.
        // The three gates BELOW are data conditions, not cadence, and apply to
        // a REINIT unchanged.
        bool bypass_cadence = false;
        if (q.reason == AnchorRequestReason::REINIT) {
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.reinit_requested;
            }
            const auto gap =
                static_cast<unsigned long long>(config_.reinit_min_kf_gap);
            const bool too_soon =
                gap > 0 && have_reinit_gen_ &&
                (request_seq_ - last_reinit_gen_seq_) < gap;
            if (too_soon) {
                // Demoted, NOT dropped: a flickering VO must not be able to
                // flood the producer, but the request still gets its ordinary
                // chance at the duty cycle below.
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.reinit_throttled;
            } else {
                bypass_cadence = true;
            }
        }
        if (!bypass_cadence &&
            request_seq_ - last_gen_seq_ < static_cast<unsigned long long>(
                                               config_.every_kf)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.skipped_cadence;
            return false;
        }
        // The pipeline had no usable telemetry for this frame, so the
        // groundtruth of the instant is not trustworthy either. Skipping keeps
        // the fake producer on exactly the frames the evaluation scores.
        if (!(q.agl_m > 0.0)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.skipped_no_telemetry;
            return false;
        }
        Eigen::Vector2d truth;
        if (!hasOrigin()) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.skipped_no_origin;
            return false;
        }
        if (!groundtruthEnu(q.frame_id, truth)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.skipped_no_groundtruth;
            return false;
        }
        // Resetting the cadence counter here is what keeps a re-anchor fix from
        // being followed one request later by a regular one: the duty cycle
        // restarts from THIS fix, whichever reason produced it.
        last_gen_seq_ = request_seq_;
        if (bypass_cadence) {
            last_reinit_gen_seq_ = request_seq_;
            have_reinit_gen_     = true;
        }

        // The outlier draw is UNCONDITIONAL so that two runs differing only in
        // outlier_rate share the same noise realization.
        const double u_outlier  = unit_(rng_);
        const bool   is_outlier = u_outlier < config_.outlier_rate;

        Eigen::Vector2d xy = truth;
        if (is_outlier) {
            const double dir = 2.0 * M_PI * unit_(rng_);
            const double mag = config_.outlier_min_m +
                               (config_.outlier_max_m - config_.outlier_min_m) *
                                   unit_(rng_);
            xy += Eigen::Vector2d(mag * std::cos(dir), mag * std::sin(dir));
        } else {
            xy += config_.sigma_m * Eigen::Vector2d(gauss_(rng_), gauss_(rng_));
        }

        AbsoluteFix fix;
        fix.timestamp_msec = q.timestamp_msec;
        fix.xy_enu         = xy;
        // An outlier still DECLARES the honest sigma — that mismatch is exactly
        // what makes it an outlier for a consumer-side gate to catch.
        fix.cov = config_.sigma_m * config_.sigma_m * Eigen::Matrix2d::Identity();
        fix.confidence = 1.0;
        fix.valid      = true;

        const bool hold = config_.latency_kf > 0;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.generated;
            if (is_outlier) ++stats_.outliers;
            if (bypass_cadence) ++stats_.reinit_generated;
        }

        record.fix         = fix;
        record.is_outlier  = is_outlier;
        record.truth_enu   = truth;
        record.error_m     = (xy - truth).norm();
        record.frame_id    = q.frame_id;
        record.emitted_now = !hold;
        record.from_reinit = bypass_cadence;

        PendingEmit item;
        item.fix              = fix;
        item.frame_id         = q.frame_id;
        item.reason           = q.reason;
        item.bypassed_cadence = bypass_cadence;
        item.is_outlier       = is_outlier;
        item.gen_seq          = request_seq_;

        if (hold) {
            delayed_.push_back({item, request_seq_ + static_cast<unsigned long long>(
                                                         config_.latency_kf)});
        } else {
            to_emit.push_back(item);
        }
        return true;
    }

    bool hasOrigin() const {
        std::lock_guard<std::mutex> lock(origin_mutex_);
        return geo_.initialized();
    }

    //! Groundtruth ENU (East, North) of `frame_id`, against the owner-supplied
    //! origin. False when there is no origin yet or no usable CSV row.
    bool groundtruthEnu(unsigned int frame_id, Eigen::Vector2d& out) const {
        std::lock_guard<std::mutex> lock(origin_mutex_);
        if (!geo_.initialized()) {
            return false;
        }
        const std::int64_t key =
            static_cast<std::int64_t>(frame_id) + config_.frame_id_offset;
        if (key < 0) {
            return false;
        }
        const sensor::DroneTelemetryRecord rec =
            reader_.byFrameId(static_cast<std::uint64_t>(key));
        if (!rec.valid) {
            return false;
        }
        const Eigen::Vector3d p =
            geo_.enu(rec.latitude_deg, rec.longitude_deg, 0.0);
        out = p.head<2>();
        return true;
    }

    //! `seq_now` is the request index this emission happens ON, `now_ts_msec`
    //! that request's timestamp — the two references the EMIT latency is
    //! measured against (they differ from the generating request only when
    //! latency_kf > 0).
    void emit(const PendingEmit& item, unsigned long long seq_now,
              double now_ts_msec) {
        ResultCallback cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            cb = result_cb_;
        }
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.emitted;
        }
        // Stage 2 of the three-stage absolute-measurement trace
        // (anchor[REQ] → anchor[EMIT] → anchor[APPLY]). `frame`/`ts` are the
        // query this fix was MADE from, not the one it leaves on — that is the
        // whole point of the two latency figures next to them. `sigma` is the
        // DECLARED one (an outlier declares the honest sigma; see
        // maybeGenerate), so an `outlier=yes` line with a small sigma is the
        // measurement lying, exactly as intended.
        spdlog::info("anchor[EMIT] frame={} ts={:.1f} xy_enu=[{:.2f}, {:.2f}] "
                     "sigma={:.2f} outlier={} reason={} bypass={} "
                     "latency_kf={} latency_ms={:.1f}",
                     item.frame_id, item.fix.timestamp_msec,
                     item.fix.xy_enu.x(), item.fix.xy_enu.y(),
                     std::sqrt(item.fix.cov(0, 0)),
                     item.is_outlier ? "yes" : "no",
                     item.reason == AnchorRequestReason::REINIT ? "REINIT"
                                                               : "KEYFRAME",
                     item.bypassed_cadence ? "yes" : "no",
                     seq_now - item.gen_seq,
                     now_ts_msec - item.fix.timestamp_msec);
        // Invoked WITHOUT any of our mutexes held: the sink is allowed to call
        // straight back into the pipeline.
        if (cb) {
            cb(item.fix);
        }
    }

    void publishRecord(const FakeFixRecord& rec) {
        GenerationCallback cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            cb = generation_cb_;
        }
        if (cb) {
            cb(rec);
        }
    }

    const FakeAnchorConfig config_;

    sensor::DroneTelemetryCsvReader reader_;
    bool                            setup_done_ = false;

    // Lifecycle.
    std::mutex        lifecycle_mutex_;
    std::atomic<bool> running_{false};
    std::thread       worker_;
    std::unique_ptr<util::ThreadsafeQueue<AnchorQuery>> queue_;

    // Callbacks.
    mutable std::mutex callback_mutex_;
    ResultCallback     result_cb_;
    GenerationCallback generation_cb_;

    // ENU origin, handed down by the owner (never read from our own config).
    mutable std::mutex    origin_mutex_;
    sensor::GeoReferencer geo_;

    // Generator state — single-writer under gen_mutex_.
    std::mutex                             gen_mutex_;
    std::mt19937                           rng_;
    std::normal_distribution<double>       gauss_{0.0, 1.0};
    std::uniform_real_distribution<double> unit_{0.0, 1.0};
    unsigned long long                     request_seq_  = 0;
    unsigned long long                     last_gen_seq_ = 0;
    //! Request index of the last fix that BYPASSED the cadence, and whether
    //! there has been one at all — the anti-flooding limit is measured against
    //! it, so the first re-anchor of a run is never throttled.
    unsigned long long                     last_reinit_gen_seq_ = 0;
    bool                                   have_reinit_gen_     = false;
    std::deque<DelayedFix>                 delayed_;

    mutable std::mutex stats_mutex_;
    FakeAnchorStats    stats_;
};

// ─────────────────────────────────────────────────────────────────────────────

FakeAnchor::FakeAnchor(const FakeAnchorConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

FakeAnchor::~FakeAnchor() = default;

bool FakeAnchor::setup() { return impl_->setup(); }
bool FakeAnchor::start() { return impl_->start(); }
void FakeAnchor::stop()  { impl_->stop(); }

void FakeAnchor::setResultCallback(ResultCallback cb) {
    impl_->setResultCallback(std::move(cb));
}

void FakeAnchor::setGenerationCallback(GenerationCallback cb) {
    impl_->setGenerationCallback(std::move(cb));
}

void FakeAnchor::setEnuOrigin(double lat0_deg, double lon0_deg) {
    impl_->setEnuOrigin(lat0_deg, lon0_deg);
}

bool FakeAnchor::requestFix(const AnchorQuery& q) { return impl_->requestFix(q); }

FakeAnchorStats FakeAnchor::stats() const { return impl_->stats(); }

} // namespace uavloc::anchor
