// test_data_source — unit test for the S6a push-style source layer:
// sensor::DataSourceInterface (4 typed channels + event channel) and
// sensor::VideoDataSource (the adapter that wraps a pull-style DataInterface).
//
// Checks:
//   1. all five channels wired, ~100 frames of a REAL mission dataset: each
//      channel receives exactly the expected number of calls (the three
//      telemetry channels once per frame that HAS telemetry, the image channel
//      once per decoded frame, the event channel one END_OF_STREAM);
//   2. PUBLISH ORDER: within one sample the image channel is always LAST —
//      the whole point of S6a, since the image is what triggers a processing
//      cycle downstream and the other three must already be buffered;
//   3. all four channels of one sample carry the SAME timestamp;
//   4. LOSSLESS REASSEMBLY: a TelemetryData rebuilt from the three telemetry
//      channels equals the TelemetryData the reader produced, field by field
//      (checked against a spy inserted between the adapter and the reader);
//   5. END_OF_STREAM is published (a) when the source reports it and (b) when
//      the source only ever answers EMPTY_FRAME — the P2-1 guard, which must
//      fire after exactly max_consecutive_bad_reads reads;
//   6. stopStreaming() mid-stream returns within its deadline, stops the
//      publishing, and a second call is safe;
//   7. clearCallbacks() silences every channel;
//   8. a sample with NO telemetry publishes the image channel ONLY (a source
//      never invents attitude/gimbal/gnss values).
//
// Checks 2-8 run on an in-test DataInterface double, so they need no dataset
// and are deterministic. Check 1 (which also feeds 2, 3 and 4) READS a mission
// config and soft-skips — still PASS — when its gitignored video is absent.
// Nothing under config/ is created or modified.
//
// EVERY wait has a deadline: a hang is reported as a failure instead of
// blocking forever (run_with_deadline / wait_until).
//
// Headless; exits non-zero on failure.

#include "uavloc/sensor/data_interface.h"
#include "uavloc/sensor/data_source_interface.h"
#include "uavloc/sensor/frame_data.h"
#include "uavloc/sensor/stream_types.h"
#include "uavloc/sensor/telemetry_data.h"
#include "uavloc/sensor/video_data_source.h"
#include "uavloc/sensor/video_reader.h"

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using uavloc::sensor::AttitudeData;
using uavloc::sensor::DataInterface;
using uavloc::sensor::FrameData;
using uavloc::sensor::FrameStatus;
using uavloc::sensor::GimbalData;
using uavloc::sensor::GnssData;
using uavloc::sensor::ImageData;
using uavloc::sensor::StreamEvent;
using uavloc::sensor::StreamEventKind;
using uavloc::sensor::TelemetryData;
using uavloc::sensor::VideoDataSource;
using uavloc::sensor::VideoDataSourceConfig;

//! Mission config of check 1. Provided by CMake so the test source carries no
//! hard-coded path; argv[1] overrides it.
#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH ""
#endif

//! Frames of the real dataset used by check 1. VideoReader::end_frame is
//! INCLUSIVE, so end_frame = REAL_RUN_FRAMES - 1 yields exactly this many.
constexpr int REAL_RUN_FRAMES = 100;

//! Frames of the shorter run that checks completion() (check 1i).
constexpr int COMPLETION_RUN_FRAMES = 20;

//! Deadline for any single blocking operation [ms]. Orders of magnitude above
//! what these checks program, so exceeding it means "stuck", not "slow".
constexpr int OP_DEADLINE_MS = 20000;

//! Deadline for polling a counter [ms].
constexpr int POLL_DEADLINE_MS = 20000;

//! Synthetic frame size of the double [px]. Content is irrelevant here — no
//! VO runs in this test — so the smallest sane image keeps the checks fast.
constexpr int SYN_W = 64;
constexpr int SYN_H = 48;

bool g_ok = true;

bool check(bool cond, const std::string& name) {
    if (cond) {
        spdlog::info("PASS: {}", name);
    } else {
        spdlog::error("FAIL: {}", name);
        g_ok = false;
    }
    return cond;
}

//! Runs `fn` on its own thread and hard-fails the process if it does not
//! return within `deadline` — a deadlocked stopStreaming() must be reported,
//! not waited on.
void run_with_deadline(const std::string& name, const std::function<void()>& fn,
                       std::chrono::milliseconds deadline) {
    std::promise<void> done;
    std::future<void>  fut = done.get_future();
    std::thread        th([&] {
        fn();
        done.set_value();
    });
    if (fut.wait_for(deadline) != std::future_status::ready) {
        spdlog::error("FAIL: {} did not finish within {} ms — HANG", name, deadline.count());
        spdlog::default_logger()->flush();
        std::_Exit(1);
    }
    th.join();
    check(true, name + " finished within its deadline");
}

//! Polls `pred` until it holds or the deadline expires; returns its final
//! value, which the caller turns into a PASS/FAIL.
template <typename Pred>
bool wait_until(Pred pred, std::chrono::milliseconds deadline) {
    const auto t_end = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < t_end) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

// ── test doubles ─────────────────────────────────────────────────────────────

//! A DataInterface that yields `good_frames` synthetic frames and then answers
//! `tail_status` forever. Deterministic telemetry so a reassembly check has an
//! exact expectation.
class FakeSource final : public DataInterface {
public:
    struct Options {
        int          good_frames    = 10;
        FrameStatus  tail_status    = FrameStatus::END_OF_STREAM;
        bool         with_telemetry = true;
        unsigned int read_delay_ms  = 0;
        bool         endless        = false;  //!< ignore good_frames, never end
    };

    explicit FakeSource(const Options& opt) : opt_(opt) {}

    bool open() override {
        opened_ = true;
        return true;
    }

    FrameStatus read(FrameData& frame) override {
        if (opt_.read_delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(opt_.read_delay_ms));
        }
        reads_.fetch_add(1);
        if (!opt_.endless && next_id_ >= static_cast<uint64_t>(opt_.good_frames)) {
            return opt_.tail_status;
        }

        frame.frame_id       = next_id_;
        frame.timestamp_msec = static_cast<double>(next_id_) * 33.0 + 0.5;
        frame.image          = cv::Mat(SYN_H, SYN_W, CV_8UC1, cv::Scalar(7));
        frame.camera_id      = "fake_cam";
        frame.source_name    = "fake_source";
        frame.status         = FrameStatus::OK;
        frame.valid          = true;
        if (opt_.with_telemetry) {
            frame.telemetry     = telemetry_for(next_id_);
            frame.has_telemetry = true;
        }
        ++next_id_;
        return FrameStatus::OK;
    }

    void        close() override { opened_ = false; }
    bool        isOpened() const override { return opened_; }
    std::string name() const override { return "fake_source"; }

    //! The telemetry the frame `id` carries — the expectation of check 4.
    static TelemetryData telemetry_for(uint64_t id) {
        const double k = static_cast<double>(id);
        TelemetryData t;
        t.frame_id        = id;
        t.timestamp_msec  = k * 33.0 + 0.5;
        t.roll_deg        = 1.0 + k * 0.25;
        t.pitch_deg       = -2.0 + k * 0.5;
        t.heading_deg     = 30.0 + k;
        t.gimbal_pan_deg  = 180.0;
        t.gimbal_tilt_deg = 90.0 - k * 0.125;
        t.latitude_deg    = 45.3 + k * 1e-5;
        t.longitude_deg   = -75.6 + k * 2e-5;
        t.altitude_m      = 100.0 + k * 0.75;
        t.speed_mps       = 30.0 - k * 0.1;
        t.valid           = true;
        return t;
    }

    int reads() const { return reads_.load(); }

private:
    Options          opt_;
    uint64_t         next_id_ = 0;
    bool             opened_  = false;
    std::atomic<int> reads_{0};
};

//! Passes every read straight through to the wrapped reader and keeps a copy of
//! what came out. Sits BETWEEN the adapter and the real VideoReader, which is
//! the only way check 4 can compare the published channels with the telemetry
//! the reader actually produced (the adapter owns the reader).
class SpySource final : public DataInterface {
public:
    explicit SpySource(std::unique_ptr<DataInterface> inner) : inner_(std::move(inner)) {}

    bool open() override { return inner_->open(); }

    FrameStatus read(FrameData& frame) override {
        const FrameStatus st = inner_->read(frame);
        if (st == FrameStatus::OK && frame.valid) {
            std::lock_guard<std::mutex> lock(mutex_);
            Seen s;
            s.frame_id       = frame.frame_id;
            s.timestamp_msec = frame.timestamp_msec;
            s.camera_id      = frame.camera_id;
            s.telemetry      = frame.telemetry;
            s.has_telemetry  = frame.has_telemetry;
            seen_.push_back(std::move(s));
        }
        return st;
    }

    void        close() override { inner_->close(); }
    bool        isOpened() const override { return inner_->isOpened(); }
    std::string name() const override { return inner_->name(); }

    struct Seen {
        uint64_t      frame_id       = 0;
        double        timestamp_msec = 0.0;
        std::string   camera_id;
        TelemetryData telemetry;
        bool          has_telemetry = false;
    };

    std::vector<Seen> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return seen_;
    }

private:
    std::unique_ptr<DataInterface> inner_;
    mutable std::mutex             mutex_;
    std::vector<Seen>              seen_;
};

//! Subscribes to all five channels and records the emission SEQUENCE — the
//! order is itself under test, so a set of counters would not be enough.
class Recorder {
public:
    enum class Kind { ATTITUDE, GIMBAL, GNSS, IMAGE, EVENT };

    struct Rec {
        Kind            kind = Kind::EVENT;
        double          t    = 0.0;
        AttitudeData    att;
        GimbalData      gim;
        GnssData        gnss;
        ImageData       img;
        StreamEvent     ev;
    };

    void subscribe(uavloc::sensor::DataSourceInterface& src) {
        src.attitudeChannel().add([this](double t, const AttitudeData& a) {
            Rec r; r.kind = Kind::ATTITUDE; r.t = t; r.att = a; push(r);
        });
        src.gimbalChannel().add([this](double t, const GimbalData& g) {
            Rec r; r.kind = Kind::GIMBAL; r.t = t; r.gim = g; push(r);
        });
        src.gnssChannel().add([this](double t, const GnssData& g) {
            Rec r; r.kind = Kind::GNSS; r.t = t; r.gnss = g; push(r);
        });
        src.imageChannel().add([this](double t, const ImageData& i) {
            Rec r; r.kind = Kind::IMAGE; r.t = t; r.img = i; push(r);
        });
        src.eventChannel().add([this](const StreamEvent& e) {
            Rec r; r.kind = Kind::EVENT; r.ev = e; push(r);
        });
    }

    std::vector<Rec> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return recs_;
    }

    std::size_t total() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return recs_.size();
    }

    std::size_t count(Kind k) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::size_t n = 0;
        for (const auto& r : recs_) {
            if (r.kind == k) ++n;
        }
        return n;
    }

    std::size_t eventCount(StreamEventKind k) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::size_t n = 0;
        for (const auto& r : recs_) {
            if (r.kind == Kind::EVENT && r.ev.kind == k) ++n;
        }
        return n;
    }

private:
    void push(const Rec& r) {
        std::lock_guard<std::mutex> lock(mutex_);
        recs_.push_back(r);
    }

    mutable std::mutex mutex_;
    std::vector<Rec>   recs_;
};

//! Splits a recording into per-sample groups and verifies the publish order.
//! Returns false (and logs the offending index) if any group is malformed.
//!
//! A well-formed recording is a concatenation of groups, each being either
//!   ATTITUDE, GIMBAL, GNSS, IMAGE   (sample with telemetry) or
//!   IMAGE                           (sample without telemetry),
//! with EVENT records allowed anywhere between groups. That single structural
//! statement covers checks 2 and 3 at once: IMAGE closes every group (so it is
//! last), and `same_t` demands one timestamp per group.
bool verify_order_and_timestamps(const std::vector<Recorder::Rec>& recs,
                                 std::size_t* out_samples) {
    std::size_t samples = 0;
    std::size_t i       = 0;
    while (i < recs.size()) {
        if (recs[i].kind == Recorder::Kind::EVENT) {
            ++i;
            continue;
        }
        if (recs[i].kind == Recorder::Kind::IMAGE) {
            ++samples;  // telemetry-less sample: image alone
            ++i;
            continue;
        }
        if (i + 3 >= recs.size() ||
            recs[i + 0].kind != Recorder::Kind::ATTITUDE ||
            recs[i + 1].kind != Recorder::Kind::GIMBAL ||
            recs[i + 2].kind != Recorder::Kind::GNSS ||
            recs[i + 3].kind != Recorder::Kind::IMAGE) {
            spdlog::error("publish order broken at record {}", i);
            return false;
        }
        const double t = recs[i].t;
        if (recs[i + 1].t != t || recs[i + 2].t != t || recs[i + 3].t != t) {
            spdlog::error("timestamps differ inside the sample starting at record {}", i);
            return false;
        }
        ++samples;
        i += 4;
    }
    if (out_samples) *out_samples = samples;
    return true;
}

// ── checks ───────────────────────────────────────────────────────────────────

//! Checks 1-4 on a real dataset. Soft-skips (still PASS) when the gitignored
//! video is absent.
void check_real_dataset(const std::string& config_path) {
    spdlog::info("── checks 1-4: real dataset, {} frames ──", REAL_RUN_FRAMES);

    uavloc::sensor::VideoReaderConfig reader_cfg;
    try {
        const YAML::Node root = YAML::LoadFile(config_path);  // READ-ONLY
        reader_cfg = uavloc::sensor::VideoReaderConfig::fromYaml(root);
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot parse mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return;
    }
    // Cut the replay short so the check runs in seconds AND the source hits a
    // real END_OF_STREAM (check 5a on real data). end_frame is inclusive.
    reader_cfg.end_frame = reader_cfg.start_frame + REAL_RUN_FRAMES - 1;

    {
        uavloc::sensor::VideoReader probe(reader_cfg);
        if (!probe.open()) {
            spdlog::warn("test_data_source: cannot open video '{}' — checks 1-4 SKIPPED "
                         "(dataset absent)", reader_cfg.video_path);
            return;  // soft-skip: still PASS
        }
    }

    auto spy_owner = std::make_unique<SpySource>(
        std::make_unique<uavloc::sensor::VideoReader>(reader_cfg));
    SpySource* spy = spy_owner.get();

    VideoDataSource src(std::move(spy_owner));
    Recorder        rec;
    rec.subscribe(src);

    check(src.startStreaming(), "1a. startStreaming() on the real dataset");
    const bool ended = wait_until([&] { return !src.isStreaming(); },
                                 std::chrono::milliseconds(OP_DEADLINE_MS));
    check(ended, "1b. the stream ends by itself at the end_frame cut");
    run_with_deadline("1c. stopStreaming() after the natural end",
                      [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const std::vector<SpySource::Seen> seen = spy->snapshot();
    const std::vector<Recorder::Rec>   recs = rec.snapshot();

    std::size_t with_telemetry = 0;
    for (const auto& s : seen) {
        if (s.has_telemetry) ++with_telemetry;
    }

    check(seen.size() == static_cast<std::size_t>(REAL_RUN_FRAMES),
          "1d. the reader delivered exactly " + std::to_string(REAL_RUN_FRAMES) +
              " frames (got " + std::to_string(seen.size()) + ")");
    check(rec.count(Recorder::Kind::IMAGE) == seen.size(),
          "1e. image channel fired once per delivered frame (" +
              std::to_string(rec.count(Recorder::Kind::IMAGE)) + ")");
    check(rec.count(Recorder::Kind::ATTITUDE) == with_telemetry &&
              rec.count(Recorder::Kind::GIMBAL) == with_telemetry &&
              rec.count(Recorder::Kind::GNSS) == with_telemetry,
          "1f. the three telemetry channels fired once per frame WITH telemetry (" +
              std::to_string(with_telemetry) + ")");
    check(rec.eventCount(StreamEventKind::END_OF_STREAM) == 1,
          "1g. exactly one END_OF_STREAM event");
    check(src.framesPublished() == seen.size(),
          "1h. framesPublished() matches the image-channel count");

    std::size_t samples = 0;
    check(verify_order_and_timestamps(recs, &samples),
          "2/3. image published LAST and all four channels share one timestamp");
    check(samples == seen.size(),
          "2b. every delivered frame formed exactly one publish group");

    // ── check 4: rebuild TelemetryData out of the three channels ────────────
    bool        reassembly_ok = true;
    bool        climb_zero    = true;
    std::size_t compared      = 0;
    std::size_t i             = 0;
    std::size_t s_idx         = 0;
    while (i < recs.size() && s_idx < seen.size()) {
        if (recs[i].kind == Recorder::Kind::EVENT) {
            ++i;
            continue;
        }
        if (recs[i].kind == Recorder::Kind::IMAGE) {  // no-telemetry sample
            ++i;
            ++s_idx;
            continue;
        }
        const Recorder::Rec& a = recs[i + 0];
        const Recorder::Rec& g = recs[i + 1];
        const Recorder::Rec& n = recs[i + 2];
        const Recorder::Rec& im = recs[i + 3];

        TelemetryData rebuilt;
        rebuilt.frame_id        = im.img.frame_id;
        rebuilt.timestamp_msec  = a.t;
        rebuilt.roll_deg        = a.att.roll_deg;
        rebuilt.pitch_deg       = a.att.pitch_deg;
        rebuilt.heading_deg     = a.att.yaw_deg;
        rebuilt.gimbal_pan_deg  = g.gim.pan_deg;
        rebuilt.gimbal_tilt_deg = g.gim.tilt_deg;
        rebuilt.latitude_deg    = n.gnss.latitude_deg;
        rebuilt.longitude_deg   = n.gnss.longitude_deg;
        rebuilt.altitude_m      = n.gnss.altitude_m;
        rebuilt.speed_mps       = n.gnss.speed_mps;

        const TelemetryData& orig = seen[s_idx].telemetry;
        // Exact equality is the right test: these doubles are copied verbatim,
        // no arithmetic happens on the way.
        const bool same = rebuilt.roll_deg == orig.roll_deg &&
                          rebuilt.pitch_deg == orig.pitch_deg &&
                          rebuilt.heading_deg == orig.heading_deg &&
                          rebuilt.gimbal_pan_deg == orig.gimbal_pan_deg &&
                          rebuilt.gimbal_tilt_deg == orig.gimbal_tilt_deg &&
                          rebuilt.latitude_deg == orig.latitude_deg &&
                          rebuilt.longitude_deg == orig.longitude_deg &&
                          rebuilt.altitude_m == orig.altitude_m &&
                          rebuilt.speed_mps == orig.speed_mps &&
                          rebuilt.frame_id == seen[s_idx].frame_id &&
                          rebuilt.timestamp_msec == seen[s_idx].timestamp_msec;
        if (!same) {
            spdlog::error("reassembly mismatch at sample {} (frame_id {})", s_idx,
                          seen[s_idx].frame_id);
            reassembly_ok = false;
        }
        // climb_mps is the one TelemetryData field the four channels do NOT
        // carry (see stream_types.h). Reassembly is lossless only while no
        // producer sets it — assert that, instead of assuming it.
        if (orig.climb_mps != 0.0) climb_zero = false;

        ++compared;
        i += 4;
        ++s_idx;
    }
    check(reassembly_ok && compared == with_telemetry,
          "4a. TelemetryData rebuilt from the channels equals the original on all " +
              std::to_string(compared) + " samples");
    check(climb_zero,
          "4b. climb_mps (the field the channels do not carry) is 0 in this dataset, "
          "so the reassembly really is lossless");
}

//! completion() on a VideoReader wrapped DIRECTLY (checks 1-4 insert a spy in
//! between, which hides the reader's frame count from the adapter). Soft-skips
//! with the same contract as check 1.
void check_completion(const std::string& config_path) {
    spdlog::info("── check 1i: completion() on a file source ──");

    uavloc::sensor::VideoReaderConfig reader_cfg;
    try {
        const YAML::Node root = YAML::LoadFile(config_path);  // READ-ONLY
        reader_cfg = uavloc::sensor::VideoReaderConfig::fromYaml(root);
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot parse mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return;
    }
    reader_cfg.end_frame = reader_cfg.start_frame + COMPLETION_RUN_FRAMES - 1;

    {
        uavloc::sensor::VideoReader probe(reader_cfg);
        if (!probe.open()) {
            spdlog::warn("test_data_source: cannot open video '{}' — check 1i SKIPPED "
                         "(dataset absent)", reader_cfg.video_path);
            return;  // soft-skip: still PASS
        }
    }

    VideoDataSource src(std::make_unique<uavloc::sensor::VideoReader>(reader_cfg));
    check(std::isnan(src.completion()),
          "1i-1. completion() is NaN before the source is opened");
    check(src.startStreaming(), "1i-2. startStreaming()");
    check(wait_until([&] { return !src.isStreaming(); },
                     std::chrono::milliseconds(OP_DEADLINE_MS)),
          "1i-3. the stream ends at the end_frame cut");
    run_with_deadline("1i-4. stopStreaming()", [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const double done = src.completion();
    check(done > 0.0 && done <= 1.0,
          "1i-5. completion() sits in (0,1] after the cut run (" + std::to_string(done) +
              ")");
}

//! Check 5a: the source reports END_OF_STREAM ⇒ the event fires immediately,
//! after the last image, and the loop stops.
void check_eos_reported() {
    spdlog::info("── check 5a: END_OF_STREAM reported by the source ──");

    FakeSource::Options opt;
    opt.good_frames = 10;
    opt.tail_status = FrameStatus::END_OF_STREAM;

    auto            fake_owner = std::make_unique<FakeSource>(opt);
    FakeSource*     fake       = fake_owner.get();
    VideoDataSource src(std::move(fake_owner));
    Recorder        rec;
    rec.subscribe(src);

    check(src.startStreaming(), "5a-1. startStreaming()");
    check(wait_until([&] { return !src.isStreaming(); },
                     std::chrono::milliseconds(OP_DEADLINE_MS)),
          "5a-2. the loop stops on END_OF_STREAM");
    run_with_deadline("5a-3. stopStreaming()", [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    const std::vector<Recorder::Rec> recs = rec.snapshot();
    check(rec.count(Recorder::Kind::IMAGE) == 10, "5a-4. 10 images published");
    check(rec.eventCount(StreamEventKind::END_OF_STREAM) == 1,
          "5a-5. exactly one END_OF_STREAM event");
    check(!recs.empty() && recs.back().kind == Recorder::Kind::EVENT &&
              recs.back().ev.kind == StreamEventKind::END_OF_STREAM,
          "5a-6. the event is the LAST record");
    check(!recs.empty() && recs.back().ev.frame_id == 9,
          "5a-7. the event carries the id of the last published frame");
    check(fake->reads() == 11, "5a-8. exactly one read past the last frame");
    check(std::isnan(src.completion()),
          "5a-9. completion() is NaN for a source that reports no frame count");
}

//! Check 5b: a cut container that answers EMPTY_FRAME forever (problem P2-1).
//! The guard must fire after exactly max_consecutive_bad_reads reads.
void check_eos_guard() {
    spdlog::info("── check 5b: END_OF_STREAM guard on repeated bad reads ──");

    constexpr int GOOD  = 5;
    constexpr unsigned int LIMIT = 8;

    FakeSource::Options opt;
    opt.good_frames = GOOD;
    opt.tail_status = FrameStatus::EMPTY_FRAME;

    VideoDataSourceConfig cfg;
    cfg.max_consecutive_bad_reads = LIMIT;

    auto            fake_owner = std::make_unique<FakeSource>(opt);
    FakeSource*     fake       = fake_owner.get();
    VideoDataSource src(std::move(fake_owner), cfg);
    Recorder        rec;
    rec.subscribe(src);

    check(src.startStreaming(), "5b-1. startStreaming()");
    check(wait_until([&] { return !src.isStreaming(); },
                     std::chrono::milliseconds(OP_DEADLINE_MS)),
          "5b-2. the guard stops the loop");
    run_with_deadline("5b-3. stopStreaming()", [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    check(rec.count(Recorder::Kind::IMAGE) == GOOD, "5b-4. 5 images published");
    check(rec.eventCount(StreamEventKind::END_OF_STREAM) == 1,
          "5b-5. exactly one END_OF_STREAM event");
    check(rec.eventCount(StreamEventKind::READ_ERROR) == 1,
          "5b-6. exactly one READ_ERROR event (first of the run, not one per read)");
    check(fake->reads() == GOOD + static_cast<int>(LIMIT),
          "5b-7. stopped after exactly max_consecutive_bad_reads bad reads (" +
              std::to_string(fake->reads()) + ")");
}

//! Check 6: stopStreaming() in the middle of an endless stream.
void check_stop_midstream() {
    spdlog::info("── check 6: stopStreaming() mid-stream ──");

    FakeSource::Options opt;
    opt.endless       = true;
    opt.read_delay_ms = 2;

    VideoDataSource src(std::make_unique<FakeSource>(opt));
    Recorder        rec;
    rec.subscribe(src);

    check(src.startStreaming(), "6-1. startStreaming()");
    check(wait_until([&] { return rec.count(Recorder::Kind::IMAGE) >= 5; },
                     std::chrono::milliseconds(POLL_DEADLINE_MS)),
          "6-2. the stream is running (>= 5 images)");

    run_with_deadline("6-3. stopStreaming() mid-stream",
                      [&] { check(src.stopStreaming(), "6-3b. stopStreaming() returns true"); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));
    check(!src.isStreaming(), "6-4. isStreaming() is false after stop");

    const std::size_t after_stop = rec.count(Recorder::Kind::IMAGE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    check(rec.count(Recorder::Kind::IMAGE) == after_stop,
          "6-5. nothing is published after stopStreaming() returned");

    run_with_deadline("6-6. second stopStreaming() is safe",
                      [&] { check(src.stopStreaming(), "6-6b. second call returns true"); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));
    check(rec.count(Recorder::Kind::IMAGE) == after_stop,
          "6-7. the second stop published nothing either");
}

//! Check 7: clearCallbacks() silences every channel while the stream runs on.
void check_clear_callbacks() {
    spdlog::info("── check 7: clearCallbacks() ──");

    FakeSource::Options opt;
    opt.endless       = true;
    opt.read_delay_ms = 2;

    VideoDataSource src(std::make_unique<FakeSource>(opt));
    Recorder        rec;
    rec.subscribe(src);

    check(src.attitudeChannel().size() == 1 && src.gimbalChannel().size() == 1 &&
              src.gnssChannel().size() == 1 && src.imageChannel().size() == 1 &&
              src.eventChannel().size() == 1,
          "7-1. all five channels hold exactly one subscriber");

    check(src.startStreaming(), "7-2. startStreaming()");
    check(wait_until([&] { return rec.count(Recorder::Kind::IMAGE) >= 5; },
                     std::chrono::milliseconds(POLL_DEADLINE_MS)),
          "7-3. the stream is running (>= 5 images)");

    src.clearCallbacks();
    check(src.attitudeChannel().empty() && src.gimbalChannel().empty() &&
              src.gnssChannel().empty() && src.imageChannel().empty() &&
              src.eventChannel().empty(),
          "7-4. every channel reports empty after clearCallbacks()");

    // One emission may already be in flight when clear() lands (CallbackSlot
    // contract, note 3), so sample AFTER a settling pause and require that
    // nothing moves from there on.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const std::size_t settled = rec.total();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    check(rec.total() == settled,
          "7-5. no channel fires any more once clearCallbacks() has settled");

    run_with_deadline("7-6. stopStreaming()", [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));
}

//! Check 8: no telemetry ⇒ image channel only.
void check_no_telemetry() {
    spdlog::info("── check 8: frames without telemetry ──");

    FakeSource::Options opt;
    opt.good_frames    = 6;
    opt.with_telemetry = false;
    opt.tail_status    = FrameStatus::END_OF_STREAM;

    VideoDataSource src(std::make_unique<FakeSource>(opt));
    Recorder        rec;
    rec.subscribe(src);

    check(src.startStreaming(), "8-1. startStreaming()");
    check(wait_until([&] { return !src.isStreaming(); },
                     std::chrono::milliseconds(OP_DEADLINE_MS)),
          "8-2. the loop stops on END_OF_STREAM");
    run_with_deadline("8-3. stopStreaming()", [&] { src.stopStreaming(); },
                      std::chrono::milliseconds(OP_DEADLINE_MS));

    check(rec.count(Recorder::Kind::IMAGE) == 6, "8-4. 6 images published");
    check(rec.count(Recorder::Kind::ATTITUDE) == 0 &&
              rec.count(Recorder::Kind::GIMBAL) == 0 &&
              rec.count(Recorder::Kind::GNSS) == 0,
          "8-5. the three telemetry channels stay silent — no invented data");

    std::size_t samples = 0;
    check(verify_order_and_timestamps(rec.snapshot(), &samples) && samples == 6,
          "8-6. the recording is still well formed (six image-only samples)");
}

}  // namespace

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    const std::string config_path =
        (argc > 1) ? argv[1] : std::string(UAVLOC_MISSION_CONFIG_PATH);

    if (config_path.empty()) {
        spdlog::warn("test_data_source: no mission config path — checks 1-4 SKIPPED");
    } else {
        check_real_dataset(config_path);
        check_completion(config_path);
    }

    check_eos_reported();
    check_eos_guard();
    check_stop_midstream();
    check_clear_callbacks();
    check_no_telemetry();

    spdlog::info("==================================");
    spdlog::info("test_data_source: {}", g_ok ? "PASS" : "FAIL");
    return g_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
