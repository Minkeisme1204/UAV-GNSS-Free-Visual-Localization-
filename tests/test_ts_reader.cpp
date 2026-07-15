// test_ts_reader — MPEG-TS smoke test: H.264 video + MISB ST 0601 KLV telemetry.
//
// Phase 1 (UAVLOC_WITH_LIBAV): demux the .ts with libavformat WITHOUT decoding
//   video; read only the KLV data stream (stream 0x100, AV_CODEC_ID_SMPTE_KLV)
//   and parse each packet as a MISB ST 0601 UAS Datalink Local Set.
// Phase 2: stream the H.264 video through sensor::VideoReader (proves the .ts
//   container works in the uavloc sensor pipeline) and correlate every logged
//   frame with the nearest-in-time KLV record.
//
// Usage:   test_ts_reader [ts_path]         (default data/HoaLac/UTC-DAY18.ts)
// Env:     UAVLOC_TS_SHOW=0     force headless (log-only). Default: when a
//                               display is present (DISPLAY/WAYLAND_DISPLAY),
//                               play the video LIVE in a cv::imshow window
//                               ("TS stream") paced to the source fps, with a
//                               per-frame KLV telemetry overlay; ESC/'q' quits.
//          UAVLOC_TS_MAXFRAMES  frame cap for Phase 2. Default: 0 (play to
//                               EOF/ESC) when displaying, 300 when headless.
//
// Soft-skips (exit 0) when the .ts file is absent so CI without the dataset
// still passes.

#include "uavloc/sensor/video_reader.h"

// Shared MISB ST 0601 KLV parser (KlvRecord, describe, readKlvStream, …) —
// factored out into tests/klv_misb0601.h so ts_klv_to_csv reuses it.
#include "klv_misb0601.h"

#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace {

using uavloc::tests::KlvRecord;
using uavloc::tests::describe;

// Nearest-in-time KLV record for a video timestamp, using RELATIVE offsets.
// Assumption: the first decoded video frame coincides (approximately) with the
// first KLV record — good enough for a smoke test; a production sync would use
// the shared MPEG-TS PTS timeline instead.
const KlvRecord* nearestKlv(const std::vector<KlvRecord>& recs,
                            double video_rel_msec) {
    if (recs.empty()) return nullptr;
    const uint64_t target = recs.front().ts_usec +
                            static_cast<uint64_t>(std::max(0.0, video_rel_msec) * 1000.0);
    auto it = std::lower_bound(recs.begin(), recs.end(), target,
                               [](const KlvRecord& r, uint64_t t) { return r.ts_usec < t; });
    if (it == recs.end()) return &recs.back();
    if (it != recs.begin()) {
        auto prev = std::prev(it);
        if (target - prev->ts_usec < it->ts_usec - target) return &*prev;
    }
    return &*it;
}

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    return std::atoi(v);
}

// Draw one overlay line twice (black shadow, then white) for readability.
void drawOverlayLine(cv::Mat& img, const std::string& text, int line_idx) {
    const double font_scale = 0.7;
    const int    baseline_y = 30 + line_idx * 28;
    cv::putText(img, text, {12, baseline_y + 2}, cv::FONT_HERSHEY_SIMPLEX,
                font_scale, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(img, text, {10, baseline_y}, cv::FONT_HERSHEY_SIMPLEX,
                font_scale, {255, 255, 255}, 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    const std::string ts_path =
        (argc > 1) ? argv[1] : "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/data/HoaLac/UTC-DAY18.ts";

    // Soft-skip when the dataset is absent (CI machines without data/).
    if (!std::ifstream(ts_path).good()) {
        spdlog::warn("test_ts_reader: '{}' not found — skipping (exit 0)", ts_path);
        return 0;
    }

    // ── Phase 1: KLV telemetry ────────────────────────────────────────────────
    std::vector<KlvRecord> klv;
#ifdef UAVLOC_WITH_LIBAV
    spdlog::info("=== Phase 1: KLV (MISB ST 0601) parse of '{}' ===", ts_path);
    if (!uavloc::tests::readKlvStream(ts_path, klv)) {
        spdlog::error("KLV pass failed");
        return 1;
    }
    spdlog::info("[KLV] parsed {} records", klv.size());
    if (!klv.empty()) {
        const double span_s =
            (klv.back().ts_usec - klv.front().ts_usec) / 1e6;
        spdlog::info("[KLV] time span: {:.1f} s", span_s);
        spdlog::info("[KLV] first : {}", describe(klv.front()));
        spdlog::info("[KLV] last  : {}", describe(klv.back()));
        for (size_t frac = 1; frac <= 3; ++frac) {   // a few evenly spaced samples
            const size_t idx = klv.size() * frac / 4;
            if (idx < klv.size())
                spdlog::info("[KLV] sample[{}]: {}", idx, describe(klv[idx]));
        }
    }
#else
    spdlog::warn("KLV unavailable — built without libav (UAVLOC_WITH_LIBAV off); "
                 "running video-only");
#endif

    // ── Phase 2: video via sensor::VideoReader ───────────────────────────────
    spdlog::info("=== Phase 2: video stream via sensor::VideoReader ===");
    uavloc::sensor::VideoReaderConfig cfg;
    cfg.video_path = ts_path;

    uavloc::sensor::VideoReader reader(cfg);
    if (!reader.open()) {
        spdlog::error("VideoReader failed to open '{}' — .ts is NOT usable in the "
                      "sensor pipeline as-is (reporting, not patching the lib)",
                      ts_path);
        return 1;
    }
    spdlog::info("VideoReader open: fps={:.2f} frame_count={}",
                 reader.getFps(), reader.getFrameCount());

    // Display is the DEFAULT when a display server is present; only an explicit
    // UAVLOC_TS_SHOW=0 (or no display) falls back to the headless log-only path.
    const char* display_env = std::getenv("DISPLAY");
    const char* wayland_env = std::getenv("WAYLAND_DISPLAY");
    const bool has_display  = (display_env && *display_env) ||
                              (wayland_env && *wayland_env);
    const char* show_env    = std::getenv("UAVLOC_TS_SHOW");
    const bool  show        = has_display &&
                              !(show_env && std::string(show_env) == "0");
    if (!show)
        spdlog::info("running headless (no display or UAVLOC_TS_SHOW=0) — log-only");

    // Displaying: play to EOF/ESC by default (cap 0 = unlimited).
    // Headless: keep the bounded 300-frame smoke default.
    const int max_frames = envInt("UAVLOC_TS_MAXFRAMES", show ? 0 : 300);

    // Pace playback to the source fps when displaying.
    const double src_fps = reader.getFps();
    const int wait_ms = (src_fps > 0.0)
                            ? std::max(1, static_cast<int>(1000.0 / src_fps))
                            : 1;

    int    frames = 0;
    double first_ts_msec = -1.0;
    const auto t0 = std::chrono::steady_clock::now();

    uavloc::sensor::FrameData frame;
    while (max_frames <= 0 || frames < max_frames) {
        const auto status = reader.read(frame);
        if (status == uavloc::sensor::FrameStatus::END_OF_STREAM) break;
        if (status != uavloc::sensor::FrameStatus::OK) {
            spdlog::warn("frame {}: status {}", frames, static_cast<int>(status));
            continue;
        }
        ++frames;
        if (first_ts_msec < 0.0) first_ts_msec = frame.timestamp_msec;

        if (frames % 50 == 0 || frames == 1) {
            const double elapsed_s = std::chrono::duration<double>(
                                         std::chrono::steady_clock::now() - t0).count();
            const double fps = (elapsed_s > 0.0) ? frames / elapsed_s : 0.0;
            const KlvRecord* k = nearestKlv(klv, frame.timestamp_msec - first_ts_msec);
            if (k) {
                spdlog::info("frame {} id={} {}x{} decode_fps={:.1f} | KLV "
                             "lat={:.7f} lon={:.7f} alt={:.1f} m heading={:.2f}",
                             frames, frame.frame_id, frame.image.cols,
                             frame.image.rows, fps,
                             k->lat_deg, k->lon_deg, k->alt_m, k->heading_deg);
            } else {
                spdlog::info("frame {} id={} {}x{} decode_fps={:.1f} | no KLV",
                             frames, frame.frame_id, frame.image.cols,
                             frame.image.rows, fps);
            }
        }

        if (show) {
            cv::Mat display = frame.image.clone();

            char line1[128];
            std::snprintf(line1, sizeof(line1), "frame %ld  t=%.2fs  %dx%d",
                          static_cast<long>(frame.frame_id),
                          (frame.timestamp_msec - first_ts_msec) / 1000.0,
                          display.cols, display.rows);
            drawOverlayLine(display, line1, 0);

            const KlvRecord* k = nearestKlv(klv, frame.timestamp_msec - first_ts_msec);
            char line2[192];
            if (k) {
                std::snprintf(line2, sizeof(line2),
                              "lat=%.7f lon=%.7f alt=%.1fm hdg=%.2f deg",
                              k->lat_deg, k->lon_deg, k->alt_m, k->heading_deg);
            } else {
                std::snprintf(line2, sizeof(line2), "KLV: n/a");
            }
            drawOverlayLine(display, line2, 1);

            cv::imshow("TS stream", display);
            const int key = cv::waitKey(wait_ms) & 0xFF;
            if (key == 27 || key == 'q') break;   // ESC / 'q' quits early
        }
    }
    if (show) cv::destroyAllWindows();
    reader.close();

    const double total_s = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0).count();
    const double avg_fps = (total_s > 0.0) ? frames / total_s : 0.0;
    spdlog::info("=== Summary ===");
    spdlog::info("video : {} frames decoded, avg decode fps {:.1f}", frames, avg_fps);
#ifdef UAVLOC_WITH_LIBAV
    if (!klv.empty()) {
        spdlog::info("klv   : {} records, span {:.1f} s", klv.size(),
                     (klv.back().ts_usec - klv.front().ts_usec) / 1e6);
    } else {
        spdlog::info("klv   : 0 records");
    }
#else
    spdlog::info("klv   : unavailable (built without libav)");
#endif
    return 0;
}
