// ts_klv_to_csv — one-shot converter: MPEG-TS MISB ST 0601 KLV → telemetry CSV.
//
// Extracts every KLV Local Set from an MPEG-TS file (no video decode) and
// writes a per-video-frame telemetry CSV consumable by the sensor module's
// DroneTelemetryCsvReader (columns mapped by name in the mission YAML).
//
// Usage:   ts_klv_to_csv <in.ts> [out.csv]
//          default out.csv = alongside the input, "<stem>_telemetry.csv"
//          (e.g. UTC-DAY18.ts → UTC-DAY18_telemetry.csv)
//
// CSV columns:
//   frame_id,timestamp_msec,heading,pitch,roll,latitude,longitude,
//   altitude_agl,gimbal_pan,gimbal_tilt
//
// Conventions / assumptions (verified against UTC-DAY18.ts):
//   * frame_id = llround((ts_us - ts0_us) * fps / 1e6) with fps = 30.0 — the
//     HoaLac video stream is H.264 1920x1080 @ 30 fps; KLV records arrive at a
//     lower rate, so frame_ids are sparse. Duplicate frame_ids are dropped
//     keeping the FIRST record.
//   * altitude_agl = tag15 (Sensor True Altitude, MSL) - tag25 (Frame Center
//     Elevation, MSL). When tag 25 is absent the terrain elevation is unknown
//     and we fall back to tag15 - 0 (i.e. treat MSL altitude as AGL).
//   * gimbal_pan  = tag18 Sensor Relative Azimuth (deg, clockwise from
//     platform nose).
//   * gimbal_tilt = -(tag19 Sensor Relative Elevation). MISB rel-elevation is
//     negative below the horizon (≈ -80..-90 when looking near-nadir), while
//     TelemetryData::gimbal_tilt_deg is measured DOWN from horizontal with
//     ~+90 = nadir — hence the sign flip. The tool logs the median rel-el so
//     the sign convention can be verified against the actual data.
//
// At the end the tool prints suggested pinhole intrinsics derived from the
// MEDIAN HFOV/VFOV tags:  fx = (W/2)/tan(HFOV/2), fy = (H/2)/tan(VFOV/2),
// cx = W/2, cy = H/2  for the 1920x1080 stream.
//
// Requires libav (built only when the LIBAV pkg-config block finds it).

#include "klv_misb0601.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Video frame rate of the HoaLac .ts stream (H.264 1920x1080 @ 30 fps).
constexpr double VIDEO_FPS = 30.0;

// Nominal image size used only for the suggested-intrinsics printout.
constexpr double IMAGE_W = 1920.0;
constexpr double IMAGE_H = 1080.0;

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

}  // namespace

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    if (argc < 2) {
        spdlog::error("usage: ts_klv_to_csv <in.ts> [out.csv]");
        return 1;
    }
    const std::string ts_path = argv[1];
    std::string csv_path;
    if (argc > 2) {
        csv_path = argv[2];
    } else {
        std::filesystem::path p(ts_path);
        p.replace_filename(p.stem().string() + "_telemetry.csv");
        csv_path = p.string();
    }

    if (!std::ifstream(ts_path).good()) {
        spdlog::error("input '{}' not found", ts_path);
        return 1;
    }

    std::vector<uavloc::tests::KlvRecord> klv;
    if (!uavloc::tests::readKlvStream(ts_path, klv)) {
        spdlog::error("KLV parse failed for '{}'", ts_path);
        return 1;
    }
    if (klv.empty()) {
        spdlog::error("no KLV records found in '{}'", ts_path);
        return 1;
    }
    spdlog::info("parsed {} KLV records, span {:.1f} s", klv.size(),
                 (klv.back().ts_usec - klv.front().ts_usec) / 1e6);

    // Sign-convention check for gimbal_tilt (see header comment).
    {
        std::vector<double> rel_els;
        for (const auto& r : klv)
            if (r.has_rel_el) rel_els.push_back(r.rel_el_deg);
        if (!rel_els.empty()) {
            const double med = median(rel_els);
            spdlog::info("tag19 rel-elevation median = {:.2f} deg "
                         "(expected ~-80..-90 when near-nadir) -> "
                         "gimbal_tilt = -rel_el (median tilt {:.2f} deg)",
                         med, -med);
            if (med > 0.0)
                spdlog::warn("median rel-elevation is POSITIVE — the "
                             "gimbal_tilt = -rel_el sign convention may be "
                             "wrong for this data; verify before use");
        } else {
            spdlog::warn("no tag19 (rel elevation) records — gimbal_tilt "
                         "column will be 0");
        }
    }

    std::ofstream out(csv_path);
    if (!out) {
        spdlog::error("cannot open '{}' for writing", csv_path);
        return 1;
    }
    out << "frame_id,timestamp_msec,heading,pitch,roll,latitude,longitude,"
           "altitude_agl,gimbal_pan,gimbal_tilt\n";

    const uint64_t ts0_us = klv.front().ts_usec;
    int64_t last_frame_id = -1;
    size_t  rows = 0, dup_dropped = 0, incomplete = 0, no_fc_elev = 0;
    std::vector<double> hfovs, vfovs;

    for (const auto& r : klv) {
        if (r.has_hfov) hfovs.push_back(r.hfov_deg);
        if (r.has_vfov) vfovs.push_back(r.vfov_deg);

        // A usable telemetry row needs position + altitude + attitude.
        if (!(r.has_lat && r.has_lon && r.has_alt &&
              r.has_heading && r.has_pitch && r.has_roll)) {
            ++incomplete;
            continue;
        }

        // frame_id from the KLV timestamp assuming the video is 30 fps and
        // frame 0 coincides with the first KLV record (same TS timeline).
        const int64_t frame_id =
            llround((r.ts_usec - ts0_us) * VIDEO_FPS / 1e6);
        if (frame_id == last_frame_id) {   // duplicate frame_id — keep first
            ++dup_dropped;
            continue;
        }
        last_frame_id = frame_id;

        const double timestamp_msec = (r.ts_usec - ts0_us) / 1000.0;

        // AGL = sensor true altitude (MSL) - frame center elevation (MSL).
        // Fallback: terrain elevation 0 when tag 25 is absent.
        double alt_agl = r.alt_m;
        if (r.has_fc_elev) alt_agl -= r.fc_elev_m;
        else ++no_fc_elev;

        const double gimbal_pan  = r.has_rel_az ? r.rel_az_deg : 0.0;
        const double gimbal_tilt = r.has_rel_el ? -r.rel_el_deg : 0.0;

        char line[512];
        std::snprintf(line, sizeof(line),
                      "%lld,%.3f,%.4f,%.4f,%.4f,%.8f,%.8f,%.2f,%.4f,%.4f\n",
                      static_cast<long long>(frame_id), timestamp_msec,
                      r.heading_deg, r.pitch_deg, r.roll_deg,
                      r.lat_deg, r.lon_deg, alt_agl,
                      gimbal_pan, gimbal_tilt);
        out << line;
        ++rows;
    }
    out.close();

    spdlog::info("wrote {} rows to '{}' ({} duplicate frame_ids dropped, "
                 "{} incomplete records skipped, {} rows without tag25 "
                 "frame-center elevation)",
                 rows, csv_path, dup_dropped, incomplete, no_fc_elev);

    // Suggested pinhole intrinsics from the median FOV tags.
    if (!hfovs.empty() && !vfovs.empty()) {
        const double hfov = median(hfovs);
        const double vfov = median(vfovs);
        const double deg2rad = M_PI / 180.0;
        const double fx = (IMAGE_W / 2.0) / std::tan(hfov * deg2rad / 2.0);
        const double fy = (IMAGE_H / 2.0) / std::tan(vfov * deg2rad / 2.0);
        spdlog::info("median HFOV = {:.4f} deg, VFOV = {:.4f} deg "
                     "({} / {} samples)", hfov, vfov, hfovs.size(), vfovs.size());
        spdlog::info("suggested intrinsics for {:.0f}x{:.0f}: "
                     "fx = {:.3f}, fy = {:.3f}, cx = {:.1f}, cy = {:.1f}",
                     IMAGE_W, IMAGE_H, fx, fy, IMAGE_W / 2.0, IMAGE_H / 2.0);
    } else {
        spdlog::warn("no HFOV/VFOV (tags 16/17) records — cannot suggest "
                     "intrinsics");
    }
    return rows > 0 ? 0 : 1;
}
