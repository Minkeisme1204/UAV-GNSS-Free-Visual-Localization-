// test_driver_parity — cross-driver regression gate (S9).
//
// tests/test_full_flight.cpp and tests/test_vo_viewer.cpp drive the SAME
// core::SystemManager over the same data. Until S9 they forced different run
// modes (the viewer forced nothing about VO, and forced the synchronous
// input/fusion path only inside its UAVLOC_VIEWER_DUMP branch), so their pose
// sequences agreeing was an observation, not a property. driver_common.cpp now
// forces one mode for both; this test is what keeps them from drifting apart
// again.
//
// What it does:
//   1. runs test_full_flight  → a 30+-column CSV with pred_x/pred_y/pred_z
//   2. runs test_vo_viewer    → UAVLOC_VIEWER_DUMP CSV (frame_id,pred_*)
//   both on the SAME config, the SAME frame cap and the same (empty) flags;
//   3. joins the two on frame_id and compares.
//
// ⚠ The comparison is NOT absolute. With the viewer compiled in, test_vo_viewer
// dumps the pose AFTER its display anchoring g0 + (p - f0), i.e. the whole line
// is shifted by a constant vector (g0 - f0) that test_full_flight does not
// apply; with ENABLE_VIEWER=OFF it dumps the raw fused ENU position and the
// shift is zero. The property under test is therefore: the per-frame difference
// is CONSTANT. The test measures the spread of that difference (max - min per
// axis) and fails when it exceeds MAX_SPREAD_M. A test comparing the raw values
// would fail on a correct system; one comparing only the mean would pass on a
// broken one. The mean is REPORTED, never asserted — its value is a legitimate
// property of the build configuration and of the data.
//
// The viewer is spawned with DISPLAY and WAYLAND_DISPLAY CLEARED: with a
// display it opens a live window and blocks until the user closes it.
//
// ⚠ The frame cap and the viewer's dump path are NOT passed to test_vo_viewer:
// that driver takes no environment variable at all any more. Both sides read
// the SAME named constants from tests/driver_common.h (eval::PARITY_MAX_FRAMES,
// eval::VIEWER_DUMP_PATH), which is what keeps the two runs on the same data.
// test_full_flight is still steered from here (it is not part of that change).
//
// Soft-skip: when the mission config's (gitignored) video is absent both
// drivers exit 0 without producing data, and so does this test — but only
// because the video was checked for FIRST. An empty CSV with the video present
// is a failure, not a skip.
//
// Usage:  ./tests/test_driver_parity [config.yaml]

#include "uavloc/sensor/video_reader.h"

#include "driver_common.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifndef UAVLOC_PARITY_CONFIG_PATH
#define UAVLOC_PARITY_CONFIG_PATH ""
#endif
#ifndef UAVLOC_FULL_FLIGHT_EXE
#define UAVLOC_FULL_FLIGHT_EXE ""
#endif
#ifndef UAVLOC_VO_VIEWER_EXE
#define UAVLOC_VO_VIEWER_EXE ""
#endif

namespace {

//! Maximum spread (max - min over the common frames, per axis) of the constant
//! offset between the two pose sequences, in metres. The two drivers run the
//! same double-precision code, so a matching run has a spread at the CSV
//! round-trip level (~1e-13 m measured on ds3/200 frames); this bound is six
//! orders above that and still far below anything a real behavioural difference
//! could hide under.
constexpr double MAX_SPREAD_M = 1e-6;

//! Refuse to pass on a near-empty intersection: a gate that compares three
//! frames is not a gate.
constexpr std::size_t MIN_COMMON_FRAMES = 50;

struct Pose {
    double x = 0.0, y = 0.0, z = 0.0;
};

//! Split one CSV line on commas (no quoting in either file).
std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::string              cell;
    std::istringstream       is(line);
    while (std::getline(is, cell, ',')) {
        out.push_back(cell);
    }
    return out;
}

//! Read `path` as frame_id → (pred_x, pred_y, pred_z), locating the columns BY
//! HEADER NAME. Both files carry those four headers but in different positions
//! and among a different number of columns, and test_full_flight's layout is
//! explicitly allowed to grow at the end — a fixed column index would turn any
//! such append into a false failure here.
bool load_poses(const std::string& path, std::map<long long, Pose>& out) {
    std::ifstream in(path);
    if (!in.is_open()) {
        spdlog::error("test_driver_parity: cannot open '{}'", path);
        return false;
    }
    std::string header;
    if (!std::getline(in, header)) {
        spdlog::error("test_driver_parity: '{}' is empty", path);
        return false;
    }
    const std::vector<std::string> cols = split_csv(header);
    auto index_of = [&cols](const char* name) -> long {
        for (std::size_t i = 0; i < cols.size(); ++i) {
            if (cols[i] == name) return static_cast<long>(i);
        }
        return -1;
    };
    const long i_id = index_of("frame_id");
    const long i_x  = index_of("pred_x");
    const long i_y  = index_of("pred_y");
    const long i_z  = index_of("pred_z");
    if (i_id < 0 || i_x < 0 || i_y < 0 || i_z < 0) {
        spdlog::error("test_driver_parity: '{}' lacks one of "
                      "frame_id/pred_x/pred_y/pred_z", path);
        return false;
    }
    const std::size_t need =
        static_cast<std::size_t>(std::max(std::max(i_id, i_x), std::max(i_y, i_z)));
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> c = split_csv(line);
        if (c.size() <= need) continue;  // truncated row — ignore, never guess
        Pose p;
        try {
            const long long id = std::stoll(c[static_cast<std::size_t>(i_id)]);
            p.x = std::stod(c[static_cast<std::size_t>(i_x)]);
            p.y = std::stod(c[static_cast<std::size_t>(i_y)]);
            p.z = std::stod(c[static_cast<std::size_t>(i_z)]);
            out[id] = p;
        } catch (const std::exception&) {
            // A non-finite pose is written as "nan"/"inf" by both drivers; such
            // a row carries no comparable number, so it is dropped rather than
            // compared. std::stod parses "nan", so this only catches garbage.
            continue;
        }
    }
    return true;
}

//! true when `path` exists and can be opened for reading.
bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

int run(const std::string& cmd) {
    spdlog::info("test_driver_parity: $ {}", cmd);
    const int rc = std::system(cmd.c_str());
    spdlog::info("test_driver_parity:   → exit {}", rc);
    return rc;
}

} // namespace

int main(int argc, char** argv) {
    const std::string config_path =
        (argc > 1) ? argv[1] : std::string(UAVLOC_PARITY_CONFIG_PATH);
    // NOT overridable: test_vo_viewer's cap is compiled in, so a cap chosen
    // here could only make the two drivers read different amounts of data.
    const int max_frames = uavloc::eval::PARITY_MAX_FRAMES;
    const std::string ff_exe(UAVLOC_FULL_FLIGHT_EXE);
    const std::string vw_exe(UAVLOC_VO_VIEWER_EXE);

    if (config_path.empty() || ff_exe.empty() || vw_exe.empty()) {
        spdlog::error("test_driver_parity: built without the config/executable "
                      "paths — check tests/CMakeLists.txt");
        return 1;
    }
    if (!file_exists(ff_exe) || !file_exists(vw_exe)) {
        spdlog::error("test_driver_parity: driver executable missing "
                      "('{}' / '{}')", ff_exe, vw_exe);
        return 1;
    }

    // Soft-skip check FIRST, so an empty CSV later can only mean a real
    // failure. Uses the same parser the drivers use, not a guess at the key.
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("test_driver_parity: cannot load '{}': {}",
                      config_path, e.what());
        return 1;
    }
    std::string video_path;
    try {
        video_path = uavloc::sensor::VideoReaderConfig::fromYaml(yaml).video_path;
    } catch (const std::exception& e) {
        spdlog::error("test_driver_parity: cannot parse '{}': {}",
                      config_path, e.what());
        return 1;
    }
    if (!file_exists(video_path)) {
        spdlog::warn("test_driver_parity: dataset video '{}' is absent — SKIPPED",
                     video_path);
        return 0;
    }

    const std::string ff_csv  = "driver_parity_full_flight.csv";
    // Written by test_vo_viewer itself, at its own compiled-in path.
    const std::string vw_csv  = uavloc::eval::VIEWER_DUMP_PATH;
    const std::string ff_log  = "driver_parity_full_flight.log";
    const std::string vw_log  = "driver_parity_viewer.log";
    const std::string frames  = std::to_string(max_frames);

    // Both drivers read the SAME number of frames: test_full_flight is told so
    // on its command line, the viewer has the same constant compiled in. The
    // viewer additionally gets DISPLAY/WAYLAND_DISPLAY cleared — with a display
    // it blocks on a live window until the user closes it.
    const std::string ff_cmd =
        "UAVLOC_VO_MAXFRAMES=" + frames + " " +
        ff_exe + " " + config_path + " " + ff_csv + " > " + ff_log + " 2>&1";
    const std::string vw_cmd =
        "env -u DISPLAY -u WAYLAND_DISPLAY " +
        vw_exe + " " + config_path + " > " + vw_log + " 2>&1";

    if (run(ff_cmd) != 0) {
        spdlog::error("test_driver_parity: test_full_flight failed — see '{}'",
                      ff_log);
        return 1;
    }
    if (run(vw_cmd) != 0) {
        spdlog::error("test_driver_parity: test_vo_viewer failed — see '{}'",
                      vw_log);
        return 1;
    }

    std::map<long long, Pose> ff_poses, vw_poses;
    if (!load_poses(ff_csv, ff_poses) || !load_poses(vw_csv, vw_poses)) {
        return 1;
    }
    spdlog::info("test_driver_parity: full_flight rows = {}, viewer rows = {}",
                 ff_poses.size(), vw_poses.size());

    // ── the comparison: is the per-frame difference constant? ────────────────
    std::size_t n = 0;
    double min_d[3] = {0, 0, 0};
    double max_d[3] = {0, 0, 0};
    double sum_d[3] = {0, 0, 0};
    long long worst_frame = -1;
    for (const auto& [id, vp] : vw_poses) {
        const auto it = ff_poses.find(id);
        if (it == ff_poses.end()) continue;
        const Pose& fp = it->second;
        const double d[3] = {vp.x - fp.x, vp.y - fp.y, vp.z - fp.z};
        for (int a = 0; a < 3; ++a) {
            if (n == 0) {
                min_d[a] = max_d[a] = d[a];
            } else {
                if (d[a] < min_d[a]) { min_d[a] = d[a]; worst_frame = id; }
                if (d[a] > max_d[a]) { max_d[a] = d[a]; worst_frame = id; }
            }
            sum_d[a] += d[a];
        }
        ++n;
    }

    if (n < MIN_COMMON_FRAMES) {
        spdlog::error("test_driver_parity FAIL: only {} common frames (need at "
                      "least {}) — the two drivers did not evaluate the same "
                      "data, so nothing was actually compared",
                      n, MIN_COMMON_FRAMES);
        return 1;
    }

    double worst_spread = 0.0;
    for (int a = 0; a < 3; ++a) {
        worst_spread = std::max(worst_spread, max_d[a] - min_d[a]);
    }
    const char* axis = "xyz";
    spdlog::info("==== test_driver_parity ====");
    spdlog::info("  config          = '{}'", config_path);
    spdlog::info("  max_frames      = {}", max_frames);
    spdlog::info("  common frames   = {}", n);
    for (int a = 0; a < 3; ++a) {
        spdlog::info("  d{} : mean = {:+.6f} m, spread = {:.3e} m",
                     axis[a], sum_d[a] / static_cast<double>(n),
                     max_d[a] - min_d[a]);
    }
    spdlog::info("  worst spread    = {:.3e} m   (limit {:.3e} m)",
                 worst_spread, MAX_SPREAD_M);
    spdlog::info("  csv             = '{}' / '{}'", ff_csv, vw_csv);
    spdlog::info("============================");

    if (worst_spread > MAX_SPREAD_M) {
        spdlog::error("test_driver_parity FAIL: the difference between the two "
                      "drivers is NOT a constant offset (spread {:.6f} m > {:.3e} "
                      "m, extremum near frame {}). The two drivers are running "
                      "different experiments — do NOT relax this threshold, find "
                      "which configuration key diverged (logs: '{}', '{}')",
                      worst_spread, MAX_SPREAD_M, worst_frame, ff_log, vw_log);
        return 1;
    }
    // The mean offset is REPORTED, never asserted: it is the viewer's display
    // anchor g0 - f0, a legitimate difference whose value depends on the data.
    spdlog::info("test_driver_parity: PASS — the two drivers differ by a "
                 "constant display offset only");
    return 0;
}
