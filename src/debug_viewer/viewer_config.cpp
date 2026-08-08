// viewer_config.cpp — DebugViewer::Config::fromYaml.
//
// Kept out of debug_viewer.cpp on purpose: that TU is the render loop and is
// already very large, while this is pure configuration parsing with no GL, no
// threads and no state. Every field follows the project pattern
// node["key"].as<T>(default), so a missing key (or a missing whole section)
// silently keeps the struct default and nothing throws.
//
// Only the keys the overlay driver actually consumed are read here — this is a
// verbatim move of the block that used to live in tests/test_vo_viewer.cpp, and
// adding keys would change the behaviour of a config that already sets them.

#include "uavloc/debug_viewer/debug_viewer.h"

#include <algorithm>

namespace uavloc::debug_viewer {
namespace {

// Validity floors, not tunables: an alignment window of 0 frames would freeze
// the transform on no data, and a 0 ms sampler period would spin a thread on
// /proc. They clamp a bad config instead of letting it reach the code.
constexpr int MIN_ALIGNMENT_WINDOW_FRAMES = 1;
constexpr int MIN_PERF_SAMPLE_MS          = 50;

} // namespace

DebugViewer::Config DebugViewer::Config::fromYaml(const YAML::Node& node) {
    Config cfg;
    if (!node) {
        return cfg;
    }

    // ── Alignment (TrajectoryAligner) ────────────────────────────────────────
    cfg.alignment_window_frames =
        std::max(MIN_ALIGNMENT_WINDOW_FRAMES,
                 node["alignment_window_frames"].as<int>(cfg.alignment_window_frames));
    cfg.alignment_min_spread_m =
        node["alignment_min_spread_m"].as<double>(cfg.alignment_min_spread_m);

    // ── World scaling ────────────────────────────────────────────────────────
    cfg.display_scale  = node["display_scale"].as<float>(cfg.display_scale);
    cfg.vertical_scale = node["vertical_scale"].as<float>(cfg.vertical_scale);

    // ── Side panels: capacities ──────────────────────────────────────────────
    cfg.video_stride        = node["video_stride"].as<int>(cfg.video_stride);
    cfg.log_capacity        = node["log_capacity"].as<int>(cfg.log_capacity);
    cfg.metric_plot_history = node["metric_plot_history"].as<int>(cfg.metric_plot_history);
    cfg.metric_plot_follow_window =
        node["metric_plot_follow_window"].as<int>(cfg.metric_plot_follow_window);
    cfg.histogram_bins    = node["histogram_bins"].as<int>(cfg.histogram_bins);
    cfg.perf_plot_history = node["perf_plot_history"].as<int>(cfg.perf_plot_history);

    // ── Side panels: initial visibility ──────────────────────────────────────
    cfg.show_log     = node["show_log"].as<bool>(cfg.show_log);
    cfg.show_inliers = node["show_inliers"].as<bool>(cfg.show_inliers);
    // NOT cfg.show_landmarks: the overlay driver deliberately defaults the map
    // cloud ON (it is the thing the alignment is fitted against), unlike the
    // bare-viewer default. Preserved verbatim from the driver.
    cfg.show_landmarks  = node["show_landmarks"].as<bool>(true);
    cfg.show_video      = node["show_video"].as<bool>(cfg.show_video);
    cfg.show_tracking   = node["show_tracking"].as<bool>(cfg.show_tracking);
    cfg.show_lost_poses = node["show_lost_poses"].as<bool>(cfg.show_lost_poses);
    cfg.show_hud        = node["show_hud"].as<bool>(cfg.show_hud);
    cfg.show_perf       = node["show_perf"].as<bool>(cfg.show_perf);
    // "Profiling" table panel; the table stays empty unless UAVLOC_PROFILE=1.
    cfg.show_profile = node["show_profile"].as<bool>(cfg.show_profile);
    // "Error (m)" panel: predicted-vs-groundtruth error line chart + histogram
    // + running summary.
    cfg.show_error       = node["show_error"].as<bool>(cfg.show_error);
    cfg.show_estimate    = node["show_estimate"].as<bool>(cfg.show_estimate);
    cfg.show_groundtruth = node["show_groundtruth"].as<bool>(cfg.show_groundtruth);

    // ── Trajectory rendering style ───────────────────────────────────────────
    cfg.est_odom          = node["est_odom"].as<bool>(cfg.est_odom);
    cfg.est_odom_every_n  = node["est_odom_every_n"].as<int>(cfg.est_odom_every_n);
    cfg.est_odom_axes_scale =
        node["est_odom_axes_scale"].as<float>(cfg.est_odom_axes_scale);
    cfg.show_fused = node["show_fused"].as<bool>(cfg.show_fused);

    // ── Driver-side overlay parameters ───────────────────────────────────────
    cfg.hud_min_baseline_m =
        node["hud_min_baseline_m"].as<double>(cfg.hud_min_baseline_m);
    cfg.perf_sample_ms =
        std::max(MIN_PERF_SAMPLE_MS, node["perf_sample_ms"].as<int>(cfg.perf_sample_ms));

    return cfg;
}

} // namespace uavloc::debug_viewer
