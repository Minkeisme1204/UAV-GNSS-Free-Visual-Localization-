#include <uavloc/debug_viewer/debug_viewer.h>
#include <uavloc/debug_viewer/telemetry_csv_reader.h>

#include <spdlog/spdlog.h>

#include <filesystem>
#include <string>

// Live groundtruth-trajectory debug viewer.
//
// Usage:
//   demo_debug_viewer [directory | video_path]
//
// - With a directory (default: data/YenBai800m), the viewer scans for a .csv and
//   a video (.mkv/.mp4/.avi). It streams the video, decodes the per-frame
//   CODE-128 barcode (which encodes the CSV imageId), looks up the full
//   telemetry from the CSV, and grows the 3D ENU groundtruth trajectory live
//   while showing the current frame in a sub-window.
// - The program runs cleanly headless (no DISPLAY): it still decodes barcodes
//   and fires callbacks, then exits 0.

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    const std::string DEFAULT_DIR = "/home/minkeisrtx5090/Desktop/Workplace/HUST/uav_localization/data/YenBai800m";
    std::string arg = (argc > 1) ? argv[1] : DEFAULT_DIR;

    uavloc::debug_viewer::DebugViewer::Config cfg;
    cfg.window_title        = "UAV Debug Viewer — YenBai 800 m";
    cfg.coord_frame_every_n = 200;
    // The YenBai flight spans ~8 km; shrink it so the whole path fits on screen.
    // Smaller display_scale => smaller trajectory. (0.01 => 1 unit == 100 m.)
    cfg.display_scale       = 0.01f;
    // Read through the video faster by processing 1 of every N frames.
    cfg.frame_stride        = 5;

    uavloc::debug_viewer::DebugViewer viewer(cfg);

    namespace fs = std::filesystem;
    if (fs::is_directory(arg)) {
        viewer.loadDirectory(arg);
    } else if (fs::is_regular_file(arg)) {
        // Treat as a video path; use its parent directory so a sibling CSV is
        // picked up for telemetry lookup.
        viewer.loadDirectory(fs::path(arg).parent_path().string());
    } else {
        spdlog::error("demo_debug_viewer: path does not exist: {}", arg);
        return 1;
    }

    viewer.run();

    spdlog::info("demo_debug_viewer: exited cleanly");
    return 0;
}
