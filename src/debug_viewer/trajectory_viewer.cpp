#include <uavloc/debug_viewer/trajectory_viewer.h>

#include "gps_to_enu.h"

#include <guik/viewer/light_viewer.hpp>
#include <glk/thin_lines.hpp>
#include <glk/primitives/primitives.hpp>
#include <imgui.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace uavloc::debug_viewer {

// ---------------------------------------------------------------------------
// Pimpl
// ---------------------------------------------------------------------------
struct TrajectoryViewer::Impl {
    Config                       cfg;
    std::vector<TelemetryRecord> records;
    std::vector<Eigen::Vector3f> enu_points;
    double                       origin_lat  = 0.0;
    double                       origin_lon  = 0.0;
    double                       origin_alt  = 0.0;

    // Stats computed in setRecords
    float mean_e = 0.0f;
    float mean_n = 0.0f;
    float mean_u = 0.0f;
    float min_e  = 0.0f, max_e = 0.0f;
    float min_n  = 0.0f, max_n = 0.0f;

    // Names of all "uav_*" coord-frame drawables currently registered
    std::vector<std::string> coord_names;

    // Build or rebuild the viewer with current cfg
    void ensureViewer() {
        if (!guik::LightViewer::running()) {
            guik::LightViewer::instance(
                Eigen::Vector2i(cfg.window_w, cfg.window_h),
                /*background=*/false,
                cfg.window_title);
        }
    }
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
TrajectoryViewer::TrajectoryViewer()
    : impl_(std::make_unique<Impl>())
{}

TrajectoryViewer::TrajectoryViewer(const Config& cfg)
    : impl_(std::make_unique<Impl>())
{
    impl_->cfg = cfg;
}

TrajectoryViewer::~TrajectoryViewer() {
    guik::LightViewer::destroy();
}

// ---------------------------------------------------------------------------
// setRecords — converts GPS to ENU, populates all drawables and UI callbacks
// ---------------------------------------------------------------------------
void TrajectoryViewer::setRecords(const std::vector<TelemetryRecord>& records)
{
    impl_->records = records;
    impl_->enu_points.clear();

    if (records.empty()) return;

    // ---- Ensure the GL window is open before removing old drawables ----
    impl_->ensureViewer();
    auto* viewer = guik::LightViewer::instance();

    // Remove previously registered coord-frame drawables
    for (const auto& name : impl_->coord_names) {
        viewer->remove_drawable(name);
    }
    impl_->coord_names.clear();

    // Use first valid record as ENU origin
    impl_->origin_lat = records.front().latitude;
    impl_->origin_lon = records.front().longitude;
    impl_->origin_alt = records.front().altitude_m;

    impl_->enu_points.reserve(records.size());
    for (const auto& r : records) {
        auto p = gps_to_enu(r.latitude, r.longitude, r.altitude_m,
                            impl_->origin_lat, impl_->origin_lon, impl_->origin_alt);
        impl_->enu_points.emplace_back(p.e, p.n, p.u);
    }

    // Compute bounding box and mean for camera centering
    impl_->min_e = impl_->max_e = impl_->enu_points[0].x();
    impl_->min_n = impl_->max_n = impl_->enu_points[0].y();
    float sum_e = 0.0f, sum_n = 0.0f, sum_u = 0.0f;
    for (const auto& pt : impl_->enu_points) {
        impl_->min_e = std::min(impl_->min_e, pt.x());
        impl_->max_e = std::max(impl_->max_e, pt.x());
        impl_->min_n = std::min(impl_->min_n, pt.y());
        impl_->max_n = std::max(impl_->max_n, pt.y());
        sum_e += pt.x();
        sum_n += pt.y();
        sum_u += pt.z();
    }
    const float n_pts  = static_cast<float>(impl_->enu_points.size());
    impl_->mean_e = sum_e / n_pts;
    impl_->mean_n = sum_n / n_pts;
    impl_->mean_u = sum_u / n_pts;

    // ---- Trajectory polyline ----
    viewer->update_drawable(
        "gps_trajectory",
        std::make_shared<glk::ThinLines>(impl_->enu_points, /*line_strip=*/true),
        guik::FlatColor(0.2f, 0.9f, 0.4f, 1.0f));

    // ---- UAV orientation frames every N records (0 = disable) ----
    const int step = impl_->cfg.coord_frame_every_n;
    if (step > 0) {
        for (int i = 0; i < static_cast<int>(records.size()); i += step) {
            const auto& r  = records[i];
            const auto& pt = impl_->enu_points[i];

            // ZYX intrinsic Euler: first yaw around Z, then pitch around Y, then roll around X
            const float y  = static_cast<float>(r.yaw_deg   * DEG2RAD);
            const float p  = static_cast<float>(r.pitch_deg * DEG2RAD);
            const float ro = static_cast<float>(r.roll_deg  * DEG2RAD);

            Eigen::Affine3f T = Eigen::Translation3f(pt)
                * Eigen::AngleAxisf(y,  Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(p,  Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(ro, Eigen::Vector3f::UnitX())
                * Eigen::UniformScaling<float>(impl_->cfg.coord_frame_scale);

            const std::string name = "uav_" + std::to_string(r.frame_id);
            viewer->update_drawable(
                name,
                glk::Primitives::coordinate_system(),
                guik::VertexColor(T));
            impl_->coord_names.push_back(name);
        }
    }

    // ---- Center view on trajectory ----
    viewer->lookat(Eigen::Vector3f(impl_->mean_e, impl_->mean_n, impl_->mean_u));

    // ---- ImGui stats panel ----
    // Capture what we need by value for the lambda
    const int     total_frames  = static_cast<int>(records.size());
    const int     first_id      = records.front().frame_id;
    const int     last_id       = records.back().frame_id;
    const float   e_extent      = impl_->max_e - impl_->min_e;
    const float   n_extent      = impl_->max_n - impl_->min_n;
    const double  orig_lat      = impl_->origin_lat;
    const double  orig_lon      = impl_->origin_lon;

    viewer->register_ui_callback("trajectory_stats", [=]() {
        ImGui::SetNextWindowSize(ImVec2(320, 160), ImGuiCond_FirstUseEver);
        ImGui::Begin("Trajectory Stats");
        ImGui::Text("Records loaded : %d", total_frames);
        ImGui::Text("Frame range    : %d – %d", first_id, last_id);
        ImGui::Text("E extent (m)   : %.1f", static_cast<double>(e_extent));
        ImGui::Text("N extent (m)   : %.1f", static_cast<double>(n_extent));
        ImGui::Text("Origin lat     : %.6f", orig_lat);
        ImGui::Text("Origin lon     : %.6f", orig_lon);
        ImGui::End();
    });
}

// ---------------------------------------------------------------------------
// spin / spinOnce
// ---------------------------------------------------------------------------
void TrajectoryViewer::spin()
{
    impl_->ensureViewer();
    guik::LightViewer::instance()->spin();
}

bool TrajectoryViewer::spinOnce()
{
    impl_->ensureViewer();
    return guik::LightViewer::instance()->spin_once();
}

} // namespace uavloc::debug_viewer
