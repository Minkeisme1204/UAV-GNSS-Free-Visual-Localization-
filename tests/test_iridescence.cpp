#include <guik/viewer/light_viewer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/thin_lines.hpp>
#include <imgui.h>
#include <Eigen/Core>
#include <cmath>

int main() {
    auto viewer = guik::LightViewer::instance();
    viewer->set_draw_xy_grid(true);

    // World-origin coordinate axes
    viewer->update_coord("origin", guik::VertexColor().scale(1.5f));

    // A small grid of colored points
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(400);
    for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
            float x = static_cast<float>(i);
            float y = static_cast<float>(j);
            float z = 0.3f * std::sin(0.6f * x) * std::cos(0.6f * y);
            pts.push_back({x, y, z});
        }
    }
    viewer->update_points("wave", pts, guik::Rainbow().set_point_scale(4.0f));

    // A simple polyline triangle
    std::vector<Eigen::Vector3f> tri = {
        {3.0f, 0.0f, 0.0f},
        {0.0f, 3.0f, 0.0f},
        {0.0f, 0.0f, 3.0f},
        {3.0f, 0.0f, 0.0f},
    };
    viewer->update_thin_lines("triangle", tri, /*line_strip=*/true,
                              guik::FlatColor(1.0f, 0.8f, 0.0f, 1.0f));

    // ImGui info panel
    viewer->register_ui_callback("info", [] {
        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_Once);
        ImGui::Begin("iridescence test");
        ImGui::Text("wave: 21x21 sin*cos point cloud");
        ImGui::Text("yellow triangle: 3-axis line strip");
        ImGui::Text("RGB axes: world origin");
        ImGui::Separator();
        ImGui::Text("Close window to exit.");
        ImGui::End();
    });

    viewer->spin();
    return 0;
}
