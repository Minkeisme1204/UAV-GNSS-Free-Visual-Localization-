#include "driver_common.h"

#include <spdlog/spdlog.h>

#include <cstdlib>
#include <iomanip>
#include <string>
#include <utility>

namespace uavloc {
namespace eval {

namespace {

//! True when the variable exists and is not the empty string — the same
//! condition env_int/env_double use to decide whether they override.
bool env_present(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr && *v != '\0';
}

//! True iff the environment variable is set AND non-empty. `DISPLAY=` yields a
//! non-null but empty string, which DebugViewer::run() already treats as
//! headless; viewer_run_policy() has to agree with it exactly.
bool env_nonempty(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr && v[0] != '\0';
}

//! Where a value ended up coming from, for the provenance log line below.
const char* provenance(const YAML::Node& node, const char* yaml_key,
                       const char* env_name) {
    if (env_present(env_name))               return "env";
    if (node && node.IsMap() && node[yaml_key]) return "yaml";
    return "default";
}

} // namespace

int env_int(const char* name, int fallback) {
    const char* v = std::getenv(name);
    return (v != nullptr && *v != '\0') ? std::atoi(v) : fallback;
}

double env_double(const char* name, double fallback) {
    const char* v = std::getenv(name);
    return (v != nullptr && *v != '\0') ? std::atof(v) : fallback;
}

void force_offline_run_mode(core::SystemConfig& cfg) {
    cfg.vo.async_enabled                    = true;
    cfg.vo.wait_for_local_bundle_adjustment = true;
    cfg.fusion.async_enabled                = false;
    cfg.async_input                         = false;
    spdlog::info("driver: run mode FORCED — vo.async_enabled=true, "
                 "vo.wait_for_local_bundle_adjustment=true, "
                 "fusion.async_enabled=false, async_input=false "
                 "(mission YAML overridden on purpose)");
}

anchor::FakeAnchorConfig fake_anchor_config(
    const YAML::Node& root, const sensor::VideoReaderConfig& reader_cfg) {
    // ── layer 1: struct defaults, plus the groundtruth the pipeline is fed ──
    anchor::FakeAnchorConfig c;
    c.telemetry       = reader_cfg.drone_telemetry;
    c.frame_id_offset = static_cast<long long>(reader_cfg.telemetry_frame_id_offset);

    // ── layer 2: the mission YAML, when it carries a FakeAnchor: node ───────
    const YAML::Node node = (root && root.IsMap()) ? root["FakeAnchor"]
                                                   : YAML::Node();
    if (node && node.IsMap()) {
        anchor::FakeAnchorConfig y = anchor::FakeAnchorConfig::fromYaml(node);
        // fromYaml() reads the groundtruth from its OWN nested node; when the
        // node does not name one, keep the VideoReader-derived source rather
        // than the empty default — otherwise merely adding "FakeAnchor:" to a
        // config would silently disarm the producer.
        if (!node["DroneTelemetry"])  y.telemetry       = c.telemetry;
        if (!node["frame_id_offset"]) y.frame_id_offset = c.frame_id_offset;
        c = y;
        spdlog::info("driver: FakeAnchor: node found in the mission config — "
                     "its keys override the defaults");
    }

    // ── layer 3: the environment, which always wins ─────────────────────────
    c.sigma_m         = env_double("UAVLOC_FIX_SIGMA_M", c.sigma_m);
    c.every_kf        = env_int("UAVLOC_FIX_EVERY_KF", c.every_kf);
    c.reinit_min_kf_gap =
        env_int("UAVLOC_FIX_REINIT_MIN_KF", c.reinit_min_kf_gap);
    c.outlier_rate    = env_double("UAVLOC_FIX_OUTLIER_RATE", c.outlier_rate);
    c.outlier_min_m   = env_double("UAVLOC_FIX_OUTLIER_MIN_M", c.outlier_min_m);
    c.outlier_max_m   = env_double("UAVLOC_FIX_OUTLIER_MAX_M", c.outlier_max_m);
    c.latency_kf      = env_int("UAVLOC_FIX_LATENCY_KF", c.latency_kf);
    c.seed = static_cast<unsigned int>(
        env_int("UAVLOC_FIX_SEED", static_cast<int>(c.seed)));

    if (c.mode != anchor::FakeAnchorMode::SYNCHRONOUS) {
        spdlog::warn("driver: FakeAnchor.mode asked for 'async' — IGNORED. Both "
                     "evaluation drivers are regression gates and need the "
                     "producer to run inline");
    }
    c.mode = anchor::FakeAnchorMode::SYNCHRONOUS;

    // Provenance, not decoration: a matrix of cadence/sigma cells is worthless
    // if the cell labels are assumed instead of read back from the run's log.
    spdlog::info("driver: fake anchor resolved — every_kf={} [{}], "
                 "sigma_m={} [{}], seed={} [{}]",
                 c.every_kf,  provenance(node, "every_kf", "UAVLOC_FIX_EVERY_KF"),
                 c.sigma_m,   provenance(node, "sigma_m",  "UAVLOC_FIX_SIGMA_M"),
                 c.seed,      provenance(node, "seed",     "UAVLOC_FIX_SEED"));
    return c;
}

MissionSetup load_viewer_mission(const std::string& config_path) {
    MissionSetup m;
    try {
        m.yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        spdlog::error("driver: failed to load YAML '{}': {}", config_path,
                      e.what());
        return m;
    }
    try {
        m.reader = sensor::VideoReaderConfig::fromYaml(m.yaml);
        m.system = core::SystemConfig::fromYaml(m.yaml);  // VO+Fusion+Camera+System
    } catch (const std::exception& e) {
        spdlog::error("driver: failed to parse config '{}': {}", config_path,
                      e.what());
        return m;
    }
    // The ONE configuration key a viewer driver legitimately differs on: it
    // only decides whether the image rides along on the debug channel.
    m.system.publish_images = true;
    force_offline_run_mode(m.system);
    m.loaded = true;
    return m;
}

std::unique_ptr<sensor::VideoDataSource> open_viewer_source(
    const MissionSetup& mission, const char* driver_name) {
    auto reader = std::make_unique<sensor::VideoReader>(mission.reader);
    if (!reader->open()) {
        spdlog::warn("{}: cannot open video '{}' — SKIPPED", driver_name,
                     mission.reader.video_path);
        return nullptr;  // soft-skip when the dataset is absent
    }
    spdlog::info("{}: video opened — fps: {:.1f}, total frames: {}", driver_name,
                 reader->getFps(), reader->getFrameCount());
    return std::make_unique<sensor::VideoDataSource>(
        std::move(reader),
        sensor::VideoDataSourceConfig::fromYaml(mission.yaml["VideoDataSource"]));
}

RunPolicy viewer_run_policy() {
    RunPolicy p;
    if (!env_nonempty("DISPLAY") && !env_nonempty("WAYLAND_DISPLAY")) {
        // Regression run: it must start by itself and it must be bounded.
        p.autostart  = true;
        p.max_frames = static_cast<std::size_t>(PARITY_MAX_FRAMES);
        p.reason     = "headless (no DISPLAY/WAYLAND_DISPLAY) — a REGRESSION "
                       "run: nobody could press the Start button, and the "
                       "frame cap keeps the gate bounded and comparable";
    } else {
        // Interactive session: the operator starts it and the operator ends it.
        p.autostart  = false;
        p.max_frames = INTERACTIVE_MAX_FRAMES;
        p.reason     = "a DISPLAY is present — an INTERACTIVE session: the run "
                       "waits for the viewer's Start button and is NOT capped";
    }
    spdlog::info("driver: run policy — autostart={}, max_frames={} ({})",
                 p.autostart, p.max_frames,
                 p.max_frames == 0 ? "no cap, runs to end of stream"
                                   : "capped regression run");
    return p;
}

std::ofstream open_pose_dump(const std::string& path, const char* driver_name) {
    std::ofstream dump(path);
    if (!dump.is_open()) {
        spdlog::error("{}: cannot open pose dump '{}'", driver_name, path);
        return dump;
    }
    dump << std::setprecision(DUMP_PRECISION);
    dump << "frame_id,pred_x,pred_y,pred_z\n";
    spdlog::info("{}: the fused poses of this run are written to '{}'",
                 driver_name, path);
    return dump;
}

} // namespace eval
} // namespace uavloc
