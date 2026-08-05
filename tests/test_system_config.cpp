// test_system_config — S2 unit test for core::SystemConfig::fromYaml().
//
// Six checks:
//   1. no "System:" block          ⇒ every field on its documented default;
//   2. fully populated "System:"   ⇒ every field read back exactly;
//   3. input_policy string parsing ⇒ block / drop_oldest (case-insensitive),
//      an unknown string keeps the default;
//   4. input_queue_capacity: 0     ⇒ replaced by DEFAULT_INPUT_QUEUE_CAPACITY
//      (0 means UNBOUNDED in util::ThreadsafeQueue and would silently kill
//      backpressure);
//   5. delegation                  ⇒ on a REAL mission config, SystemConfig's
//      .vo / .fusion match vo::VOConfig::fromYaml() / fusion::FusionConfig::
//      fromYaml() called directly, proving S2 changed no existing parsing;
//   6. completely empty YAML       ⇒ THROWS, and the message names "Camera".
//
// Check 6 asserts a throw on purpose. Camera intrinsics are not an optional
// knob: defaulting them would push a degenerate K (fx = fy = 0) into the
// pipeline, so a mission config without a "Camera:" section must fail at load
// time. Optional System: keys stay tolerant — that is what checks 1-4 cover.
// Consequently every in-memory YAML below carries a minimal Camera block.
//
// Every YAML in checks 1-4 is built in-memory with YAML::Load(); check 5 only
// READS a config file. Nothing under config/ is created or modified.
//
// Headless; exits non-zero on failure.

#include "uavloc/core/system_config.h"
#include "uavloc/core/system_types.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/new_vo/vo_config.h"

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>

namespace {

using uavloc::core::InputPolicy;
using uavloc::core::SystemConfig;

//! Path of the mission config used by the delegation check. Provided by CMake
//! (UAVLOC_MISSION_CONFIG_PATH) so the test carries no hard-coded path; argv[1]
//! overrides it.
#ifndef UAVLOC_MISSION_CONFIG_PATH
#define UAVLOC_MISSION_CONFIG_PATH ""
#endif

//! Tolerance for the double comparisons of the delegation check. Zero on
//! purpose: both sides run the SAME parser on the SAME file, so anything but
//! an exact match means SystemConfig re-parsed something instead of
//! delegating. The named constant keeps that intent explicit.
constexpr double EXACT_TOL = 0.0;

bool g_ok = true;

bool check(bool cond, const char* name) {
    if (cond) {
        spdlog::info("PASS: {}", name);
    } else {
        spdlog::error("FAIL: {}", name);
        g_ok = false;
    }
    return cond;
}

bool same(double a, double b) { return std::abs(a - b) <= EXACT_TOL; }

//! Minimal "Camera:" section prepended to every in-memory document: without it
//! fromYaml() throws by design (check 6), which would mask what checks 1-4
//! actually test.
const char* const CAMERA_BLOCK =
    "Camera:\n"
    "  fx: 1000.0\n"
    "  fy: 1000.0\n"
    "  cx: 640.0\n"
    "  cy: 360.0\n";

constexpr double CAMERA_FX = 1000.0;
constexpr double CAMERA_CY = 360.0;

//! Parse an in-memory mission config made of CAMERA_BLOCK + `body`.
SystemConfig config_from(const std::string& body) {
    return SystemConfig::fromYaml(YAML::Load(std::string(CAMERA_BLOCK) + body));
}

// ── 1. Defaults ───────────────────────────────────────────────────────────
//! The documented defaults of §4.7.
bool has_defaults(const SystemConfig& cfg) {
    return cfg.async_input          == false
        && cfg.input_policy         == InputPolicy::BLOCK
        && cfg.input_queue_capacity == SystemConfig::DEFAULT_INPUT_QUEUE_CAPACITY
        && cfg.push_timeout_ms      == 100u
        && cfg.enable_fusion        == true
        && cfg.enable_geo           == true
        && cfg.publish_images       == false
        && cfg.stats_period_ms      == 500u;
}

void test_defaults_without_system_block() {
    // A realistic mission config that simply has no "System:" key.
    const SystemConfig cfg = config_from(
        "VO:\n"
        "  scale_levels: 8\n"
        "Fusion:\n"
        "  lag_seconds: 40.0\n");
    check(has_defaults(cfg), "no System: block -> all System defaults");
    // Delegation still happened even though "System:" is missing.
    check(cfg.camera.fx == CAMERA_FX && cfg.camera.cy == CAMERA_CY,
          "no System: block -> Camera still parsed through VOConfig");
}

// ── 6. A mission config without Camera: must fail at load time ────────────
void test_missing_camera_throws() {
    bool        threw = false;
    std::string message;
    try {
        (void)SystemConfig::fromYaml(YAML::Load(""));
    } catch (const std::exception& ex) {
        threw   = true;
        message = ex.what();
    }
    if (!check(threw, "empty YAML document throws instead of defaulting the camera")) {
        return;
    }
    spdlog::info("       message: '{}'", message);
    check(message.find("Camera") != std::string::npos,
          "the exception message names the missing 'Camera' section");

    // Same for a document that has other sections but still no Camera:
    // the throw must come from the missing intrinsics, not from an empty doc.
    bool threw_partial = false;
    try {
        (void)SystemConfig::fromYaml(YAML::Load("System:\n  async_input: true\n"));
    } catch (const std::exception&) {
        threw_partial = true;
    }
    check(threw_partial, "a System: block without Camera: still throws");
}

// ── 2. Fully populated block ──────────────────────────────────────────────
void test_all_fields_read_back() {
    const SystemConfig cfg = config_from(
        "System:\n"
        "  async_input: true\n"
        "  input_policy: drop_oldest\n"
        "  input_queue_capacity: 32\n"
        "  push_timeout_ms: 250\n"
        "  enable_fusion: false\n"
        "  enable_geo: false\n"
        "  publish_images: true\n"
        "  stats_period_ms: 1000\n");
    check(cfg.async_input          == true,                     "async_input read back");
    check(cfg.input_policy         == InputPolicy::DROP_OLDEST, "input_policy read back");
    check(cfg.input_queue_capacity == 32u,                      "input_queue_capacity read back");
    check(cfg.push_timeout_ms      == 250u,                     "push_timeout_ms read back");
    check(cfg.enable_fusion        == false,                    "enable_fusion read back");
    check(cfg.enable_geo           == false,                    "enable_geo read back");
    check(cfg.publish_images       == true,                     "publish_images read back");
    check(cfg.stats_period_ms      == 1000u,                    "stats_period_ms read back");
}

// ── 3. input_policy parsing ───────────────────────────────────────────────
SystemConfig config_with_policy(const std::string& value) {
    return config_from("System:\n  input_policy: " + value + "\n");
}

void test_input_policy_parsing() {
    check(config_with_policy("drop_oldest").input_policy == InputPolicy::DROP_OLDEST,
          "input_policy 'drop_oldest'");
    // NOTE: the case-insensitivity evidence lives in the DROP_OLDEST variants
    // ONLY. A 'Block' assertion would prove nothing, because an unrecognised
    // string also yields InputPolicy::BLOCK (the default) — pass and failure
    // are indistinguishable there. Any future case-handling change must be
    // re-tested on a NON-default value.
    check(config_with_policy("DROP_OLDEST").input_policy == InputPolicy::DROP_OLDEST,
          "input_policy 'DROP_OLDEST' (case-insensitive)");
    check(config_with_policy("DrOp_OlDeSt").input_policy == InputPolicy::DROP_OLDEST,
          "input_policy 'DrOp_OlDeSt' (mixed case)");
    check(config_with_policy("block").input_policy == InputPolicy::BLOCK,
          "input_policy 'block' (weak: BLOCK is also the fallback)");
    // Unknown string: keep the default, warn, never throw.
    check(config_with_policy("dropoldest").input_policy == InputPolicy::BLOCK,
          "unknown input_policy keeps the default");
    check(config_with_policy("42").input_policy == InputPolicy::BLOCK,
          "non-string-looking input_policy keeps the default");
}

// ── 4. Unbounded capacity is refused ──────────────────────────────────────
void test_capacity_fallback() {
    const SystemConfig cfg = config_from("System:\n  input_queue_capacity: 0\n");
    check(cfg.input_queue_capacity == SystemConfig::DEFAULT_INPUT_QUEUE_CAPACITY,
          "input_queue_capacity 0 falls back to DEFAULT_INPUT_QUEUE_CAPACITY");
    check(cfg.input_queue_capacity != SystemConfig::UNBOUNDED_INPUT_QUEUE_CAPACITY,
          "input_queue_capacity never leaves fromYaml as UNBOUNDED");

    // A negative value is not a valid unsigned scalar: yaml-cpp falls back to
    // the default rather than wrapping around to a huge capacity.
    const SystemConfig neg = config_from("System:\n  input_queue_capacity: -4\n");
    check(neg.input_queue_capacity == SystemConfig::DEFAULT_INPUT_QUEUE_CAPACITY,
          "negative input_queue_capacity falls back to the default");
}

// ── 5. Delegation to VOConfig / FusionConfig ──────────────────────────────
bool vo_configs_match(const uavloc::vo::VOConfig& a, const uavloc::vo::VOConfig& b) {
    return a.orb_name                       == b.orb_name
        && a.orb_scale_factor               == b.orb_scale_factor
        && a.orb_num_levels                 == b.orb_num_levels
        && a.orb_ini_fast_thr               == b.orb_ini_fast_thr
        && a.orb_min_fast_thr               == b.orb_min_fast_thr
        && a.num_grid_cols                  == b.num_grid_cols
        && a.num_grid_rows                  == b.num_grid_rows
        && a.frame_tracker_num_matches_thr  == b.frame_tracker_num_matches_thr
        && a.frame_tracker_margin           == b.frame_tracker_margin
        && a.use_fixed_seed                 == b.use_fixed_seed
        && a.max_num_local_keyfrms          == b.max_num_local_keyfrms
        && a.min_inliers_to_track           == b.min_inliers_to_track
        && a.local_map_projection_margin    == b.local_map_projection_margin
        && a.projection_lowe_ratio          == b.projection_lowe_ratio
        && a.min_num_shared_lms             == b.min_num_shared_lms
        && a.triangulation_parallax_deg_thr == b.triangulation_parallax_deg_thr
        && a.triangulation_residual_deg_thr == b.triangulation_residual_deg_thr
        && same(a.baseline_dist_thr_ratio, b.baseline_dist_thr_ratio)
        && a.temporal_mapping_enabled       == b.temporal_mapping_enabled
        && a.num_temporal_keyframes         == b.num_temporal_keyframes
        && a.erase_temporal_keyframes       == b.erase_temporal_keyframes
        && a.async_enabled                  == b.async_enabled
        && a.mapping_queue_threshold        == b.mapping_queue_threshold
        && a.wait_for_local_bundle_adjustment == b.wait_for_local_bundle_adjustment
        && a.enable_metric_scale            == b.enable_metric_scale
        && a.weld_on_reinit                 == b.weld_on_reinit
        && same(a.camera_intrinsics.fx, b.camera_intrinsics.fx)
        && same(a.camera_intrinsics.fy, b.camera_intrinsics.fy)
        && same(a.camera_intrinsics.cx, b.camera_intrinsics.cx)
        && same(a.camera_intrinsics.cy, b.camera_intrinsics.cy)
        && a.camera_intrinsics.width  == b.camera_intrinsics.width
        && a.camera_intrinsics.height == b.camera_intrinsics.height
        && a.camera_intrinsics.dist_coeffs == b.camera_intrinsics.dist_coeffs;
}

bool fusion_configs_match(const uavloc::fusion::FusionConfig& a,
                          const uavloc::fusion::FusionConfig& b) {
    return a.async_enabled == b.async_enabled
        && same(a.lag_seconds,            b.lag_seconds)
        && same(a.vo_rot_sigma_deg,       b.vo_rot_sigma_deg)
        && same(a.vo_trans_sigma_m,       b.vo_trans_sigma_m)
        && same(a.delta_yaw_sigma_rad,    b.delta_yaw_sigma_rad)
        && same(a.delta_yaw_max_step_deg, b.delta_yaw_max_step_deg)
        && same(a.rollpitch_sigma_rad,    b.rollpitch_sigma_rad)
        && same(a.agl_sigma_m,            b.agl_sigma_m)
        && same(a.agl_bias_walk_m,        b.agl_bias_walk_m)
        && same(a.agl_bias_prior_sigma_m, b.agl_bias_prior_sigma_m)
        && same(a.scale_walk_sigma,       b.scale_walk_sigma)
        && same(a.scale_prior_sigma,      b.scale_prior_sigma)
        && same(a.anchor_yaw_sigma_deg,   b.anchor_yaw_sigma_deg)
        && same(a.anchor_xy_sigma_m,      b.anchor_xy_sigma_m)
        && same(a.reinit_trans_inflation, b.reinit_trans_inflation)
        && a.map_depth_enabled == b.map_depth_enabled
        && same(a.map_depth_sigma,     b.map_depth_sigma)
        && a.map_depth_min_lms == b.map_depth_min_lms
        && same(a.map_depth_min_agl_m, b.map_depth_min_agl_m)
        && same(a.fix_gate_chi2,   b.fix_gate_chi2)
        && a.fix_robust_kernel == b.fix_robust_kernel
        && same(a.fix_tukey_c,     b.fix_tukey_c)
        && same(a.fix_match_tolerance_sec, b.fix_match_tolerance_sec)
        && same(a.fix_age_margin_sec,      b.fix_age_margin_sec)
        && same(a.fix_drift_rate_m_per_m,  b.fix_drift_rate_m_per_m)
        && same(a.huber_k,         b.huber_k)
        && same(a.health_converged_sigma_m, b.health_converged_sigma_m)
        && same(a.health_drifting_sigma_m,  b.health_drifting_sigma_m);
}

void test_delegation(const std::string& config_path) {
    if (config_path.empty()) {
        spdlog::error("FAIL: delegation check has no mission config path");
        g_ok = false;
        return;
    }
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);  // READ-ONLY
    } catch (const std::exception& ex) {
        spdlog::error("FAIL: cannot read mission config '{}': {}", config_path, ex.what());
        g_ok = false;
        return;
    }
    spdlog::info("delegation check against '{}'", config_path);

    const SystemConfig sys = SystemConfig::fromYaml(root);
    check(vo_configs_match(sys.vo, uavloc::vo::VOConfig::fromYaml(root)),
          "SystemConfig.vo == VOConfig::fromYaml(root)");
    check(fusion_configs_match(sys.fusion, uavloc::fusion::FusionConfig::fromYaml(root)),
          "SystemConfig.fusion == FusionConfig::fromYaml(root)");
    // The stand-alone camera copy must be the very intrinsics VO uses.
    check(same(sys.camera.fx, sys.vo.camera_intrinsics.fx) &&
          same(sys.camera.fy, sys.vo.camera_intrinsics.fy) &&
          same(sys.camera.cx, sys.vo.camera_intrinsics.cx) &&
          same(sys.camera.cy, sys.vo.camera_intrinsics.cy) &&
          sys.camera.width  == sys.vo.camera_intrinsics.width &&
          sys.camera.height == sys.vo.camera_intrinsics.height,
          "SystemConfig.camera == SystemConfig.vo.camera_intrinsics");
    // The raw VO sub-tree must be forwarded, not dropped: the components built
    // from it (Initializer, KeyframeInserter, ...) read it verbatim.
    check(static_cast<bool>(sys.vo.vo_node) == static_cast<bool>(root["VO"]),
          "SystemConfig.vo.vo_node forwards the raw VO sub-tree");
}

} // namespace

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);

    const std::string config_path =
        (argc > 1) ? argv[1] : std::string(UAVLOC_MISSION_CONFIG_PATH);

    test_defaults_without_system_block();
    test_all_fields_read_back();
    test_input_policy_parsing();
    test_capacity_fallback();
    test_delegation(config_path);
    test_missing_camera_throws();

    if (!g_ok) {
        spdlog::error("test_system_config: FAIL");
        return 1;
    }
    spdlog::info("test_system_config: PASS");
    return 0;
}
