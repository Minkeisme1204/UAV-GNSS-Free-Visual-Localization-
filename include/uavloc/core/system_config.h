#pragma once

// SystemConfig — one assembled configuration for core::SystemManager (S2).
// See .docs/designs/system_manager_design.md §4.7.
//
// It does NOT re-parse the VO / Fusion sub-trees: fromYaml() delegates to
// vo::VOConfig::fromYaml() and fusion::FusionConfig::fromYaml() so the
// packaging layer can never make an existing mission config behave
// differently. Only the "System:" block is read here, with the
// node["key"].as<T>(default) pattern — a config with no "System:" block runs
// on the defaults below (synchronous + BLOCK), i.e. unchanged behaviour.
//
// fromYaml() CAN throw, and that is deliberate: it delegates to
// vo::VOConfig::fromYaml(), which rejects a mission config with no "Camera:"
// section. Camera intrinsics are not an optional knob — defaulting them would
// put a degenerate K (fx = fy = 0) into the pipeline and turn a config
// mistake into silently wrong poses. Optional System: keys, in contrast, are
// tolerant: missing keys take their default, an unknown input_policy string
// warns and keeps the default.

#include "uavloc/core/system_types.h"
#include "uavloc/fusion/fusion_config.h"
#include "uavloc/new_vo/vo_config.h"
#include "uavloc/sensor/camera_model.h"

#include <yaml-cpp/yaml.h>

namespace uavloc {
namespace core {

struct SystemConfig {
    //! The value util::ThreadsafeQueue reads as "no limit". SystemConfig never
    //! hands this out: an unbounded input queue makes push_blocking_if_full()
    //! never wait, which silently disables backpressure and lets the queue
    //! grow without bound.
    static constexpr unsigned int UNBOUNDED_INPUT_QUEUE_CAPACITY = 0;

    //! Default input-queue depth [frames], and the value fromYaml() falls back
    //! to when the YAML asks for UNBOUNDED_INPUT_QUEUE_CAPACITY. One constant
    //! for both so the field default and the fallback can never drift apart.
    //! (A fallback of 1 would be the worst operating point — the producer
    //! would block almost immediately — and someone writing 0 meant "do not
    //! constrain me", not "punish me".)
    static constexpr unsigned int DEFAULT_INPUT_QUEUE_CAPACITY = 8;

    vo::VOConfig             vo;
    fusion::FusionConfig     fusion;
    sensor::CameraIntrinsics camera;

    //! Run the pipeline on its own thread behind an input queue. false =
    //! inline on the caller's thread (deterministic offline evaluation).
    bool async_input = false;

    //! Queue behaviour when full; only meaningful when async_input is true.
    InputPolicy input_policy = InputPolicy::BLOCK;

    //! Input queue depth [frames]. A configured 0 (UNBOUNDED) is rejected by
    //! fromYaml() and replaced with DEFAULT_INPUT_QUEUE_CAPACITY.
    unsigned int input_queue_capacity = DEFAULT_INPUT_QUEUE_CAPACITY;

    //! Deadline of a BLOCK push [ms]. 0 turns BLOCK into try-then-give-up.
    unsigned int push_timeout_ms = 100;

    bool enable_fusion  = true;   //!< run the fusion back-end
    bool enable_geo     = true;   //!< run geo-referencing (ENU → lat/lon)
    bool publish_images = false;  //!< attach the image to FrameProcessed

    //! Monitor-thread publication period [ms].
    unsigned int stats_period_ms = 500;

    //! Time span of the core::Extrapolator buffers [s] — how far back the
    //! attitude / gimbal / GNSS samples are kept so an image arriving late can
    //! still be paired with them. <= 0 means unbounded (memory grows for the
    //! whole flight), so the default is a few seconds.
    double extrapolator_buffer_span_sec = 5.0;

    //! Widest gap between the two samples bracketing a query that
    //! core::Extrapolator will still interpolate across [s]. 0 = no limit,
    //! which is the S6b behaviour and therefore the default: raising it above 0
    //! makes the assembly step build frames with NO telemetry where a channel
    //! went silent, instead of inventing a value in the hole.
    double extrapolator_max_gap_sec = 0.0;

    //! Build from the root of the mission YAML (reads "System" here, and
    //! delegates "VO" / "Camera" / "Fusion" to their own fromYaml()).
    static SystemConfig fromYaml(const YAML::Node& root);
};

} // namespace core
} // namespace uavloc
