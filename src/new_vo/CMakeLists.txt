# new_vo module — faithful mirror of stella_vslam being ported incrementally.
#
# PARTIALLY BUILDABLE: the VoFrameLoader subset (feature/orb + data/ + a real
# camera adapter + the sensor→frame bridge) is ENABLED and compiles into
# libuavloc. The remaining folders (solve/, match/, initialize/, optimize/, and
# the rest of module/) still need g2o + more seams and stay COMMENTED OUT.
# See .docs/designs/new_vo_port_notes.md.
target_sources(uavloc PRIVATE
    # ── feature/orb (ENABLED) ─────────────────────────────────────────────
    feature/orb/orb_extractor.cpp
    feature/orb/orb_impl.cpp
    feature/orb/orb_params.cpp

    # ── data/ (ENABLED) ───────────────────────────────────────────────────
    data/common.cpp
    data/frame.cpp
    data/keyframe.cpp
    data/landmark.cpp
    data/graph_node.cpp
    data/map_database.cpp
    # NOTE: frame_observation.h is header-only (no .cpp).
    # NOTE(pure-vo): data/{bow_database,bow_vocabulary,bow_vocabulary_fwd} REMOVED
    # (BoW handed to vpr/fusion — see .docs/designs/vo_design_v3.md §10).
    # NOTE(pure-vo): marker/* REMOVED — see vo_design_v3.md §10.

    # ── camera/ (ENABLED — real adapter, NEW code) ────────────────────────
    camera/base.cpp
    camera/perspective_camera.cpp
    # NOTE: camera::PerspectiveCamera adapts sensor::CameraModel to camera::Base;
    # base.cpp supplies camera::Base batch-virtual bodies (+ vtable).

    # NOTE: util helpers live in the shared uavloc::util module (src/util/):
    # angle, random_array and Converter (converter.cpp) are built there.

    # ── module/ (partially ENABLED) ───────────────────────────────────────
    module/vo_frame_loader.cpp
    # NOTE: vo_frame_loader is the sensor→frame bridge (sensor::FrameData →
    # data::Frame) reproducing stella's create_monocular_frame.

    # ── solve/ (ENABLED) ──────────────────────────────────────────────────
    solve/common.cpp
    solve/essential_solver.cpp
    solve/fundamental_solver.cpp
    solve/homography_solver.cpp
    solve/pnp_solver.cpp
    # NOTE: essential_5pt.h and triangulator.h are header-only (no .cpp).

    # ── match/ (ENABLED) ──────────────────────────────────────────────────
    match/area.cpp
    match/fuse.cpp
    match/projection.cpp
    match/robust.cpp
    # NOTE: base.h is header-only (no .cpp). stereo.* was NOT ported
    # (intentionally skipped; nadir is monocular). fuse.cpp provides
    # landmark-fusion (match::Fuse) used by LocalMapper.
    # NOTE(pure-vo): match/bow_tree REMOVED (BoW handed to vpr/fusion).

    # ── initialize/ (ENABLED) ─────────────────────────────────────────────
    initialize/base.cpp
    initialize/perspective.cpp
    initialize/bearing_vector.cpp

    # ── optimize/ (ENABLED; links g2o) ────────────────────────────────────
    optimize/pose_optimizer_g2o.cpp
    optimize/local_bundle_adjuster_g2o.cpp
    optimize/global_bundle_adjuster.cpp
    optimize/terminate_action.cpp
    # NOTE: gtsam backend NOT ported. Links g2o (.thirdparty/g2o/).
    # NOTE(pure-vo): optimize/{graph_optimizer,transform_optimizer} +
    # internal/sim3/ REMOVED (loop/Sim3 only — see vo_design_v3.md §10).

    # ── module/ (ENABLED rest; links g2o) ─────────────────────────────────
    module/frame_tracker.cpp
    module/initializer.cpp
    module/keyframe_inserter.cpp
    module/local_map_cleaner.cpp
    module/local_map_updater.cpp
    module/mapping_module.cpp
    module/two_view_triangulator.cpp

    # ── new_vo orchestration layer (VOModule glue) ────────────────────────
    module/vo_config.cpp
    module/vo_module.cpp
    module/local_mapper.cpp
    # NOTE: mapping_module.cpp holds minimal no-op MappingModule bodies (the
    # interface KeyframeInserter forwards keyframes to; VOModule provides the
    # real async orchestrator later).
    # NOTE(pure-vo): module/{loop_bundle_adjuster,loop_detector,relocalizer} +
    # module/type.h REMOVED (loop/reloc handed to vpr/fusion — vo_design_v3.md §10).
)

# ── g2o link (optimize/ + module/ pull in g2o core+stuff) ─────────────────────
# PRIVATE: g2o is an internal implementation detail of new_vo (only the .cpp in
# optimize/ + module/ touch it); it must not leak into the installed/exported
# interface of libuavloc.
target_link_libraries(uavloc PRIVATE g2o::core g2o::stuff)
target_include_directories(uavloc PRIVATE
    $<BUILD_INTERFACE:${G2O_ROOT}>
    $<BUILD_INTERFACE:${G2O_ROOT}/build>)

if(USE_SSE_ORB)
    target_compile_definitions(uavloc PRIVATE USE_SSE_ORB)
endif()
