# vo module — current refactor ships these sources; VOModule wraps the whole
# pipeline (detector -> matcher -> estimator -> initializer -> tracker) behind a
# synchronous processFrame() entry point.
target_sources(uavloc PRIVATE
    feature_detector.cpp
    frame_queue.cpp
    projection_matcher.cpp
    pose_estimator.cpp
    local_map.cpp
    local_mapper.cpp
    initializer.cpp
    tracker.cpp
    local_optimizer.cpp
    vo_module.cpp
)
