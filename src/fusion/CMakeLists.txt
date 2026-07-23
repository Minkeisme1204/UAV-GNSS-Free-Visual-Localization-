# fusion module — GTSAM fixed-lag-smoother back-end (Stage F1 skeleton).
# Requires: find_package(GTSAM REQUIRED) in the root CMakeLists.
target_sources(uavloc PRIVATE
    fusion_config.cpp
    fusion_module.cpp
)

# PRIVATE: gtsam is an internal implementation detail of the fusion module —
# the public headers under include/uavloc/fusion/ are gtsam-free (only
# Eigen/std/yaml-cpp types cross the API), so gtsam must not leak into the
# installed/exported interface of libuavloc.
target_link_libraries(uavloc PRIVATE gtsam)
