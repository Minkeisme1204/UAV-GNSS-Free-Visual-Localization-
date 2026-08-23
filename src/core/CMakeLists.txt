# core module — SystemManager and friends (.docs/designs/system_manager_design.md)
target_sources(uavloc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/extrapolator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rate_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/system_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/system_manager.cpp
)
