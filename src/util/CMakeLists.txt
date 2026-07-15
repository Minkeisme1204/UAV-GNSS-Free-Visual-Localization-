# util module — minimal, dependency-free helpers.
# Chỉ compile keep-set util; các util cần g2o/sqlite3/camera-config đã bỏ, thêm lại khi cần.

target_sources(uavloc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/angle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/random_array.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sys_monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yaml.cpp
    # NOTE: converter.cpp (util::Converter) uses g2o's SE3Quat; the g2o include
    # dirs + link are configured on the uavloc target in src/new_vo/CMakeLists.txt.
    ${CMAKE_CURRENT_SOURCE_DIR}/converter.cpp
)
