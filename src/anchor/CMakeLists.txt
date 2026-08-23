# anchor module — request/response producer of ABSOLUTE horizontal positions
# (AnchorInterface). M1 ships the groundtruth-driven FakeAnchor; the real VPR
# producer lands behind the same interface at M3-M5.
target_sources(uavloc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fake_anchor.cpp
)

if(UAVLOC_HAVE_VPR)
    target_sources(uavloc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vpr_anchor.cpp)
endif()
