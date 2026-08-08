add_library(livo_slam_viz_lib STATIC visualizer.cpp)

target_include_directories(livo_slam_viz_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(
  livo_slam_viz_lib
  PUBLIC livo_slam_core_lib
  PRIVATE livo_slam_warnings)

if(LIVO_SLAM_ENABLE_RERUN)
  target_link_libraries(livo_slam_viz_lib PRIVATE rerun_sdk)
  target_compile_definitions(livo_slam_viz_lib PRIVATE LIVO_SLAM_ENABLE_RERUN)
endif()
