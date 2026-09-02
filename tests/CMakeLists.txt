file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp)

add_executable(livo_slam_tests ${test_sources})

target_link_libraries(
  livo_slam_tests
  PRIVATE livo_slam_core_lib livo_slam_ros_lib livo_slam_warnings
          GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(livo_slam_tests)
