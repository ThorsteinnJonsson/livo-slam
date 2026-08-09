add_executable(livo_slam_tests)

add_subdirectory(core)

target_link_libraries(
  livo_slam_tests
  PRIVATE livo_slam_core_lib livo_slam_warnings GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(livo_slam_tests)
