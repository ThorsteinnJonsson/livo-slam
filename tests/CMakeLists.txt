add_executable(
  livo_slam_tests imu_initializer_test.cpp imu_propagator_test.cpp
  initial_estimate_test.cpp measurement_buffer_test.cpp state_test.cpp)

target_link_libraries(
  livo_slam_tests
  PRIVATE livo_slam_core_lib livo_slam_warnings GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(livo_slam_tests)
