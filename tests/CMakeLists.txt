add_executable(livo_slam_tests smoke_test.cpp node_test.cpp)

target_link_libraries(
  livo_slam_tests
  PRIVATE
    livo_slam_ros_lib livo_slam_warnings GTest::gtest_main Eigen3::Eigen
    Sophus::Sophus)

include(GoogleTest)
gtest_discover_tests(livo_slam_tests)
