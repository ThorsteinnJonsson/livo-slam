add_library(
  livo_slam_core_lib
  STATIC
    pipeline/livo_pipeline.cpp
    pipeline/measurement_buffer.cpp
    imu/imu_initializer.cpp
    imu/imu_propagator.cpp
    estimator/initial_estimate.cpp)

target_include_directories(livo_slam_core_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(
  livo_slam_core_lib
  PUBLIC Eigen3::Eigen Sophus::Sophus
  PRIVATE livo_slam_warnings)
