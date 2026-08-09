target_sources(
  livo_slam_tests
  PRIVATE
    imu_initializer_test.cpp imu_propagator_test.cpp initial_estimate_test.cpp
    measurement_buffer_test.cpp state_test.cpp)
