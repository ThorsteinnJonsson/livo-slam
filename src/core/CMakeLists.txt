add_library(
  livo_slam_core_lib
  STATIC
    pipeline/livo_pipeline.cpp
    pipeline/measurement_buffer.cpp
    imu/imu_initializer.cpp
    imu/imu_propagator.cpp
    lidar/deskew.cpp
    lidar/downsample.cpp
    lidar/range_filter.cpp
    map/voxel_octree.cpp
    map/voxel_map.cpp
    estimator/iekf.cpp
    estimator/initial_estimate.cpp
    estimator/lidar_residual.cpp
    estimator/lidar_update.cpp
    math/voxel_key.cpp)

target_include_directories(livo_slam_core_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(
  livo_slam_core_lib
  PUBLIC Eigen3::Eigen Sophus::Sophus
  PRIVATE livo_slam_warnings)
