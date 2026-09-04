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
    camera/projection.cpp
    map/voxel_octree.cpp
    map/voxel_map.cpp
    map/camera/visual_point_selection.cpp
    estimator/iekf.cpp
    estimator/initial_estimate.cpp
    estimator/lidar/lidar_residual.cpp
    estimator/lidar/lidar_update.cpp
    estimator/camera/camera_residual.cpp
    estimator/camera/camera_update.cpp
    math/voxel_key.cpp)

target_include_directories(livo_slam_core_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(
  livo_slam_core_lib
  PUBLIC Eigen3::Eigen Sophus::Sophus ${OpenCV_LIBS}
  PRIVATE livo_slam_warnings)
