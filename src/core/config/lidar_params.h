#pragma once

#include <sophus/se3.hpp>

namespace livo_slam {

struct LidarParams {
  Sophus::SE3d T_imu_lidar;

  double min_range = 1.0;
  double max_range = 60.0;
  double voxel_leaf_size = 0.5;

  // t_imu = t_lidar_header + time_shift
  double time_shift = 0.0;
};

}  // namespace livo_slam
