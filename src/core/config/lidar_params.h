#pragma once

#include <sophus/se3.hpp>

namespace livo_slam {

struct LidarParams {
  Sophus::SE3d imu_from_lidar;

  double min_range = 1.0;
  double max_range = 60.0;
};

}  // namespace livo_slam
