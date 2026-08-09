#pragma once

#include <sophus/se3.hpp>

namespace livo_slam {

struct LidarParams {
  Sophus::SE3d imu_from_lidar;
};

}  // namespace livo_slam
