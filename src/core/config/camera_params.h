#pragma once

#include <cstdint>
#include <sophus/se3.hpp>

namespace livo_slam {

struct CameraParams {
  Sophus::SE3d imu_from_cam;

  std::uint32_t width = 0;
  std::uint32_t height = 0;

  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;

  // t_imu = t_cam + time_shift
  double time_shift = 0.0;
};

}  // namespace livo_slam
