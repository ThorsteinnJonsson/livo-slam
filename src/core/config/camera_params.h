#pragma once

#include <cstdint>
#include <sophus/se3.hpp>
#include <variant>

namespace livo_slam {

// Brown-Conrady, in OpenCV's coefficient order.
struct RadTanParams {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
};

// Equidistant (Kannala-Brandt) or OpenCV's fisheye model.
struct EquidistantParams {
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
};

using DistortionParams = std::variant<RadTanParams, EquidistantParams>;

struct CameraParams {
  Sophus::SE3d T_imu_cam;

  std::uint32_t width = 0;
  std::uint32_t height = 0;

  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  DistortionParams distortion;

  // t_imu = t_cam + time_shift
  double time_shift = 0.0;
};

}  // namespace livo_slam
