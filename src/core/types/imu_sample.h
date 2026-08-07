#pragma once

#include <Eigen/Core>

namespace livo_slam {

struct ImuSample {
  double timestamp = 0.0;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

}  // namespace livo_slam
