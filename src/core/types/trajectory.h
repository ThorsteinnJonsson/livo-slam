#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

namespace livo_slam {

struct ImuPose {
  double timestamp = 0.0;

  Sophus::SE3d pose;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Trajectory = std::vector<ImuPose>;

}  // namespace livo_slam
