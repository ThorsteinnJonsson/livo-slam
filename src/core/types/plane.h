#pragma once

#include <Eigen/Core>

namespace livo_slam {

struct Plane {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double variance = 0.0;
};
}  // namespace livo_slam