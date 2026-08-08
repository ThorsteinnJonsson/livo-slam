#pragma once

#include <Eigen/Core>

namespace livo_slam {

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d out;
  // clang-format off
  out << 0.0,  -v.z(), v.y(), 
         v.z(), 0.0,  -v.x(), 
        -v.y(), v.x(), 0.0;
  // clang-format on
  return out;
}

}  // namespace livo_slam
