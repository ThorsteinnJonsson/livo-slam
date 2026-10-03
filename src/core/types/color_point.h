#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace livo_slam {

struct ColorPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

}  // namespace livo_slam
