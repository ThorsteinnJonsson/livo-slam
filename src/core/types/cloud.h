#pragma once

#include <Eigen/Core>
#include <vector>

namespace livo_slam {

struct Point {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  float intensity = 0.0F;
  double timestamp = 0.0;
};

struct Cloud {
  double start_time = 0.0;
  double end_time = 0.0;
  std::vector<Point> points;
};

}  // namespace livo_slam
