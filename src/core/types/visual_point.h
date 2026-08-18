#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <sophus/se3.hpp>
#include <vector>

namespace livo_slam {

struct VisualPatch {
  Eigen::VectorXf intensities;
  Sophus::SE3d world_from_camera;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  std::size_t level = 0;
  double inv_exposure = 1.0;
};

struct VisualPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::vector<VisualPatch> patches;
};

}  // namespace livo_slam