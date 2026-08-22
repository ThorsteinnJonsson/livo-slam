#pragma once

#include <Eigen/Core>
#include <array>
#include <sophus/se3.hpp>
#include <vector>

#include "core/config/update_params.h"

namespace livo_slam {

struct VisualPatch {
  std::array<Eigen::VectorXf, CameraUpdateParams::kPyramidLevels>
      intensities;  // Intensity patch per-level
  Sophus::SE3d T_world_camera;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  double inv_exposure = 1.0;
};

struct VisualPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::vector<VisualPatch> patches;
};

}  // namespace livo_slam