#pragma once

#include <Eigen/Core>

namespace livo_slam {

// Where a visual point landed in the current image, and whether it survived the
// photometric gate. This is only used for visualization.
struct VisualObservation {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // world frame
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();     // full resolution
  bool matched = false;
};

}  // namespace livo_slam
