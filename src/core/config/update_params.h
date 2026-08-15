#pragma once

#include <cstddef>

namespace livo_slam {

struct LidarUpdateParams {
  // Rotation and position are six degrees of freedom, so fewer rows than this
  // cannot constrain them.
  static constexpr std::size_t kMinMatches = 6;

  std::size_t max_iterations = 5;
  double point_noise_std = 0.03;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
};

}  // namespace livo_slam
