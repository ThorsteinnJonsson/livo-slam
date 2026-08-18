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

struct CameraUpdateParams {
  static constexpr std::size_t kMinMatches = 6;

  std::size_t max_iterations = 5;
  std::size_t patch_size = 8;
  std::size_t pyramid_levels = 3;
  double intensity_noise_std = 10.0;
  double outlier_threshold = 1000.0;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
};

}  // namespace livo_slam
