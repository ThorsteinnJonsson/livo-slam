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
  static constexpr std::size_t kPyramidLevels = 3;

  std::size_t max_iterations = 5;
  std::size_t patch_size = 8;
  double intensity_noise_std = 10.0;
  double outlier_threshold = 1000.0;
  double max_view_angle_deg = 15.0;
  // A depth jump this big inside a patch means the point sits on an edge.
  double max_depth_discontinuity = 0.5;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
  // Keep only the nearest point per cell of this many pixels, so the points
  // spread over the image. Zero keeps all of them.
  std::size_t selection_grid_size = 30;
  // Off leaves inverse exposure where it started.
  bool estimate_exposure = true;
};

}  // namespace livo_slam
