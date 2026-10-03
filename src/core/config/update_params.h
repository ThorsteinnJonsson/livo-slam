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
  // How far the scan may differ in depth inside a patch before the point counts
  // as sitting on an edge.
  double max_depth_discontinuity = 0.5;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
  // Only the nearest point in each cell of this many pixels is used, so the
  // constraints spread over the image. Zero keeps every point.
  std::size_t selection_grid_size = 30;
  // Off compares raw intensities and estimates exposure. On divides each patch
  // by its own mean, which also blinds the outlier threshold to brightness.
  bool normalize_patches = true;
  // Off holds inverse exposure at its initial value.
  bool estimate_exposure = true;
};

}  // namespace livo_slam
