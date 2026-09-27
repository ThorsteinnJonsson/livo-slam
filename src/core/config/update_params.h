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
  // A patch has to sit on one surface. If the current scan sees anything this
  // much nearer or further inside the patch, the point is on a depth edge.
  double max_depth_discontinuity = 0.5;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
  // The update keeps only the nearest point per grid cell of this many pixels,
  // which spreads the constraints and stops one textured region dominating.
  // Zero disables the decimation.
  std::size_t selection_grid_size = 30;
  // Pick the reference patch that agrees best with the point's other patches
  // instead of the one taken from the closest viewing direction.
  bool reference_patch_medoid = false;
  bool reference_patch_scored = false;
  // Skip points whose voxel had no plane instead of warping them as if the
  // surface faced the reference camera.
  bool require_normal = false;
  // Pick the nearest point per grid cell before the gates run, so a cell whose
  // nearest point fails contributes nothing. This is FAST-LIVO2's order.
  bool decimate_before_gates = false;
  // Divide each patch by its own mean, which makes the residual blind to
  // brightness. Off puts exposure back in the residual and in the state.
  bool normalize_patches = true;
  bool open_loop = false;  // TEMP DEBUG: compute the update but discard it.
};

}  // namespace livo_slam
