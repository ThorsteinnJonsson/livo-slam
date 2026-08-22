#include "visual_points.h"

#include <limits>

namespace livo_slam {

std::vector<VisualPoint> BuildVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    const State& state, const CameraParams& camera,
    const CameraUpdateParams& params, const VoxelMap& map) {
  // Mark the grid cells already covered by visual points in view, so they are
  // not filled again
  //
  // For each point
  //    Transform to the camera frame, Project, skip if behind
  //    Skip if not InFrame with a patch_size margin
  //    Find its grid cell, skip if already covered or already claimed by a
  //    better scoring point
  //    Claim the cell with this point and its CornerScore
  //
  // For each claimed cell
  //    Skip if the score is below the minimum
  //    CreateVisualPatch at the projected pixel
  //    Append a VisualPoint holding that patch

  return {};  // TODO: Implement
}

VisualPatch CreateVisualPatch(const std::vector<cv::Mat>& image_pyramid,
                              const Eigen::Vector2d& pixel,
                              const Sophus::SE3d& T_world_camera,
                              double inv_exposure, std::size_t patch_size) {
  return {};  // TODO: Implement
}

double CornerScore(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  return std::numeric_limits<double>::signaling_NaN();  // TODO: Implement
}

}  // namespace livo_slam