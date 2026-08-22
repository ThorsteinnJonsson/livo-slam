#include "visual_point_selection.h"

#include <limits>

namespace livo_slam {

std::vector<VisualPoint> BuildVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    const State& state, const CameraParams& camera,
    const CameraUpdateParams& params,
    const std::vector<const VisualPoint*>& existing_visual_points) {
  // Project the existing visual points and mark the cells they land in. Cells
  // are grid_size-pixel squares of the image, one new point each.
  //
  // For each point
  //    Transform to the camera frame, Project, skip if behind
  //    Skip if not InFrame with a patch_size margin
  //    Find its cell, skip if that cell is marked or already holds a
  //    higher-scoring candidate
  //    Make this point the cell's candidate, with its CornerScore
  //
  // For each cell holding a candidate
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