#include "visual_point_selection.h"

#include <limits>

namespace livo_slam {

VisualMapUpdate SelectVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    const State& state, const CameraParams& camera,
    const CameraUpdateParams& params,
    const std::vector<const VisualPoint*>& existing_visual_points) {
  // Cells are grid_size-pixel squares of the image.
  //
  // For each existing visual point
  //    Project, skip if behind or not InFrame with a patch_size margin
  //    Mark its cell as covered, and remember the point and its pixel
  //
  // For each candidate point
  //    Project, skip if behind or not InFrame with a patch_size margin
  //    Skip if its cell is covered
  //    Keep it as the cell's candidate if its CornerScore is the highest so
  //    far, and its depth the smallest of the candidates in its voxel
  //
  // For each covered cell
  //    Add a patch for its visual point if more than min_frames_between_patches
  //    have passed since its last patch, or the projection has moved further
  //    than min_patch_pixel_distance from where that patch was taken
  //
  // For each cell with a candidate
  //    Skip if the score is below the minimum
  //    CreateVisualPatch at the projected pixel
  //    Append a VisualPoint holding that patch and the voxel's plane normal
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