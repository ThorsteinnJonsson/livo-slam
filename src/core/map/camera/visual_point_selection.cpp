#include "visual_point_selection.h"

#include <limits>

namespace livo_slam {

VisualMapUpdate SelectVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    const State& state, const CameraParams& camera,
    const CameraUpdateParams& params,
    const std::vector<const VisualPoint*>& existing_visual_points) {
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