#include "camera_residual.h"

namespace livo_slam {

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera, const CameraUpdateParams& params) {
  // TODO: Implement
  return {};
}

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state, const CameraParams& camera,
                             double intensity_noise_std) {
  // TODO: Implement
  return {};
}

}  // namespace livo_slam