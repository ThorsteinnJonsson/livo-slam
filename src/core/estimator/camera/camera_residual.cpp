#include "camera_residual.h"

namespace livo_slam {

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera, const CameraUpdateParams& params) {
  //          For each visual point in the map near the current view
  //             Transform to the camera frame, Project, skip if behind
  //             InFrame with a patch_size margin, skip if too close to an edge
  //             SampleBilinear the patch, skip if the photometric error
  //             exceeds outlier_threshold

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