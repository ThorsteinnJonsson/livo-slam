#include "camera_update.h"

#include "core/estimator/camera/camera_residual.h"
#include "core/estimator/iekf.h"

namespace livo_slam {
CameraUpdateResult UpdateWithCamera(
    const Estimate& prior, const std::vector<cv::Mat>& image_pyramid,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params,
    const CameraUpdateParams& update_params) {
  CameraUpdateResult result;
  Estimate& posterior = result.estimate;
  posterior = prior;

  for (std::size_t level = CameraUpdateParams::kPyramidLevels; level-- > 0;) {
    for (std::size_t iter = 0; iter < update_params.max_iterations; ++iter) {
      const cv::Mat& image = image_pyramid[level];
      // Record where every visible point landed on the last pass at full
      // resolution, for the visualization.
      const bool record = level == 0;
      if (record) {
        result.observations.clear();
      }
      std::vector<PhotometricMatch> matches = BuildCameraMatches(
          image, level, posterior.state, visual_points, camera_params,
          update_params, record ? &result.observations : nullptr);

      if (matches.size() < CameraUpdateParams::kMinMatches) {
        break;  // Continue one level deeper, might have more matches.
      }

      IekfSystem system =
          BuildCameraSystem(matches, posterior.state, camera_params,
                            update_params.intensity_noise_std);

      auto iekf_result = SolveIekfStep(prior, posterior.state, system);

      posterior.state = BoxPlus(posterior.state, iekf_result.increment);
      posterior.covariance = iekf_result.covariance;

      if (IekfConverged(iekf_result.increment, update_params.rotation_epsilon,
                        update_params.position_epsilon)) {
        break;  // Converged, continue one level deeper
      }
    }
  }
  return result;
}
}  // namespace livo_slam