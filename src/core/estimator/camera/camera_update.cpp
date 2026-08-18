#include "camera_update.h"

#include <opencv2/imgproc.hpp>

#include "core/estimator/camera/camera_residual.h"

namespace livo_slam {
Estimate UpdateWithCamera(const Estimate& prior, const ImageFrame& image_frame,
                          const VoxelMap& map,
                          const CameraParams& camera_params,
                          const CameraUpdateParams& update_params) {
  // Start from the prior: state to iterate, covariance to carry
  //
  // For each pyramid level, coarsest first
  //    For iteration 0..max_iterations
  //       BuildCameraMatches(image, state, map, camera)
  //          For each visual point in the map near the current view
  //             Transform to the camera frame, Project, skip if behind
  //             InFrame with a patch_size margin, skip if too close to an edge
  //             SampleBilinear the patch, skip if the photometric error
  //             exceeds outlier_threshold
  //       Stop if there are fewer than kMinMatches
  //       BuildCameraSystem(matches, state, camera, intensity_noise_std)
  //       SolveIekfStep(prior, state, system)
  //       state = BoxPlus(state, increment)
  //       Stop if IekfConverged(increment, ...)
  //
  // Return the state, covariance and prior.timestamp

  Estimate posterior = prior;

  std::vector<cv::Mat> image_pyramid;
  cv::buildPyramid(image_frame.image, image_pyramid,
                   static_cast<int>(update_params.pyramid_levels - 1));

  for (std::size_t level = update_params.pyramid_levels; level-- >= 0;) {
    for (std::size_t iter = 0; iter < update_params.max_iterations; ++iter) {
      const cv::Mat& image = image_pyramid[level];
      std::vector<PhotometricMatch> matches = BuildCameraMatches(
          image, posterior.state, map, camera_params, update_params);
    }
  }
  return posterior;
}
}  // namespace livo_slam