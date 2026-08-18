#include "camera_update.h"

namespace livo_slam {
Estimate UpdateWithCamera(const Estimate& prior, const ImageFrame& image_frame,
                          const VoxelMap& map,
                          const CameraUpdateParams& params) {
  // TODO: Implement, add camera parameters
  //
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
  return {};
}
}  // namespace livo_slam