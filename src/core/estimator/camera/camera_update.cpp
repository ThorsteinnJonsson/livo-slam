#include "camera_update.h"

#include <limits>
#include <print>

#include "core/estimator/camera/camera_residual.h"
#include "core/estimator/iekf.h"

namespace livo_slam {
CameraUpdateResult UpdateWithCamera(
    const Estimate& prior, const std::vector<cv::Mat>& image_pyramid,
    const std::vector<const VisualPoint*>& visual_points,
    const std::vector<Point>& scan_imu, const CameraParams& camera_params,
    const CameraUpdateParams& update_params) {
  CameraUpdateResult result;
  Estimate& posterior = result.estimate;
  posterior = prior;

  // Built once from the predicted pose and reused at every level.
  const cv::Mat depth_image =
      BuildDepthImage(image_pyramid[0].size(), scan_imu, camera_params);

  // Selected once per frame at the predicted pose, exactly as FAST-LIVO2 builds
  // its visual submap before iterating. The iteration then minimises a fixed
  // objective instead of one that changes as gates admit and drop points.
  const std::vector<SelectedPoint> selection = SelectCameraPoints(
      image_pyramid[0], 0, prior.state, visual_points, camera_params,
      update_params, &result.observations, depth_image);

  {  // TEMP DEBUG: how many stored patches the selected points actually have.
    std::size_t total_patches = 0;
    std::size_t single = 0;
    std::size_t over_five = 0;
    for (const SelectedPoint& selected : selection) {
      const std::size_t count = selected.point->patches.size();
      total_patches += count;
      if (count == 1) ++single;
      if (count > 5) ++over_five;
    }
    std::println("CAMSEL,{},{},{},{}", selection.size(), total_patches, single,
                 over_five);
  }

  for (std::size_t level = CameraUpdateParams::kPyramidLevels; level-- > 0;) {
    // A step is kept only while it lowers the photometric cost. The state
    // behind the best cost so far is held so a bad step can be undone.
    double best_cost = std::numeric_limits<double>::max();
    Estimate accepted = posterior;

    for (std::size_t iter = 0; iter < update_params.max_iterations; ++iter) {
      std::vector<PhotometricMatch> matches = BuildMatchesFromSelection(
          image_pyramid[level], level, posterior.state, selection,
          camera_params, update_params);

      if (matches.size() < CameraUpdateParams::kMinMatches) {
        break;  // Continue one level deeper, might have more matches.
      }

      double cost = 0.0;
      for (const auto& match : matches) {
        cost +=
            update_params.normalize_patches
                ? (match.reference_patch - match.current_patch).squaredNorm()
                : (match.reference_inv_exposure * match.reference_patch -
                   posterior.state.inv_exposure * match.current_patch)
                      .squaredNorm();
      }
      cost /= static_cast<double>(matches.size());

      result.iterations.push_back({level, iter, matches.size(), cost});

      if (cost > best_cost) {
        posterior = accepted;  // The last step made the image fit worse.
        ++result.rejected_steps;
        break;
      }
      best_cost = cost;
      accepted = posterior;

      IekfSystem system = BuildCameraSystem(
          matches, posterior.state, camera_params,
          update_params.intensity_noise_std, update_params.normalize_patches,
          update_params.estimate_exposure);

      auto iekf_result = SolveIekfStep(prior, posterior.state, system);

      posterior.state = BoxPlus(posterior.state, iekf_result.increment);
      posterior.covariance = iekf_result.covariance;
      ++result.applied_steps;

      if (IekfConverged(iekf_result.increment, update_params.rotation_epsilon,
                        update_params.position_epsilon)) {
        break;  // Converged, continue one level deeper
      }
    }
  }
  return result;
}
}  // namespace livo_slam