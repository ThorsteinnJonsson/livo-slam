#include "camera_update.h"

#include <opencv2/imgproc.hpp>
#include <print>

#include "core/estimator/camera/camera_residual.h"
#include "core/estimator/iekf.h"
#include "core/types/visual_point.h"

namespace livo_slam {
Estimate UpdateWithCamera(const Estimate& prior, const ImageFrame& image_frame,
                          const Cloud& cloud_imu, const VoxelMap& map,
                          const CameraParams& camera_params,
                          const CameraUpdateParams& update_params) {
  Estimate posterior = prior;

  // Collect candidate visual points
  const Sophus::SE3d T_world_imu(posterior.state.rotation,
                                 posterior.state.position);
  Cloud cloud_world = cloud_imu;
  for (Point& point : cloud_world.points) {
    point.position = T_world_imu * point.position;
  }
  const std::vector<const VisualPoint*> visual_points =
      map.CollectVisualPoints(cloud_world);

  // Build image pyramid
  std::vector<cv::Mat> image_pyramid;
  cv::buildPyramid(image_frame.image, image_pyramid,
                   static_cast<int>(update_params.pyramid_levels - 1));

  for (std::size_t level = update_params.pyramid_levels; level-- > 0;) {
    for (std::size_t iter = 0; iter < update_params.max_iterations; ++iter) {
      const cv::Mat& image = image_pyramid[level];
      std::vector<PhotometricMatch> matches =
          BuildCameraMatches(image, level, posterior.state, visual_points,
                             camera_params, update_params);

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
  return posterior;
}
}  // namespace livo_slam