#include "camera_residual.h"

#include <Eigen/src/Core/Matrix.h>

#include <limits>
#include <optional>

#include "core/camera/projection.h"
#include "core/math/skew.h"
#include "core/types/state.h"
#include "core/types/visual_point.h"

namespace livo_slam {

const VisualPatch* SelectReferencePatch(
    const VisualPoint& visual_point, const Eigen::Vector3d& camera_position) {
  const Eigen::Vector3d view_dir =
      (camera_position - visual_point.position).normalized();

  double max_cos_angle = std::numeric_limits<double>::lowest();
  const VisualPatch* best_match = nullptr;
  for (const VisualPatch& patch : visual_point.patches) {
    const Eigen::Vector3d patch_dir =
        (patch.T_world_camera.translation() - visual_point.position)
            .normalized();
    const double cos_angle = view_dir.dot(patch_dir);
    if (best_match == nullptr || max_cos_angle < cos_angle) {
      max_cos_angle = cos_angle;
      best_match = &patch;
    }
  }
  return best_match;
}

PhotometricMatch CreatePhotometricMatch(const cv::Mat& image,
                                        const Eigen::Vector2d& pixel,
                                        const Eigen::Vector3d& point_imu,
                                        const VisualPatch& reference_patch,
                                        std::size_t pyramid_level,
                                        std::size_t patch_size) {
  const int half_patch = static_cast<int>(patch_size) / 2;
  const Eigen::Index total_patch_size =
      static_cast<Eigen::Index>(patch_size * patch_size);

  PhotometricMatch match;
  match.point_imu = point_imu;
  match.reference_patch = &reference_patch.intensities[pyramid_level];
  match.reference_inv_exposure = reference_patch.inv_exposure;
  match.current_patch.resize(total_patch_size);
  match.gradients.resize(total_patch_size, 2);

  Eigen::Index index = 0;
  for (int dv = -half_patch; dv < half_patch; ++dv) {
    for (int du = -half_patch; du < half_patch; ++du) {
      const Eigen::Vector2d sample_pixel = pixel + Eigen::Vector2d(du, dv);
      match.current_patch(index) =
          static_cast<float>(SampleBilinear(image, sample_pixel));
      match.gradients(index, 0) =
          0.5 *
          (SampleBilinear(image, sample_pixel + Eigen::Vector2d::UnitX()) -
           SampleBilinear(image, sample_pixel - Eigen::Vector2d::UnitX()));
      match.gradients(index, 1) =
          0.5 *
          (SampleBilinear(image, sample_pixel + Eigen::Vector2d::UnitY()) -
           SampleBilinear(image, sample_pixel - Eigen::Vector2d::UnitY()));
      ++index;
    }
  }
  return match;
}

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params,
    const CameraUpdateParams& update_params) {
  const std::size_t margin = update_params.patch_size / 2 + 1;
  const double scale =
      1.0 / static_cast<double>(std::size_t{1} << pyramid_level);

  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Sophus::SO3d R_inv = state.rotation.inverse();
  const Sophus::SE3d T_world_imu(state.rotation, state.position);
  const Eigen::Vector3d cam_position =
      (T_world_imu * camera_params.T_imu_cam).translation();

  std::vector<PhotometricMatch> matches;
  matches.reserve(visual_points.size());
  for (const VisualPoint* visual_point : visual_points) {
    const Eigen::Vector3d p_imu =
        R_inv * (visual_point->position - state.position);
    const Eigen::Vector3d p_cam = T_cam_imu * p_imu;

    const std::optional<Eigen::Vector2d> projected =
        Project(camera_params, p_cam);
    if (!projected) continue;

    // Scale the pixel to this pyramid level
    const Eigen::Vector2d pixel = (*projected) * scale;

    if (!InFrame(image, pixel, margin)) continue;

    const VisualPatch* reference_patch =
        SelectReferencePatch(*visual_point, cam_position);

    if (!reference_patch) continue;  // Never observed

    const PhotometricMatch match =
        CreatePhotometricMatch(image, pixel, p_imu, *reference_patch,
                               pyramid_level, update_params.patch_size);

    const double photometric_error =
        (match.reference_inv_exposure * (*match.reference_patch) -
         state.inv_exposure * match.current_patch)
            .squaredNorm();

    if (photometric_error < update_params.outlier_threshold) {
      matches.push_back(std::move(match));
    }
  }

  matches.shrink_to_fit();
  return matches;
}

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state,
                             const CameraParams& camera_params,
                             double intensity_noise_std) {
  IekfSystem system;

  const double weight = 1.0 / (intensity_noise_std * intensity_noise_std);
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Eigen::Matrix3d R_cam_imu = T_cam_imu.rotationMatrix();

  for (const auto& match : matches) {
    const Eigen::Vector3d& p_imu = match.point_imu;
    const Eigen::Vector3d p_cam = T_cam_imu * match.point_imu;

    // dU_dP is ∂u/∂p_cam. Move the 3D point in the camera frame, how far
    // the pixel u slides.
    const Eigen::Matrix<double, 2, 3> dU_dp =
        ProjectionJacobian(camera_params, p_cam);
    // dP_dtheta is ∂p_cam/∂δθ. Rotate the body a little, how the point's
    // camera-frame coordinates change.
    const Eigen::Matrix3d dP_dtheta = R_cam_imu * Skew(p_imu);
    // dP_dp is ∂p_cam/∂δp. Translate the body a little, how the point's
    // camera-frame coordinates change.
    const Eigen::Matrix3d dP_dp =
        -R_cam_imu * state.rotation.matrix().transpose();

    for (Eigen::Index i = 0; i < match.current_patch.size(); ++i) {
      const auto residual =
          match.reference_inv_exposure * (*match.reference_patch)[i] -
          state.inv_exposure * match.current_patch[i];
      const Eigen::RowVector3d row = match.gradients.row(i) * dU_dp;

      ErrorState jacobian = ErrorState::Zero();
      jacobian.segment<3>(kIdxRotation) =
          -state.inv_exposure * (row * dP_dtheta).transpose();
      jacobian.segment<3>(kIdxPosition) =
          -state.inv_exposure * (row * dP_dp).transpose();
      jacobian(kIdxInvExposure) = -match.current_patch[i];

      system.hth += weight * jacobian * jacobian.transpose();
      system.htz += weight * jacobian * -residual;
    }
  }

  return system;
}

}  // namespace livo_slam