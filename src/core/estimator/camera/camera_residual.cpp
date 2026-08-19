#include "camera_residual.h"

#include <optional>

#include "core/camera/projection.h"
#include "core/math/skew.h"
#include "core/types/state.h"
#include "core/types/visual_point.h"

namespace livo_slam {

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params,
    const CameraUpdateParams& update_params) {
  // For each visual point
  //    p_imu = state.rotation.inverse() * (position - state.position)
  //    p_cam = camera.T_imu_cam.inverse() * p_imu
  //    Project, skip if nullopt
  //    Scale the pixel to this level, skip if not InFrame with the margin
  //
  //    Pick the reference patch from point->patches
  //
  //    For each of the patch_size^2 offsets around the pixel
  //       current(i)     = SampleBilinear at pixel + offset
  //       gradients(i,0) = central difference in u
  //       gradients(i,1) = central difference in v
  //
  //    Skip if the photometric error against the reference exceeds
  //    outlier_threshold
  //
  //    Append {point_imu, reference, current, gradients,
  //            reference_inv_exposure}

  std::vector<PhotometricMatch> matches;

  const std::size_t margin = update_params.patch_size / 2 + 1;
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Sophus::SO3d R_inv = state.rotation.inverse();

  for (const VisualPoint* visual_point : visual_points) {
    const Eigen::Vector3d p_imu =
        R_inv * (visual_point->position - state.position);
    const Eigen::Vector3d p_cam = T_cam_imu * p_imu;

    const std::optional<Eigen::Vector2d> projected =
        Project(camera_params, p_cam);
    if (!projected) continue;

    // Scale the pixel to this pyramid level
    const double scale = 1.0 / static_cast<double>(1U << pyramid_level);
    const Eigen::Vector2d pixel = *projected * scale;

    if (!InFrame(image, pixel, margin)) continue;

    // TODO: Continue
  }
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
      const auto residual = match.current_patch[i] -
                            state.inv_exposure * match.reference_patch[i];
      const Eigen::RowVector3d row = match.gradients.row(i) * dU_dp;

      ErrorState jacobian = ErrorState::Zero();
      jacobian.segment<3>(kIdxRotation) = (row * dP_dtheta).transpose();
      jacobian.segment<3>(kIdxPosition) = (row * dP_dp).transpose();
      jacobian(kIdxInvExposure) = -match.reference_patch[i];

      system.hth += weight * jacobian * jacobian.transpose();
      system.htz += weight * jacobian * -residual;
    }
  }

  return system;
}

}  // namespace livo_slam