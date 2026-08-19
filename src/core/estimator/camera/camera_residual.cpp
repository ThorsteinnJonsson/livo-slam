#include "camera_residual.h"

#include "core/camera/projection.h"
#include "core/math/skew.h"
#include "core/types/state.h"

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
  IekfSystem system;

  const double weight = 1.0 / (intensity_noise_std * intensity_noise_std);
  const Sophus::SE3d T_cam_imu = camera.T_imu_cam.inverse();
  const Eigen::Matrix3d R_cam_imu = T_cam_imu.rotationMatrix();

  for (const auto& match : matches) {
    const Eigen::Vector3d& p_imu = match.point_imu;
    const Eigen::Vector3d p_cam = T_cam_imu * match.point_imu;

    // dU_dP is ∂u/∂p_cam. Move the 3D point in the camera frame, how far
    // the pixel u slides.
    const Eigen::Matrix<double, 2, 3> dU_dp = ProjectionJacobian(camera, p_cam);
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