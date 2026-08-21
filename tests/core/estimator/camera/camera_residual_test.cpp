#include "core/estimator/camera/camera_residual.h"

#include <gtest/gtest.h>

#include "core/camera/projection.h"

namespace livo_slam {
namespace {

CameraParams MakeCamera() {
  CameraParams camera;
  camera.width = 752;
  camera.height = 480;
  camera.fx = 425.0;
  camera.fy = 426.8;
  camera.cx = 386.0;
  camera.cy = 241.9;
  camera.T_imu_cam = Sophus::SE3d(Sophus::SO3d::exp({0.02, -0.01, 1.57}),
                                  Eigen::Vector3d(0.005, -0.124, 0.016));
  return camera;
}

State MakeState() {
  State state;
  state.rotation = Sophus::SO3d::exp({0.3, -0.2, 0.7});
  state.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  state.inv_exposure = 0.9;
  return state;
}

// One patch pixel, so the Jacobian can be read straight back out of htz.
PhotometricMatch MakeMatch() {
  PhotometricMatch match;
  match.point_imu = Eigen::Vector3d(0.7, 1.3, 4.0);
  match.reference_patch = Eigen::VectorXf::Constant(1, 120.0F);
  match.current_patch = Eigen::VectorXf::Constant(1, 130.0F);
  match.gradients = Eigen::Matrix<double, Eigen::Dynamic, 2>(1, 2);
  match.gradients.row(0) = Eigen::RowVector2d(3.5, -2.1);
  return match;
}

double Residual(const PhotometricMatch& match, const State& state) {
  return static_cast<double>(match.current_patch[0]) -
         state.inv_exposure * static_cast<double>(match.reference_patch[0]);
}

// htz = weight * jacobian * -residual, so the jacobian divides back out.
ErrorState RecoverJacobian(const IekfSystem& system,
                           const PhotometricMatch& match, const State& state,
                           double intensity_noise_std) {
  const double weight = 1.0 / (intensity_noise_std * intensity_noise_std);
  return system.htz / (weight * -Residual(match, state));
}

TEST(BuildCameraSystem, NoMatchesGivesAnEmptySystem) {
  const IekfSystem system =
      BuildCameraSystem({}, MakeState(), MakeCamera(), 10.0);

  EXPECT_TRUE(system.hth.isZero());
  EXPECT_TRUE(system.htz.isZero());
}

TEST(BuildCameraSystem, TouchesOnlyThePoseAndExposureEntries) {
  const IekfSystem system =
      BuildCameraSystem({MakeMatch()}, MakeState(), MakeCamera(), 10.0);

  const ErrorState jacobian =
      RecoverJacobian(system, MakeMatch(), MakeState(), 10.0);

  EXPECT_FALSE(jacobian.segment<3>(kIdxRotation).isZero());
  EXPECT_FALSE(jacobian.segment<3>(kIdxPosition).isZero());
  EXPECT_NE(jacobian(kIdxInvExposure), 0.0);

  EXPECT_TRUE(jacobian.segment<3>(kIdxVelocity).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxGyroBias).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxAccelBias).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxGravity).isZero());
}

TEST(BuildCameraSystem, ExposureEntryIsTheNegatedReferenceIntensity) {
  const PhotometricMatch match = MakeMatch();
  const State state = MakeState();

  const IekfSystem system =
      BuildCameraSystem({match}, state, MakeCamera(), 10.0);
  const ErrorState jacobian = RecoverJacobian(system, match, state, 10.0);

  EXPECT_NEAR(jacobian(kIdxInvExposure),
              -static_cast<double>(match.reference_patch[0]), 1e-9);
}

TEST(BuildCameraSystem, JacobianMatchesFiniteDifferences) {
  const CameraParams camera = MakeCamera();
  const State state = MakeState();
  const PhotometricMatch match = MakeMatch();
  const double noise = 10.0;

  const IekfSystem system = BuildCameraSystem({match}, state, camera, noise);
  const ErrorState analytic = RecoverJacobian(system, match, state, noise);

  // The point is fixed in the world; perturbing the state moves the camera.
  const Eigen::Vector3d point_world =
      state.rotation * match.point_imu + state.position;
  const Eigen::Vector2d base_pixel =
      *Project(camera, camera.T_imu_cam.inverse() * match.point_imu);
  const Eigen::RowVector2d gradient = match.gradients.row(0);

  // The image is modelled by its local linearization, which is exactly what
  // the analytic Jacobian assumes.
  const auto residual_at = [&](const ErrorState& delta) {
    const State perturbed = BoxPlus(state, delta);
    const Eigen::Vector3d point_imu =
        perturbed.rotation.inverse() * (point_world - perturbed.position);
    const Eigen::Vector2d pixel =
        *Project(camera, camera.T_imu_cam.inverse() * point_imu);
    const double intensity = static_cast<double>(match.current_patch[0]) +
                             gradient.dot(pixel - base_pixel);
    return intensity - perturbed.inv_exposure *
                           static_cast<double>(match.reference_patch[0]);
  };

  const double eps = 1e-7;
  const double base = residual_at(ErrorState::Zero());
  for (int index :
       {kIdxRotation, kIdxRotation + 1, kIdxRotation + 2, kIdxPosition,
        kIdxPosition + 1, kIdxPosition + 2, kIdxInvExposure}) {
    ErrorState delta = ErrorState::Zero();
    delta(index) = eps;
    const double numeric = (residual_at(delta) - base) / eps;
    EXPECT_NEAR(analytic(index), numeric, 1e-3) << "index " << index;
  }
}

TEST(BuildCameraSystem, WeightsByInverseNoiseVariance) {
  const State state = MakeState();
  const CameraParams camera = MakeCamera();
  const PhotometricMatch match = MakeMatch();

  const IekfSystem coarse = BuildCameraSystem({match}, state, camera, 20.0);
  const IekfSystem fine = BuildCameraSystem({match}, state, camera, 10.0);

  EXPECT_TRUE(fine.hth.isApprox(4.0 * coarse.hth));
  EXPECT_TRUE(fine.htz.isApprox(4.0 * coarse.htz));
}

TEST(BuildCameraSystem, AccumulatesOverMatches) {
  const State state = MakeState();
  const CameraParams camera = MakeCamera();
  const PhotometricMatch match = MakeMatch();

  const IekfSystem one = BuildCameraSystem({match}, state, camera, 10.0);
  const IekfSystem two = BuildCameraSystem({match, match}, state, camera, 10.0);

  EXPECT_TRUE(two.hth.isApprox(2.0 * one.hth));
  EXPECT_TRUE(two.htz.isApprox(2.0 * one.htz));
}

TEST(BuildCameraSystem, AccumulatesOverPatchPixels) {
  const State state = MakeState();
  const CameraParams camera = MakeCamera();

  PhotometricMatch wide = MakeMatch();
  wide.reference_patch = Eigen::VectorXf::Constant(4, 120.0F);
  wide.current_patch = Eigen::VectorXf::Constant(4, 130.0F);
  wide.gradients = Eigen::Matrix<double, Eigen::Dynamic, 2>(4, 2);
  for (Eigen::Index i = 0; i < 4; ++i) {
    wide.gradients.row(i) = Eigen::RowVector2d(3.5, -2.1);
  }

  const IekfSystem one = BuildCameraSystem({MakeMatch()}, state, camera, 10.0);
  const IekfSystem four = BuildCameraSystem({wide}, state, camera, 10.0);

  EXPECT_TRUE(four.hth.isApprox(4.0 * one.hth));
  EXPECT_TRUE(four.htz.isApprox(4.0 * one.htz));
}

TEST(BuildCameraSystem, ZeroGradientContributesNothing) {
  PhotometricMatch match = MakeMatch();
  match.gradients.setZero();

  const IekfSystem system =
      BuildCameraSystem({match}, MakeState(), MakeCamera(), 10.0);

  // A textureless patch says nothing about the pose, but still constrains
  // exposure.
  const ErrorState jacobian = RecoverJacobian(system, match, MakeState(), 10.0);
  EXPECT_TRUE(jacobian.segment<3>(kIdxRotation).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxPosition).isZero());
  EXPECT_NE(jacobian(kIdxInvExposure), 0.0);
}

TEST(BuildCameraSystem, ResidualOpposesTheIntensityDifference) {
  const State state = MakeState();
  const CameraParams camera = MakeCamera();

  PhotometricMatch brighter = MakeMatch();
  PhotometricMatch darker = MakeMatch();
  // Mirror the residual about zero: current = inv_exposure * reference +- 10.
  const float balanced =
      static_cast<float>(state.inv_exposure) * brighter.reference_patch[0];
  brighter.current_patch = Eigen::VectorXf::Constant(1, balanced + 10.0F);
  darker.current_patch = Eigen::VectorXf::Constant(1, balanced - 10.0F);

  const IekfSystem up = BuildCameraSystem({brighter}, state, camera, 10.0);
  const IekfSystem down = BuildCameraSystem({darker}, state, camera, 10.0);

  EXPECT_TRUE(up.htz.isApprox(-down.htz));
  EXPECT_TRUE(up.hth.isApprox(down.hth));
}

// A patch observed from `camera_position`, looking at the origin.
VisualPatch MakePatch(const Eigen::Vector3d& camera_position,
                      std::size_t level) {
  VisualPatch patch;
  patch.T_world_camera = Sophus::SE3d(Sophus::SO3d{}, camera_position);
  patch.level = level;
  return patch;
}

TEST(SelectReferencePatch, NoPatchesSelectsNothing) {
  const VisualPoint point;

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 0), nullptr);
}

TEST(SelectReferencePatch, NoPatchAtTheRequestedLevelSelectsNothing) {
  VisualPoint point;
  point.patches.push_back(MakePatch({0.0, 0.0, 1.0}, 0));
  point.patches.push_back(MakePatch({0.0, 0.0, 1.0}, 1));

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 2), nullptr);
}

TEST(SelectReferencePatch, SinglePatchAtTheLevelIsSelected) {
  VisualPoint point;
  point.patches.push_back(MakePatch({5.0, 5.0, 5.0}, 1));

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 1), &point.patches[0]);
}

TEST(SelectReferencePatch, PrefersTheClosestViewingDirection) {
  VisualPoint point;
  point.patches.push_back(MakePatch({1.0, 0.0, 0.0}, 0));  // 90 degrees off
  point.patches.push_back(MakePatch({0.2, 0.0, 1.0}, 0));  // nearly aligned
  point.patches.push_back(MakePatch({0.0, 1.0, 0.0}, 0));  // 90 degrees off

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 0), &point.patches[1]);
}

TEST(SelectReferencePatch, IgnoresOtherLevelsWhenChoosing) {
  VisualPoint point;
  // The best-aligned patch is at the wrong level, so the worse one wins.
  point.patches.push_back(MakePatch({0.0, 0.0, 1.0}, 1));
  point.patches.push_back(MakePatch({1.0, 0.0, 0.2}, 0));

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 0), &point.patches[1]);
}

TEST(SelectReferencePatch, SelectsAPatchViewedFromBehind) {
  VisualPoint point;
  // FAST-LIVO2 floors the comparison at zero and would return an arbitrary
  // patch here; we return the least-bad one.
  point.patches.push_back(MakePatch({0.0, 0.0, -1.0}, 0));  // opposite
  point.patches.push_back(MakePatch({1.0, 0.0, -0.5}, 0));  // less opposite

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 0), &point.patches[1]);
}

TEST(SelectReferencePatch, DistanceAlongTheViewDirectionDoesNotMatter) {
  VisualPoint point;
  point.patches.push_back(MakePatch({0.0, 0.0, 50.0}, 0));
  point.patches.push_back(MakePatch({0.5, 0.0, 1.0}, 0));

  // Both directions are normalised, so the far but aligned patch wins.
  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}, 0), &point.patches[0]);
}

}  // namespace
}  // namespace livo_slam
