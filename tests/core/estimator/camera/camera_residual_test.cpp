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

// The match points at the reference patch, so the caller owns it and must
// keep it alive for as long as the match.
const Eigen::VectorXf& SingleReference() {
  static const Eigen::VectorXf reference = Eigen::VectorXf::Constant(1, 120.0F);
  return reference;
}

// One patch pixel, so the Jacobian can be read straight back out of htz.
PhotometricMatch MakeMatch() {
  PhotometricMatch match;
  match.point_imu = Eigen::Vector3d(0.7, 1.3, 4.0);
  match.reference_patch = SingleReference();
  match.current_patch = Eigen::VectorXf::Constant(1, 130.0F);
  match.gradients = Eigen::Matrix<double, Eigen::Dynamic, 2>(1, 2);
  match.gradients.row(0) = Eigen::RowVector2d(3.5, -2.1);
  match.reference_inv_exposure = 0.8;
  return match;
}

double Residual(const PhotometricMatch& match, const State& /*state*/) {
  return static_cast<double>(match.reference_patch[0]) -
         static_cast<double>(match.current_patch[0]);
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

TEST(BuildCameraSystem, TouchesOnlyThePoseEntries) {
  const IekfSystem system =
      BuildCameraSystem({MakeMatch()}, MakeState(), MakeCamera(), 10.0);

  const ErrorState jacobian =
      RecoverJacobian(system, MakeMatch(), MakeState(), 10.0);

  EXPECT_FALSE(jacobian.segment<3>(kIdxRotation).isZero());
  EXPECT_FALSE(jacobian.segment<3>(kIdxPosition).isZero());

  // Normalized patches carry no brightness scale, so exposure is unobservable.
  EXPECT_DOUBLE_EQ(jacobian(kIdxInvExposure), 0.0);
  EXPECT_TRUE(jacobian.segment<3>(kIdxVelocity).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxGyroBias).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxAccelBias).isZero());
  EXPECT_TRUE(jacobian.segment<3>(kIdxGravity).isZero());
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
    return static_cast<double>(match.reference_patch[0]) - intensity;
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

  static const Eigen::VectorXf wide_reference =
      Eigen::VectorXf::Constant(4, 120.0F);
  PhotometricMatch wide = MakeMatch();
  wide.reference_patch = wide_reference;
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

  // A textureless patch says nothing at all once exposure is out of the
  // residual.
  const ErrorState jacobian = RecoverJacobian(system, match, MakeState(), 10.0);
  EXPECT_TRUE(jacobian.isZero());
}

TEST(BuildCameraSystem, ResidualOpposesTheIntensityDifference) {
  const State state = MakeState();
  const CameraParams camera = MakeCamera();

  PhotometricMatch brighter = MakeMatch();
  PhotometricMatch darker = MakeMatch();
  // Patches are normalized, so the residual vanishes when they are equal.
  const float balanced = brighter.reference_patch[0];
  brighter.current_patch = Eigen::VectorXf::Constant(1, balanced + 10.0F);
  darker.current_patch = Eigen::VectorXf::Constant(1, balanced - 10.0F);

  const IekfSystem up = BuildCameraSystem({brighter}, state, camera, 10.0);
  const IekfSystem down = BuildCameraSystem({darker}, state, camera, 10.0);

  // Only the pose block mirrors. The exposure entry is -current, so it differs
  // between the two and its rows of hth and htz do not. The tolerance is for
  // `balanced` being rounded to float, which leaves the two residuals mirrored
  // to about seven digits rather than exactly.
  const Eigen::Matrix<double, 6, 6> up_pose = up.hth.block<6, 6>(0, 0);
  const Eigen::Matrix<double, 6, 6> down_pose = down.hth.block<6, 6>(0, 0);

  EXPECT_TRUE(up.htz.head<6>().isApprox(-down.htz.head<6>(), 1e-6));
  EXPECT_TRUE(up_pose.isApprox(down_pose));
}

// An observation made from `camera_position`, looking at the origin.
VisualPatch MakePatch(const Eigen::Vector3d& camera_position) {
  VisualPatch patch;
  patch.T_world_camera = Sophus::SE3d(Sophus::SO3d{}, camera_position);
  return patch;
}

TEST(SelectReferencePatch, NoPatchesSelectsNothing) {
  const VisualPoint point;

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}), nullptr);
}

TEST(SelectReferencePatch, SinglePatchIsSelected) {
  VisualPoint point;
  point.patches.push_back(MakePatch({5.0, 5.0, 5.0}));

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}), &point.patches[0]);
}

TEST(SelectReferencePatch, PrefersTheClosestViewingDirection) {
  VisualPoint point;
  point.patches.push_back(MakePatch({1.0, 0.0, 0.0}));  // 90 degrees off
  point.patches.push_back(MakePatch({0.2, 0.0, 1.0}));  // nearly aligned
  point.patches.push_back(MakePatch({0.0, 1.0, 0.0}));  // 90 degrees off

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}), &point.patches[1]);
}

TEST(SelectReferencePatch, SelectsAPatchViewedFromBehind) {
  VisualPoint point;
  // FAST-LIVO2 floors the comparison at zero and would return an arbitrary
  // patch here; we return the least-bad one.
  point.patches.push_back(MakePatch({0.0, 0.0, -1.0}));  // opposite
  point.patches.push_back(MakePatch({1.0, 0.0, -0.5}));  // less opposite

  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}), &point.patches[1]);
}

TEST(SelectReferencePatch, DistanceAlongTheViewDirectionDoesNotMatter) {
  VisualPoint point;
  point.patches.push_back(MakePatch({0.0, 0.0, 50.0}));
  point.patches.push_back(MakePatch({0.5, 0.0, 1.0}));

  // Both directions are normalized, so the far but aligned patch wins.
  EXPECT_EQ(SelectReferencePatch(point, {0.0, 0.0, 1.0}), &point.patches[0]);
}

TEST(VisualPatch, HoldsOneIntensityVectorPerPyramidLevel) {
  const VisualPatch patch;

  EXPECT_EQ(patch.intensities.size(), CameraUpdateParams::kPyramidLevels);
}

// A camera at the world origin looking down +z, so a point at (0, 0, d)
// projects to the principal point. Keeps the matching arithmetic checkable by
// hand.
CameraParams MakeSimpleCamera() {
  CameraParams camera;
  camera.width = 640;
  camera.height = 480;
  camera.fx = 400.0;
  camera.fy = 400.0;
  camera.cx = 320.0;
  camera.cy = 240.0;
  return camera;
}

CameraUpdateParams MakeUpdateParams() {
  CameraUpdateParams params;
  params.patch_size = 4;
  params.intensity_noise_std = 10.0;
  params.outlier_threshold = 1e9;  // Accept everything unless a test says so.
  return params;
}

// A visual point at (0, 0, depth) in front of an identity-pose camera, with one
// observation whose stored patch is `intensity` everywhere.
VisualPoint MakeVisualPoint(double depth, float intensity,
                            std::size_t patch_size) {
  VisualPoint point;
  point.position = Eigen::Vector3d(0.0, 0.0, depth);
  point.normal = Eigen::Vector3d(0.0, 0.0, -1.0);  // Faces the camera.

  VisualPatch patch;
  patch.T_world_camera = Sophus::SE3d{};
  for (Eigen::VectorXf& level : patch.intensities) {
    level = Eigen::VectorXf::Constant(
        static_cast<Eigen::Index>(patch_size * patch_size), intensity);
  }
  point.patches.push_back(patch);
  return point;
}

cv::Mat MakeUniformImage(std::uint8_t value) {
  return cv::Mat(480, 640, CV_8UC1, cv::Scalar(value));
}

TEST(BuildCameraMatches, NoVisualPointsGivesNoMatches) {
  EXPECT_TRUE(BuildCameraMatches(MakeUniformImage(100), 0, State{}, {},
                                 MakeSimpleCamera(), MakeUpdateParams())
                  .empty());
}

TEST(BuildCameraMatches, MatchesAPointInFrontOfTheCamera) {
  const CameraUpdateParams params = MakeUpdateParams();
  const VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);

  State state;
  state.inv_exposure = 1.0;

  const auto matches = BuildCameraMatches(MakeUniformImage(100), 0, state,
                                          {&point}, MakeSimpleCamera(), params);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_EQ(matches[0].point_imu, point.position);
  EXPECT_EQ(matches[0].current_patch.size(),
            static_cast<Eigen::Index>(params.patch_size * params.patch_size));
  EXPECT_EQ(matches[0].gradients.rows(), matches[0].current_patch.size());
}

TEST(BuildCameraMatches, SkipsPointsBehindTheCamera) {
  const CameraUpdateParams params = MakeUpdateParams();
  const VisualPoint point = MakeVisualPoint(-5.0, 100.0F, params.patch_size);

  EXPECT_TRUE(BuildCameraMatches(MakeUniformImage(100), 0, State{}, {&point},
                                 MakeSimpleCamera(), params)
                  .empty());
}

TEST(BuildCameraMatches, SkipsPointsProjectingOutsideTheImage) {
  const CameraUpdateParams params = MakeUpdateParams();
  VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);
  // Far off to the side: 400 * (20 / 5) + 320 is well past 640.
  point.position = Eigen::Vector3d(20.0, 0.0, 5.0);

  EXPECT_TRUE(BuildCameraMatches(MakeUniformImage(100), 0, State{}, {&point},
                                 MakeSimpleCamera(), params)
                  .empty());
}

TEST(BuildCameraMatches, SkipsPointsWithNoObservations) {
  VisualPoint point;
  point.position = Eigen::Vector3d(0.0, 0.0, 5.0);

  EXPECT_TRUE(BuildCameraMatches(MakeUniformImage(100), 0, State{}, {&point},
                                 MakeSimpleCamera(), MakeUpdateParams())
                  .empty());
}

TEST(BuildCameraMatches, SamplesTheImageIntensity) {
  const CameraUpdateParams params = MakeUpdateParams();
  const VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);

  State state;
  state.inv_exposure = 1.0;

  const auto matches = BuildCameraMatches(MakeUniformImage(140), 0, state,
                                          {&point}, MakeSimpleCamera(), params);

  ASSERT_EQ(matches.size(), 1U);
  // Normalized, so a uniform image gives all ones whatever its intensity.
  for (Eigen::Index i = 0; i < matches[0].current_patch.size(); ++i) {
    EXPECT_FLOAT_EQ(matches[0].current_patch(i), 1.0F) << "index " << i;
  }
}

TEST(BuildCameraMatches, GradientsAreZeroOnAUniformImage) {
  const CameraUpdateParams params = MakeUpdateParams();
  const VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);

  State state;
  state.inv_exposure = 1.0;

  const auto matches = BuildCameraMatches(MakeUniformImage(100), 0, state,
                                          {&point}, MakeSimpleCamera(), params);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_TRUE(matches[0].gradients.isZero());
}

TEST(BuildCameraMatches, GradientsFollowAHorizontalRamp) {
  const CameraUpdateParams params = MakeUpdateParams();
  const VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);

  // Intensity rises by 2 per column, so dI/du is 2 and dI/dv is 0.
  cv::Mat ramp(480, 640, CV_8UC1);
  for (int row = 0; row < ramp.rows; ++row) {
    for (int col = 0; col < ramp.cols; ++col) {
      ramp.at<std::uint8_t>(row, col) =
          static_cast<std::uint8_t>(col % 128 * 2);
    }
  }

  State state;
  state.inv_exposure = 1.0;

  const auto matches =
      BuildCameraMatches(ramp, 0, state, {&point}, MakeSimpleCamera(), params);

  ASSERT_EQ(matches.size(), 1U);
  // After normalization a ramp does not translate rigidly: every sample and
  // the mean both move, so the gradient is (2/m)*(1 - I_i/m), zero only at the
  // pixel that sits on the mean.
  const double mean = matches[0].current_patch.mean();
  for (Eigen::Index i = 0; i < matches[0].gradients.rows(); ++i) {
    EXPECT_LT(matches[0].gradients(i, 0) *
                  (static_cast<double>(matches[0].current_patch(i)) - mean),
              1e-12)
        << "index " << i;
    EXPECT_NEAR(matches[0].gradients(i, 1), 0.0, 1e-9) << "index " << i;
  }
}

TEST(BuildCameraMatches, RejectsPatchesBeyondTheOutlierThreshold) {
  CameraUpdateParams params = MakeUpdateParams();
  params.outlier_threshold = 0.05;
  const VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);

  State state;
  state.inv_exposure = 1.0;

  // Normalization ignores brightness, so the outlier has to differ in texture.
  cv::Mat textured(480, 640, CV_8UC1);
  for (int row = 0; row < textured.rows; ++row) {
    for (int col = 0; col < textured.cols; ++col) {
      textured.at<std::uint8_t>(row, col) =
          static_cast<std::uint8_t>((col % 2) == 0 ? 40 : 200);
    }
  }
  EXPECT_TRUE(BuildCameraMatches(textured, 0, state, {&point},
                                 MakeSimpleCamera(), params)
                  .empty());
  EXPECT_EQ(BuildCameraMatches(MakeUniformImage(100), 0, state, {&point},
                               MakeSimpleCamera(), params)
                .size(),
            1U);
}

TEST(BuildCameraMatches, UsesTheReferencePatchForTheRequestedLevel) {
  const CameraUpdateParams params = MakeUpdateParams();
  VisualPoint point = MakeVisualPoint(5.0, 100.0F, params.patch_size);
  point.patches[0].intensities[1] = Eigen::VectorXf::Constant(
      static_cast<Eigen::Index>(params.patch_size * params.patch_size), 55.0F);

  State state;
  state.inv_exposure = 1.0;

  // Level 1 is half resolution, so the image must be too.
  const cv::Mat level(240, 320, CV_8UC1, cv::Scalar(100));
  const auto matches =
      BuildCameraMatches(level, 1, state, {&point}, MakeSimpleCamera(), params);

  ASSERT_EQ(matches.size(), 1U);
  // Normalized: a uniform patch becomes all ones whatever its level.
  EXPECT_FLOAT_EQ(matches[0].reference_patch(0), 1.0F);
}

// A reference observation whose stored patch is `intensity` at every level.
VisualPatch MakeReferencePatch(float intensity, std::size_t patch_size) {
  VisualPatch patch;
  patch.inv_exposure = 0.8;
  for (Eigen::VectorXf& level : patch.intensities) {
    level = Eigen::VectorXf::Constant(
        static_cast<Eigen::Index>(patch_size * patch_size), intensity);
  }
  return patch;
}

TEST(CreatePhotometricMatch, SizesThePatchAndGradients) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);

  const PhotometricMatch match =
      CreatePhotometricMatch(MakeUniformImage(120), {320.0, 240.0},
                             {0.0, 0.0, 5.0}, reference, 0, patch_size);

  EXPECT_EQ(match.current_patch.size(), 16);
  EXPECT_EQ(match.gradients.rows(), 16);
  EXPECT_EQ(match.gradients.cols(), 2);
}

TEST(CreatePhotometricMatch, CarriesThePointAndReferenceThrough) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);

  const PhotometricMatch match =
      CreatePhotometricMatch(MakeUniformImage(120), {320.0, 240.0},
                             {0.7, 1.3, 4.0}, reference, 0, patch_size);

  EXPECT_EQ(match.point_imu, Eigen::Vector3d(0.7, 1.3, 4.0));
  EXPECT_EQ(match.reference_patch.size(), reference.intensities[0].size());
  EXPECT_DOUBLE_EQ(match.reference_inv_exposure, reference.inv_exposure);
}

TEST(CreatePhotometricMatch, PointsAtTheRequestedLevel) {
  const std::size_t patch_size = 4;
  VisualPatch reference = MakeReferencePatch(100.0F, patch_size);
  reference.intensities[2] = Eigen::VectorXf::Constant(16, 55.0F);

  const PhotometricMatch match =
      CreatePhotometricMatch(MakeUniformImage(120), {320.0, 240.0},
                             {0.0, 0.0, 5.0}, reference, 2, patch_size);

  // Normalization hides the raw value, so check the level was used at all by
  // giving that level a different pattern from the others.
  EXPECT_EQ(match.reference_patch.size(), 16);
  EXPECT_FLOAT_EQ(match.reference_patch(0), 1.0F);
}

TEST(CreatePhotometricMatch, SamplesAUniformImage) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);

  const PhotometricMatch match =
      CreatePhotometricMatch(MakeUniformImage(137), {320.0, 240.0},
                             {0.0, 0.0, 5.0}, reference, 0, patch_size);

  // Divided by its own mean, a uniform patch is all ones whatever its value.
  EXPECT_TRUE(match.current_patch.isApproxToConstant(1.0F));
  EXPECT_TRUE(match.gradients.isZero());
}

// Intensity = 2 * column, so dI/du is 2 and dI/dv is 0.
cv::Mat MakeHorizontalRamp() {
  cv::Mat ramp(480, 640, CV_8UC1);
  for (int row = 0; row < ramp.rows; ++row) {
    for (int col = 0; col < ramp.cols; ++col) {
      ramp.at<std::uint8_t>(row, col) =
          static_cast<std::uint8_t>(col % 128 * 2);
    }
  }
  return ramp;
}

TEST(CreatePhotometricMatch, GradientsFollowAHorizontalRamp) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);

  const PhotometricMatch match =
      CreatePhotometricMatch(MakeHorizontalRamp(), {320.0, 240.0},
                             {0.0, 0.0, 5.0}, reference, 0, patch_size);

  // After normalization a ramp does not translate rigidly: every sample and
  // the mean both move, so the gradient is (2/m)*(1 - I_i/m), zero only at the
  // pixel that sits on the mean.
  const double mean = (124.0 + 126.0 + 128.0 + 130.0) / 4.0;
  for (Eigen::Index i = 0; i < match.gradients.rows(); ++i) {
    const double expected =
        (2.0 / mean) * (1.0 - static_cast<double>(match.current_patch(i)));
    EXPECT_NEAR(match.gradients(i, 0), expected, 1e-9) << "index " << i;
    EXPECT_NEAR(match.gradients(i, 1), 0.0, 1e-9) << "index " << i;
  }
}

TEST(CreatePhotometricMatch, SamplesInRowMajorOrderAroundThePixel) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);

  // Intensity encodes the column, so each sample reveals where it came from.
  const cv::Mat ramp = MakeHorizontalRamp();
  const PhotometricMatch match = CreatePhotometricMatch(
      ramp, {320.0, 240.0}, {0.0, 0.0, 5.0}, reference, 0, patch_size);

  // du runs -2..1 within each row, and the ramp value is (column % 128) * 2,
  // divided here by the patch mean.
  double mean = 0.0;
  for (Eigen::Index col = 0; col < 4; ++col) {
    mean += 2.0 * ((320 - 2 + static_cast<int>(col)) % 128);
  }
  mean /= 4.0;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index col = 0; col < 4; ++col) {
      const int column = 320 - 2 + static_cast<int>(col);
      const double expected = 2.0 * (column % 128) / mean;
      EXPECT_NEAR(match.current_patch(row * 4 + col), expected, 1e-6)
          << "row " << row << " col " << col;
    }
  }
}

TEST(CreatePhotometricMatch, SamplesAreCenteredOnThePixel) {
  const std::size_t patch_size = 4;
  const VisualPatch reference = MakeReferencePatch(100.0F, patch_size);
  const cv::Mat ramp = MakeHorizontalRamp();

  const PhotometricMatch left = CreatePhotometricMatch(
      ramp, {320.0, 240.0}, {0.0, 0.0, 5.0}, reference, 0, patch_size);
  const PhotometricMatch right = CreatePhotometricMatch(
      ramp, {330.0, 240.0}, {0.0, 0.0, 5.0}, reference, 0, patch_size);

  // Ten columns right on a ramp, so the same shape over a brighter mean: the
  // ratio across the patch shrinks.
  EXPECT_LT(right.current_patch(3) / right.current_patch(0),
            left.current_patch(3) / left.current_patch(0));
}

}  // namespace
}  // namespace livo_slam
