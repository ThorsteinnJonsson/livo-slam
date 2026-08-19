#include "core/camera/projection.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

// The NTU VIRAL left camera.
CameraParams MakeParams() {
  CameraParams params;
  params.width = 752;
  params.height = 480;
  params.fx = 425.0258563372763;
  params.fy = 426.7976260903337;
  params.cx = 386.0151866550880;
  params.cy = 241.9130336743440;
  return params;
}

TEST(Project, PointOnTheOpticalAxisHitsThePrincipalPoint) {
  const CameraParams params = MakeParams();

  const auto pixel = Project(params, {0.0, 0.0, 5.0});

  ASSERT_TRUE(pixel.has_value());
  EXPECT_DOUBLE_EQ(pixel->x(), params.cx);
  EXPECT_DOUBLE_EQ(pixel->y(), params.cy);
}

TEST(Project, ScalesByFocalLengthAndDepth) {
  const CameraParams params = MakeParams();

  const auto pixel = Project(params, {1.0, 2.0, 4.0});

  ASSERT_TRUE(pixel.has_value());
  EXPECT_DOUBLE_EQ(pixel->x(), params.fx * 0.25 + params.cx);
  EXPECT_DOUBLE_EQ(pixel->y(), params.fy * 0.5 + params.cy);
}

TEST(Project, HalvingDepthDoublesTheOffsetFromTheCenter) {
  const CameraParams params = MakeParams();

  const auto near = Project(params, {1.0, 1.0, 2.0});
  const auto far = Project(params, {1.0, 1.0, 4.0});

  ASSERT_TRUE(near.has_value());
  ASSERT_TRUE(far.has_value());
  EXPECT_DOUBLE_EQ(near->x() - params.cx, 2.0 * (far->x() - params.cx));
  EXPECT_DOUBLE_EQ(near->y() - params.cy, 2.0 * (far->y() - params.cy));
}

TEST(Project, RejectsPointsBehindTheCamera) {
  const CameraParams params = MakeParams();

  EXPECT_FALSE(Project(params, {0.1, 0.1, -1.0}).has_value());
  EXPECT_FALSE(Project(params, {0.1, 0.1, 0.0}).has_value());
}

// Matches the camera resolution above.
cv::Mat MakeFrame() { return cv::Mat(480, 752, CV_8UC1, cv::Scalar(0)); }

TEST(InFrame, AcceptsThePrincipalPointWithoutMargin) {
  const CameraParams params = MakeParams();

  EXPECT_TRUE(InFrame(MakeFrame(), {params.cx, params.cy}, 0));
}

TEST(InFrame, RejectsOutsideTheImage) {
  const cv::Mat image = MakeFrame();

  EXPECT_FALSE(InFrame(image, {-1.0, 100.0}, 0));
  EXPECT_FALSE(InFrame(image, {100.0, -1.0}, 0));
  EXPECT_FALSE(InFrame(image, {752.0, 100.0}, 0));
  EXPECT_FALSE(InFrame(image, {100.0, 480.0}, 0));
}

TEST(InFrame, BoundsAreInclusiveBelowAndExclusiveAbove) {
  const cv::Mat image = MakeFrame();

  EXPECT_TRUE(InFrame(image, {0.0, 0.0}, 0));
  EXPECT_TRUE(InFrame(image, {751.9, 479.9}, 0));
  EXPECT_FALSE(InFrame(image, {752.0, 479.9}, 0));
  EXPECT_FALSE(InFrame(image, {751.9, 480.0}, 0));
}

TEST(InFrame, MarginShrinksTheAcceptedRegion) {
  const cv::Mat image = MakeFrame();

  EXPECT_TRUE(InFrame(image, {5.0, 5.0}, 5));
  EXPECT_FALSE(InFrame(image, {4.9, 5.0}, 5));
  EXPECT_TRUE(InFrame(image, {746.9, 474.9}, 5));
  EXPECT_FALSE(InFrame(image, {747.0, 474.9}, 5));
}

TEST(InFrame, FollowsTheImageRatherThanTheCameraResolution) {
  // A pyramid level is smaller than the full-resolution camera.
  const cv::Mat level(120, 188, CV_8UC1, cv::Scalar(0));

  EXPECT_TRUE(InFrame(level, {187.9, 119.9}, 0));
  EXPECT_FALSE(InFrame(level, {188.0, 60.0}, 0));
  EXPECT_FALSE(InFrame(level, {90.0, 120.0}, 0));
}

// (u=0,v=0)=10  (u=1,v=0)=20
// (u=0,v=1)=30  (u=1,v=1)=50
cv::Mat MakeImage() { return (cv::Mat_<std::uint8_t>(2, 2) << 10, 20, 30, 50); }

TEST(SampleBilinear, ReturnsThePixelAtIntegerCoordinates) {
  const cv::Mat image = MakeImage();

  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.0, 0.0}), 10.0);
  EXPECT_DOUBLE_EQ(SampleBilinear(image, {1.0, 0.0}), 20.0);
  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.0, 1.0}), 30.0);
}

TEST(SampleBilinear, InterpolatesAlongTheColumnDirection) {
  const cv::Mat image = MakeImage();

  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.5, 0.0}), 15.0);
  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.25, 0.0}), 12.5);
}

TEST(SampleBilinear, InterpolatesAlongTheRowDirection) {
  const cv::Mat image = MakeImage();

  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.0, 0.5}), 20.0);
  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.0, 0.25}), 15.0);
}

TEST(SampleBilinear, CenterIsTheMeanOfTheFourNeighbours) {
  const cv::Mat image = MakeImage();

  EXPECT_DOUBLE_EQ(SampleBilinear(image, {0.5, 0.5}), 27.5);
}

TEST(SampleBilinear, IsUniformOnAUniformImage) {
  const cv::Mat image(4, 4, CV_8UC1, cv::Scalar(77));

  EXPECT_DOUBLE_EQ(SampleBilinear(image, {1.3, 2.7}), 77.0);
}

TEST(ProjectionJacobian, MatchesFiniteDifferences) {
  const CameraParams params = MakeParams();
  const Eigen::Vector3d point(0.7, -1.3, 4.2);

  const Eigen::Matrix<double, 2, 3> analytic =
      ProjectionJacobian(params, point);

  const double eps = 1e-6;
  const Eigen::Vector2d base = *Project(params, point);
  Eigen::Matrix<double, 2, 3> numeric;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d perturbed = point;
    perturbed(axis) += eps;
    numeric.col(axis) = (*Project(params, perturbed) - base) / eps;
  }

  EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-4);
}

TEST(ProjectionJacobian, HasZeroCrossTerms) {
  const Eigen::Matrix<double, 2, 3> jacobian =
      ProjectionJacobian(MakeParams(), {0.7, -1.3, 4.2});

  EXPECT_DOUBLE_EQ(jacobian(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(jacobian(1, 0), 0.0);
}

TEST(ProjectionJacobian, SensitivityFallsOffWithDepth) {
  const CameraParams params = MakeParams();

  const Eigen::Matrix<double, 2, 3> near =
      ProjectionJacobian(params, {0.0, 0.0, 2.0});
  const Eigen::Matrix<double, 2, 3> far =
      ProjectionJacobian(params, {0.0, 0.0, 4.0});

  EXPECT_DOUBLE_EQ(near(0, 0), 2.0 * far(0, 0));
  EXPECT_DOUBLE_EQ(near(1, 1), 2.0 * far(1, 1));
}

TEST(ProjectionJacobian, DepthColumnOpposesTheOffsetFromTheCenter) {
  const CameraParams params = MakeParams();

  // Pushing a point further away pulls its projection back to the center.
  const Eigen::Matrix<double, 2, 3> jacobian =
      ProjectionJacobian(params, {1.0, -1.0, 4.0});

  EXPECT_LT(jacobian(0, 2), 0.0);
  EXPECT_GT(jacobian(1, 2), 0.0);
}

}  // namespace
}  // namespace livo_slam
