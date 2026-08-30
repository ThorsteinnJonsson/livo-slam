#include "core/map/camera/visual_point_selection.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace livo_slam {
namespace {

constexpr int kImageSize = 64;

// Period-4 square wave. Over any 8 consecutive samples the central differences
// sum to zero and their squares sum to 8, which keeps the structure tensor
// diagonal and makes the expected score exact.
int SquareWave(int i) { return i % 4 < 2 ? 1 : 0; }

// Intensity is separable: amplitude_x drives the horizontal gradient and
// amplitude_y the vertical one, so the score is min(amplitude)^2 / 2.
cv::Mat MakeTexture(int amplitude_x, int amplitude_y, int offset = 0,
                    int textured_rows = kImageSize) {
  cv::Mat image(kImageSize, kImageSize, CV_8UC1, cv::Scalar(offset));
  for (int r = 0; r < textured_rows; ++r) {
    for (int c = 0; c < kImageSize; ++c) {
      image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(
          offset + amplitude_x * SquareWave(c) + amplitude_y * SquareWave(r));
    }
  }
  return image;
}

TEST(CornerScore, UniformImageHasNoCorner) {
  const cv::Mat image(kImageSize, kImageSize, CV_8UC1, cv::Scalar(128));

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 0.0);
}

TEST(CornerScore, LinearRampHasNoCorner) {
  cv::Mat image(kImageSize, kImageSize, CV_8UC1);
  for (int r = 0; r < kImageSize; ++r) {
    for (int c = 0; c < kImageSize; ++c) {
      image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(r + c);
    }
  }

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 0.0);
}

TEST(CornerScore, VerticalStripesHaveNoCorner) {
  const cv::Mat image = MakeTexture(100, 0);

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 0.0);
}

TEST(CornerScore, HorizontalStripesHaveNoCorner) {
  const cv::Mat image = MakeTexture(0, 100);

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 0.0);
}

TEST(CornerScore, TextureInBothDirectionsScoresTheSmallerEigenvalue) {
  const cv::Mat image = MakeTexture(100, 100);

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 100.0 * 100.0 / 2.0);
}

TEST(CornerScore, ScoreIsSetByTheWeakerDirection) {
  const cv::Mat image = MakeTexture(100, 40);

  EXPECT_DOUBLE_EQ(CornerScore(image, {32.0, 32.0}), 40.0 * 40.0 / 2.0);
}

TEST(CornerScore, HigherContrastScoresHigher) {
  const cv::Mat weak = MakeTexture(20, 20);
  const cv::Mat strong = MakeTexture(80, 80);

  EXPECT_GT(CornerScore(strong, {32.0, 32.0}), CornerScore(weak, {32.0, 32.0}));
}

TEST(CornerScore, IsUnaffectedByABrightnessOffset) {
  const cv::Mat dark = MakeTexture(60, 60);
  const cv::Mat bright = MakeTexture(60, 60, 100);

  EXPECT_DOUBLE_EQ(CornerScore(bright, {32.0, 32.0}),
                   CornerScore(dark, {32.0, 32.0}));
}

TEST(CornerScore, PixelXIsTheColumnAndPixelYIsTheRow) {
  // Only the top half is textured.
  const cv::Mat image = MakeTexture(100, 100, 0, kImageSize / 2);

  EXPECT_DOUBLE_EQ(CornerScore(image, {48.0, 16.0}), 100.0 * 100.0 / 2.0);
  EXPECT_DOUBLE_EQ(CornerScore(image, {16.0, 48.0}), 0.0);
}


// --- CreateVisualPatch -------------------------------------------------------

constexpr int kPatchSize = 8;

// A pyramid whose levels are independent images, so a patch built from the
// wrong level is visible in the sampled values.
std::vector<cv::Mat> MakeConstantPyramid(int level0, int level1, int level2) {
  return {cv::Mat(128, 128, CV_8UC1, cv::Scalar(level0)),
          cv::Mat(64, 64, CV_8UC1, cv::Scalar(level1)),
          cv::Mat(32, 32, CV_8UC1, cv::Scalar(level2))};
}

// Intensity equals the column index, so a sampled value reports where it came
// from along x. Sized for one pyramid level.
cv::Mat MakeColumnRamp(int size) {
  cv::Mat image(size, size, CV_8UC1);
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(c);
    }
  }
  return image;
}

cv::Mat MakeRowRamp(int size) {
  cv::Mat image(size, size, CV_8UC1);
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(r);
    }
  }
  return image;
}

TEST(CreateVisualPatch, FillsEveryPyramidLevel) {
  const std::vector<cv::Mat> pyramid = MakeConstantPyramid(10, 20, 30);

  const VisualPatch patch = CreateVisualPatch(pyramid, {100.0, 60.0},
                                              Sophus::SE3d{}, 1.0, 1.0,
                                              kPatchSize);

  for (const Eigen::VectorXf& level : patch.intensities) {
    EXPECT_EQ(level.size(), kPatchSize * kPatchSize);
  }
}

TEST(CreateVisualPatch, EachLevelSamplesItsOwnImage) {
  const std::vector<cv::Mat> pyramid = MakeConstantPyramid(10, 20, 30);

  const VisualPatch patch = CreateVisualPatch(pyramid, {100.0, 60.0},
                                              Sophus::SE3d{}, 1.0, 1.0,
                                              kPatchSize);

  EXPECT_FLOAT_EQ(patch.intensities[0].maxCoeff(), 10.0F);
  EXPECT_FLOAT_EQ(patch.intensities[0].minCoeff(), 10.0F);
  EXPECT_FLOAT_EQ(patch.intensities[1].maxCoeff(), 20.0F);
  EXPECT_FLOAT_EQ(patch.intensities[2].maxCoeff(), 30.0F);
}

TEST(CreateVisualPatch, HalvesThePixelForEachLevel) {
  // Every level is a column ramp, so the sampled value is the column sampled.
  const std::vector<cv::Mat> pyramid = {MakeColumnRamp(128), MakeColumnRamp(64),
                                        MakeColumnRamp(32)};

  const VisualPatch patch = CreateVisualPatch(pyramid, {100.0, 60.0},
                                              Sophus::SE3d{}, 1.0, 1.0,
                                              kPatchSize);

  // The patch spans [center - 4, center + 3] at each level.
  EXPECT_FLOAT_EQ(patch.intensities[0](0), 100.0F - 4.0F);
  EXPECT_FLOAT_EQ(patch.intensities[1](0), 50.0F - 4.0F);
  EXPECT_FLOAT_EQ(patch.intensities[2](0), 25.0F - 4.0F);
}

TEST(CreateVisualPatch, IsRowMajorWithTheColumnVaryingFastest) {
  const std::vector<cv::Mat> columns = {MakeColumnRamp(128), MakeColumnRamp(64),
                                        MakeColumnRamp(32)};
  const std::vector<cv::Mat> rows = {MakeRowRamp(128), MakeRowRamp(64),
                                     MakeRowRamp(32)};

  const VisualPatch by_column = CreateVisualPatch(
      columns, {100.0, 60.0}, Sophus::SE3d{}, 1.0, 1.0, kPatchSize);
  const VisualPatch by_row = CreateVisualPatch(rows, {100.0, 60.0},
                                               Sophus::SE3d{}, 1.0, 1.0,
                                               kPatchSize);

  // Index 7 is the end of the first row, index 8 the start of the second.
  EXPECT_FLOAT_EQ(by_column.intensities[0](7), 100.0F + 3.0F);
  EXPECT_FLOAT_EQ(by_column.intensities[0](8), 100.0F - 4.0F);
  EXPECT_FLOAT_EQ(by_row.intensities[0](7), 60.0F - 4.0F);
  EXPECT_FLOAT_EQ(by_row.intensities[0](8), 60.0F - 3.0F);
}

TEST(CreateVisualPatch, StoresTheFullResolutionPixelAndTheStateItWasTakenWith) {
  const std::vector<cv::Mat> pyramid = MakeConstantPyramid(10, 20, 30);
  const Sophus::SE3d T_world_camera(Sophus::SO3d::rotX(0.3),
                                    Eigen::Vector3d(1.0, 2.0, 3.0));

  const VisualPatch patch = CreateVisualPatch(
      pyramid, {100.0, 60.0}, T_world_camera, 123.5, 0.25, kPatchSize);

  EXPECT_EQ(patch.pixel, Eigen::Vector2d(100.0, 60.0));
  EXPECT_DOUBLE_EQ(patch.timestamp, 123.5);
  EXPECT_DOUBLE_EQ(patch.inv_exposure, 0.25);
  EXPECT_TRUE(patch.T_world_camera.matrix().isApprox(T_world_camera.matrix()));
}

// --- SelectVisualPoints ------------------------------------------------------

constexpr int kSceneSize = 128;
constexpr std::size_t kSceneGridSize = 32;

// Identity extrinsics, so a world point is also a camera point.
CameraParams MakeSceneCamera() {
  CameraParams params;
  params.width = kSceneSize;
  params.height = kSceneSize;
  params.fx = 128.0;
  params.fy = 128.0;
  params.cx = 64.0;
  params.cy = 64.0;
  return params;
}

CameraUpdateParams MakeSceneUpdateParams() {
  CameraUpdateParams params;
  params.patch_size = kPatchSize;
  return params;
}

VisualMapParams MakeSceneMapParams() {
  VisualMapParams params;
  params.grid_size = kSceneGridSize;
  return params;
}

// Textured everywhere, but columns [32, 48) are lower contrast and so score
// lower than the rest of the same cell.
cv::Mat MakeSceneImage() {
  cv::Mat image(kSceneSize, kSceneSize, CV_8UC1);
  for (int r = 0; r < kSceneSize; ++r) {
    for (int c = 0; c < kSceneSize; ++c) {
      const int amplitude = (c >= 32 && c < 48) ? 40 : 100;
      image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(
          amplitude * SquareWave(c) + amplitude * SquareWave(r));
    }
  }
  return image;
}

std::vector<cv::Mat> MakeScenePyramid() {
  std::vector<cv::Mat> pyramid;
  cv::buildPyramid(MakeSceneImage(), pyramid,
                   static_cast<int>(CameraUpdateParams::kPyramidLevels) - 1);
  return pyramid;
}

Eigen::Vector3d BackProject(const CameraParams& params,
                            const Eigen::Vector2d& pixel, double depth) {
  return {(pixel.x() - params.cx) * depth / params.fx,
          (pixel.y() - params.cy) * depth / params.fy, depth};
}

Point MakeCandidate(const Eigen::Vector2d& pixel, double depth) {
  Point point;
  point.position = BackProject(MakeSceneCamera(), pixel, depth);
  return point;
}

VisualPoint MakeExistingPoint(const Eigen::Vector2d& pixel, double depth,
                              double patch_timestamp,
                              const Eigen::Vector2d& patch_pixel) {
  VisualPoint point;
  point.position = BackProject(MakeSceneCamera(), pixel, depth);
  VisualPatch patch;
  patch.timestamp = patch_timestamp;
  patch.pixel = patch_pixel;
  point.patches.push_back(patch);
  return point;
}

VisualMapUpdate RunSelect(
    const std::vector<Point>& points,
    const std::vector<const VisualPoint*>& existing, double timestamp = 10.0,
    const VisualMapParams& map_params = MakeSceneMapParams()) {
  return SelectVisualPoints(points, MakeScenePyramid(), timestamp, State{},
                            MakeSceneCamera(), MakeSceneUpdateParams(),
                            map_params, existing);
}

TEST(SelectVisualPoints, CreatesOnePointPerTexturedCell) {
  const std::vector<Point> points = {MakeCandidate({56.0, 40.0}, 5.0),
                                     MakeCandidate({72.0, 72.0}, 5.0)};

  const VisualMapUpdate update = RunSelect(points, {});

  EXPECT_EQ(update.new_points.size(), 2);
  EXPECT_TRUE(update.new_patches.empty());
}

TEST(SelectVisualPoints, NewPointsCarryExactlyOnePatch) {
  const std::vector<Point> points = {MakeCandidate({72.0, 72.0}, 5.0)};

  const VisualMapUpdate update = RunSelect(points, {});

  ASSERT_EQ(update.new_points.size(), 1);
  EXPECT_EQ(update.new_points[0].patches.size(), 1);
  EXPECT_EQ(update.new_points[0].position, points[0].position);
}

TEST(SelectVisualPoints, KeepsOnlyTheHighestScoringCandidateInACell) {
  // Both land in the cell spanning [32, 64), the second on higher contrast.
  const Point low_contrast = MakeCandidate({40.0, 40.0}, 5.0);
  const Point high_contrast = MakeCandidate({56.0, 40.0}, 5.0);

  const VisualMapUpdate update = RunSelect({low_contrast, high_contrast}, {});

  ASSERT_EQ(update.new_points.size(), 1);
  EXPECT_EQ(update.new_points[0].position, high_contrast.position);
}

TEST(SelectVisualPoints, SkipsCandidatesBelowTheMinimumScore) {
  VisualMapParams map_params = MakeSceneMapParams();
  map_params.min_corner_score = 1.0e9;

  const VisualMapUpdate update =
      RunSelect({MakeCandidate({72.0, 72.0}, 5.0)}, {}, 10.0, map_params);

  EXPECT_TRUE(update.new_points.empty());
}

TEST(SelectVisualPoints, SkipsCellsAlreadyCoveredByAVisualPoint) {
  const VisualPoint existing =
      MakeExistingPoint({68.0, 68.0}, 5.0, 10.0, {68.0, 68.0});

  const VisualMapUpdate update =
      RunSelect({MakeCandidate({72.0, 72.0}, 5.0)}, {&existing});

  EXPECT_TRUE(update.new_points.empty());
}

TEST(SelectVisualPoints, IgnoresPointsBehindTheCamera) {
  Point behind;
  behind.position = {0.0, 0.0, -5.0};

  const VisualMapUpdate update = RunSelect({behind}, {});

  EXPECT_TRUE(update.new_points.empty());
}

TEST(SelectVisualPoints, IgnoresPointsInsideTheBorderMargin) {
  // The margin is (patch_size / 2 + 1) << (kPyramidLevels - 1) = 20 pixels.
  const VisualMapUpdate update =
      RunSelect({MakeCandidate({10.0, 10.0}, 5.0)}, {});

  EXPECT_TRUE(update.new_points.empty());
}

TEST(SelectVisualPoints, AddsAPatchOnceTheLastOneIsStale) {
  const VisualPoint existing =
      MakeExistingPoint({72.0, 72.0}, 5.0, 0.0, {72.0, 72.0});

  const VisualMapUpdate update = RunSelect({}, {&existing}, 10.0);

  ASSERT_EQ(update.new_patches.size(), 1);
  EXPECT_EQ(update.new_patches[0].first, existing.position);
  EXPECT_DOUBLE_EQ(update.new_patches[0].second.timestamp, 10.0);
}

TEST(SelectVisualPoints, AddsAPatchOnceTheProjectionHasMovedFarEnough) {
  // Recent, but the last patch was taken 73 pixels away.
  const VisualPoint existing =
      MakeExistingPoint({72.0, 72.0}, 5.0, 9.5, {20.0, 20.0});

  const VisualMapUpdate update = RunSelect({}, {&existing}, 10.0);

  EXPECT_EQ(update.new_patches.size(), 1);
}

TEST(SelectVisualPoints, DoesNotAddAPatchWhenRecentAndStill) {
  const VisualPoint existing =
      MakeExistingPoint({72.0, 72.0}, 5.0, 9.5, {72.0, 72.0});

  const VisualMapUpdate update = RunSelect({}, {&existing}, 10.0);

  EXPECT_TRUE(update.new_patches.empty());
}

TEST(SelectVisualPoints, PatchesTheNearestVisualPointInACell) {
  const VisualPoint far = MakeExistingPoint({68.0, 68.0}, 10.0, 0.0, {0.0, 0.0});
  const VisualPoint near = MakeExistingPoint({76.0, 76.0}, 3.0, 0.0, {0.0, 0.0});

  const VisualMapUpdate update = RunSelect({}, {&far, &near});

  ASSERT_EQ(update.new_patches.size(), 1);
  EXPECT_EQ(update.new_patches[0].first, near.position);
}

}  // namespace
}  // namespace livo_slam
