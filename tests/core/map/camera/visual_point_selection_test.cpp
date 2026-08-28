#include "core/map/camera/visual_point_selection.h"

#include <gtest/gtest.h>

#include <cstdint>

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

}  // namespace
}  // namespace livo_slam
