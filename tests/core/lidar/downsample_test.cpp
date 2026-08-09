#include "core/lidar/downsample.h"

#include <gtest/gtest.h>

#include <algorithm>

namespace livo_slam {
namespace {

Point MakePoint(const Eigen::Vector3d& position, double timestamp = 0.0,
                float intensity = 0.0F) {
  Point point;
  point.position = position;
  point.timestamp = timestamp;
  point.intensity = intensity;
  return point;
}

TEST(VoxelDownsample, MergesPointsSharingAVoxelIntoTheirCentroid) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.1, 0.1, 0.1}));
  cloud.points.push_back(MakePoint({0.3, 0.3, 0.3}));

  const Cloud out = VoxelDownsample(cloud, 1.0);

  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_LT((out.points[0].position - Eigen::Vector3d(0.2, 0.2, 0.2)).norm(),
            1e-12);
}

TEST(VoxelDownsample, KeepsPointsInSeparateVoxels) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.5, 0.5, 0.5}));
  cloud.points.push_back(MakePoint({1.5, 0.5, 0.5}));
  cloud.points.push_back(MakePoint({0.5, 1.5, 0.5}));

  EXPECT_EQ(VoxelDownsample(cloud, 1.0).points.size(), 3U);
}

TEST(VoxelDownsample, FloorsNegativeCoordinates) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({-1.4, 0.0, 0.0}));
  cloud.points.push_back(MakePoint({-0.6, 0.0, 0.0}));
  cloud.points.push_back(MakePoint({-0.4, 0.0, 0.0}));
  cloud.points.push_back(MakePoint({0.4, 0.0, 0.0}));

  const Cloud out = VoxelDownsample(cloud, 1.0);

  std::vector<double> centroids;
  for (const Point& point : out.points) {
    centroids.push_back(point.position.x());
  }
  std::sort(centroids.begin(), centroids.end());

  ASSERT_EQ(centroids.size(), 3U);
  EXPECT_DOUBLE_EQ(centroids[0], -1.4);
  EXPECT_DOUBLE_EQ(centroids[1], -0.5);
  EXPECT_DOUBLE_EQ(centroids[2], 0.4);
}

TEST(VoxelDownsample, AveragesIntensity) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.1, 0.0, 0.0}, 0.0, 10.0F));
  cloud.points.push_back(MakePoint({0.2, 0.0, 0.0}, 0.0, 20.0F));

  const Cloud out = VoxelDownsample(cloud, 1.0);

  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_FLOAT_EQ(out.points[0].intensity, 15.0F);
}

TEST(VoxelDownsample, KeepsEarliestTimestampInVoxel) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.1, 0.0, 0.0}, 5.0));
  cloud.points.push_back(MakePoint({0.2, 0.0, 0.0}, 3.0));
  cloud.points.push_back(MakePoint({0.3, 0.0, 0.0}, 7.0));

  const Cloud out = VoxelDownsample(cloud, 1.0);

  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_DOUBLE_EQ(out.points[0].timestamp, 3.0);
}

TEST(VoxelDownsample, SkipsNonFinitePoints) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.1, 0.0, 0.0}));
  cloud.points.push_back(
      MakePoint({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));

  EXPECT_EQ(VoxelDownsample(cloud, 1.0).points.size(), 1U);
}

TEST(VoxelDownsample, PreservesScanTimes) {
  Cloud cloud;
  cloud.start_time = 10.0;
  cloud.end_time = 10.1;
  cloud.points.push_back(MakePoint({0.1, 0.0, 0.0}));

  const Cloud out = VoxelDownsample(cloud, 1.0);

  EXPECT_DOUBLE_EQ(out.start_time, 10.0);
  EXPECT_DOUBLE_EQ(out.end_time, 10.1);
}

TEST(VoxelDownsample, NonPositiveLeafReturnsInput) {
  Cloud cloud;
  cloud.points.push_back(MakePoint({0.1, 0.0, 0.0}));
  cloud.points.push_back(MakePoint({0.2, 0.0, 0.0}));

  EXPECT_EQ(VoxelDownsample(cloud, 0.0).points.size(), 2U);
}

TEST(VoxelDownsample, SmallerLeafKeepsMorePoints) {
  Cloud cloud;
  for (int i = 0; i < 40; ++i) {
    cloud.points.push_back(MakePoint({0.1 * static_cast<double>(i), 0.0, 0.0}));
  }

  const std::size_t coarse = VoxelDownsample(cloud, 1.0).points.size();
  const std::size_t fine = VoxelDownsample(cloud, 0.2).points.size();

  EXPECT_LT(coarse, fine);
  EXPECT_LE(fine, cloud.points.size());
}

}  // namespace
}  // namespace livo_slam
