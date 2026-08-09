#include "core/lidar/range_filter.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

Cloud MakeCloud(const std::vector<double>& ranges) {
  Cloud cloud;
  cloud.start_time = 1.0;
  cloud.end_time = 1.1;
  for (double range : ranges) {
    Point point;
    point.timestamp = 1.05;
    point.intensity = 3.0F;
    point.position = Eigen::Vector3d(range, 0.0, 0.0);
    cloud.points.push_back(point);
  }
  return cloud;
}

TEST(FilterByRange, DropsPointsOutsideTheRange) {
  const Cloud cloud = MakeCloud({0.1, 0.5, 1.5, 30.0, 59.0, 80.0});
  const Cloud out = FilterByRange(cloud, 1.0, 60.0);

  ASSERT_EQ(out.points.size(), 3U);
  EXPECT_DOUBLE_EQ(out.points[0].position.x(), 1.5);
  EXPECT_DOUBLE_EQ(out.points[1].position.x(), 30.0);
  EXPECT_DOUBLE_EQ(out.points[2].position.x(), 59.0);
}

TEST(FilterByRange, BoundsAreInclusive) {
  const Cloud cloud = MakeCloud({1.0, 60.0});
  const Cloud out = FilterByRange(cloud, 1.0, 60.0);
  EXPECT_EQ(out.points.size(), 2U);
}

TEST(FilterByRange, MeasuresDistanceNotAxis) {
  Cloud cloud;
  Point point;
  point.position = Eigen::Vector3d(3.0, 4.0, 0.0);
  cloud.points.push_back(point);

  EXPECT_EQ(FilterByRange(cloud, 1.0, 4.9).points.size(), 0U);
  EXPECT_EQ(FilterByRange(cloud, 1.0, 5.1).points.size(), 1U);
}

TEST(FilterByRange, PreservesScanTimesAndMetadata) {
  const Cloud cloud = MakeCloud({5.0});
  const Cloud out = FilterByRange(cloud, 1.0, 60.0);

  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_DOUBLE_EQ(out.start_time, cloud.start_time);
  EXPECT_DOUBLE_EQ(out.end_time, cloud.end_time);
  EXPECT_DOUBLE_EQ(out.points[0].timestamp, cloud.points[0].timestamp);
  EXPECT_FLOAT_EQ(out.points[0].intensity, cloud.points[0].intensity);
}

TEST(FilterByRange, EmptyCloudStaysEmpty) {
  EXPECT_TRUE(FilterByRange(Cloud{}, 1.0, 60.0).points.empty());
}

}  // namespace
}  // namespace livo_slam
