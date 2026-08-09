#include "core/pipeline/measurement_buffer.h"

#include <gtest/gtest.h>

#include <thread>
#include <vector>

namespace livo_slam {
namespace {

std::vector<Point> MakePoints(double start, double end, std::size_t count) {
  std::vector<Point> points;
  points.reserve(count);
  const double step = (end - start) / static_cast<double>(count);
  for (std::size_t i = 0; i < count; ++i) {
    Point point;
    point.timestamp = start + step * static_cast<double>(i);
    point.position = Eigen::Vector3d(point.timestamp, 0.0, 0.0);
    points.push_back(point);
  }
  return points;
}

void PushImuRange(MeasurementBuffer& buffer, double start, double end,
                  double step) {
  for (double t = start; t <= end; t += step) {
    ImuSample sample;
    sample.timestamp = t;
    buffer.PushImu(sample);
  }
}

ImageFrame MakeImage(double timestamp) {
  ImageFrame image;
  image.timestamp = timestamp;
  image.width = 4;
  image.height = 2;
  return image;
}

TEST(MeasurementBuffer, ShutdownUnblocksPop) {
  MeasurementBuffer buffer;
  buffer.Shutdown();
  EXPECT_FALSE(buffer.PopMeasureGroup().has_value());
}

TEST(MeasurementBuffer, ShutdownWakesWaitingConsumer) {
  MeasurementBuffer buffer;
  std::thread waiter(
      [&buffer] { EXPECT_FALSE(buffer.PopMeasureGroup().has_value()); });
  buffer.Shutdown();
  waiter.join();
}

TEST(MeasurementBuffer, CutsAtImageTimestamp) {
  MeasurementBuffer buffer;
  buffer.PushPoints(MakePoints(0.0, 0.2, 200));
  PushImuRange(buffer, 0.0, 0.2, 0.01);
  buffer.PushImage(MakeImage(0.1));

  const auto group = buffer.PopMeasureGroup();
  ASSERT_TRUE(group.has_value());

  EXPECT_DOUBLE_EQ(group->cloud.end_time, 0.1);
  EXPECT_DOUBLE_EQ(group->image.timestamp, 0.1);
  for (const Point& point : group->cloud.points) {
    EXPECT_LE(point.timestamp, 0.1);
  }
  for (const ImuSample& sample : group->imu) {
    EXPECT_LE(sample.timestamp, 0.1);
  }
}

TEST(MeasurementBuffer, ConsecutiveGroupsAreContiguous) {
  MeasurementBuffer buffer;
  buffer.PushPoints(MakePoints(0.0, 0.3, 300));
  PushImuRange(buffer, 0.0, 0.3, 0.01);
  buffer.PushImage(MakeImage(0.1));
  buffer.PushImage(MakeImage(0.2));

  const auto first = buffer.PopMeasureGroup();
  const auto second = buffer.PopMeasureGroup();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_DOUBLE_EQ(second->cloud.start_time, first->cloud.end_time);
  EXPECT_DOUBLE_EQ(second->cloud.end_time, 0.2);
  for (const Point& point : second->cloud.points) {
    EXPECT_GT(point.timestamp, 0.1);
  }
}

TEST(MeasurementBuffer, WaitsForCoverageBeyondImageTime) {
  MeasurementBuffer buffer;
  buffer.PushPoints(MakePoints(0.0, 0.05, 50));
  PushImuRange(buffer, 0.0, 0.05, 0.01);
  buffer.PushImage(MakeImage(0.1));

  std::optional<MeasureGroup> group;
  std::thread waiter([&] { group = buffer.PopMeasureGroup(); });

  buffer.PushPoints(MakePoints(0.05, 0.2, 150));
  PushImuRange(buffer, 0.06, 0.2, 0.01);
  waiter.join();

  ASSERT_TRUE(group.has_value());
  EXPECT_DOUBLE_EQ(group->cloud.end_time, 0.1);
}

TEST(MeasurementBuffer, OverlappingScansStayTimeOrdered) {
  MeasurementBuffer buffer;
  buffer.PushPoints(MakePoints(0.0, 0.1, 100));
  buffer.PushPoints(MakePoints(0.07, 0.17, 100));
  buffer.PushPoints(MakePoints(0.17, 0.25, 80));
  PushImuRange(buffer, 0.0, 0.25, 0.01);
  buffer.PushImage(MakeImage(0.12));

  const auto group = buffer.PopMeasureGroup();
  ASSERT_TRUE(group.has_value());
  ASSERT_FALSE(group->cloud.points.empty());

  for (std::size_t i = 1; i < group->cloud.points.size(); ++i) {
    EXPECT_LE(group->cloud.points[i - 1].timestamp,
              group->cloud.points[i].timestamp);
  }
  EXPECT_LE(group->cloud.points.back().timestamp, 0.12);
}

TEST(MeasurementBuffer, OverlappingScansKeepAllPoints) {
  const auto first_scan = MakePoints(0.0, 0.1, 100);
  const auto second_scan = MakePoints(0.07, 0.2, 130);
  const double cut = 0.17;

  std::size_t expected = 0;
  for (const auto& scan : {first_scan, second_scan}) {
    for (const Point& point : scan) {
      if (point.timestamp <= cut) {
        ++expected;
      }
    }
  }

  MeasurementBuffer buffer;
  buffer.PushPoints(first_scan);
  buffer.PushPoints(second_scan);
  PushImuRange(buffer, 0.0, 0.2, 0.01);
  buffer.PushImage(MakeImage(cut));

  const auto group = buffer.PopMeasureGroup();
  ASSERT_TRUE(group.has_value());
  EXPECT_EQ(group->cloud.points.size(), expected);
}

}  // namespace
}  // namespace livo_slam
