#include "ros/conversions.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace livo_slam {
namespace {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField MakeField(const std::string& name, std::uint32_t offset,
                     std::uint8_t datatype) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

template <typename T>
void WriteAt(PointCloud2& msg, std::size_t index, std::uint32_t offset,
             T value) {
  std::memcpy(msg.data.data() + index * msg.point_step + offset, &value,
              sizeof(T));
}

// x/y/z at 0/4/8 and intensity at 16 in both layouts, matching the real
// Ouster and Hesai clouds.
PointCloud2 MakeCloud(bool ouster, std::uint32_t points, double header_time) {
  PointCloud2 msg;
  msg.header.stamp.sec = static_cast<std::int32_t>(header_time);
  msg.header.stamp.nanosec = static_cast<std::uint32_t>(
      (header_time - static_cast<std::int32_t>(header_time)) * 1e9);
  msg.height = 1;
  msg.width = points;
  msg.point_step = ouster ? 32 : 48;
  msg.fields = {MakeField("x", 0, PointField::FLOAT32),
                MakeField("y", 4, PointField::FLOAT32),
                MakeField("z", 8, PointField::FLOAT32),
                MakeField("intensity", 16, PointField::FLOAT32)};
  msg.fields.push_back(ouster
                           ? MakeField("t", 20, PointField::UINT32)
                           : MakeField("timestamp", 24, PointField::FLOAT64));
  msg.data.assign(static_cast<std::size_t>(points) * msg.point_step, 0);

  for (std::uint32_t i = 0; i < points; ++i) {
    WriteAt<float>(msg, i, 0, 1.0F + static_cast<float>(i));
    WriteAt<float>(msg, i, 4, 2.0F);
    WriteAt<float>(msg, i, 8, 3.0F);
    WriteAt<float>(msg, i, 16, 10.0F * static_cast<float>(i));
  }
  return msg;
}

TEST(ToPoints, OusterTimesAreOffsetsFromTheHeader) {
  PointCloud2 msg = MakeCloud(true, 2, 1000.0);
  WriteAt<std::uint32_t>(msg, 0, 20, 0);
  WriteAt<std::uint32_t>(msg, 1, 20, 50'000'000);  // 50 ms

  const auto points = ToPoints(msg, 0.0);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].timestamp, 1000.0);
  EXPECT_DOUBLE_EQ(points[1].timestamp, 1000.05);
}

TEST(ToPoints, HesaiTimesAreAbsoluteAndIgnoreTheHeader) {
  PointCloud2 msg = MakeCloud(false, 2, 1000.0);
  WriteAt<double>(msg, 0, 24, 2000.0);
  WriteAt<double>(msg, 1, 24, 2000.05);

  const auto points = ToPoints(msg, 0.0);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].timestamp, 2000.0);
  EXPECT_DOUBLE_EQ(points[1].timestamp, 2000.05);
}

TEST(ToPoints, TimeShiftAppliesToBothLayouts) {
  PointCloud2 ouster = MakeCloud(true, 1, 1000.0);
  WriteAt<std::uint32_t>(ouster, 0, 20, 0);
  PointCloud2 hesai = MakeCloud(false, 1, 1000.0);
  WriteAt<double>(hesai, 0, 24, 2000.0);

  EXPECT_DOUBLE_EQ(ToPoints(ouster, -0.1)[0].timestamp, 999.9);
  EXPECT_DOUBLE_EQ(ToPoints(hesai, -0.1)[0].timestamp, 1999.9);
}

TEST(ToPoints, ReadsPositionAndIntensityInBothLayouts) {
  for (const bool ouster : {true, false}) {
    PointCloud2 msg = MakeCloud(ouster, 2, 1000.0);

    const auto points = ToPoints(msg, 0.0);

    ASSERT_EQ(points.size(), 2U) << "ouster=" << ouster;
    EXPECT_EQ(points[0].position, Eigen::Vector3d(1.0, 2.0, 3.0));
    EXPECT_EQ(points[1].position, Eigen::Vector3d(2.0, 2.0, 3.0));
    EXPECT_FLOAT_EQ(points[1].intensity, 10.0F);
  }
}

TEST(ToPoints, ThrowsWhenNeitherTimeFieldIsPresent) {
  PointCloud2 msg = MakeCloud(true, 1, 1000.0);
  msg.fields.pop_back();  // drop 't'

  EXPECT_THROW(ToPoints(msg, 0.0), std::runtime_error);
}

TEST(ToPoints, SkipsZeroAndNonFinitePoints) {
  PointCloud2 msg = MakeCloud(false, 3, 1000.0);
  WriteAt<double>(msg, 0, 24, 2000.0);
  WriteAt<double>(msg, 1, 24, 2000.0);
  WriteAt<double>(msg, 2, 24, 2000.0);
  WriteAt<float>(msg, 0, 0, 0.0F);
  WriteAt<float>(msg, 0, 4, 0.0F);
  WriteAt<float>(msg, 0, 8, 0.0F);
  WriteAt<float>(msg, 1, 0, std::numeric_limits<float>::quiet_NaN());

  EXPECT_EQ(ToPoints(msg, 0.0).size(), 1U);
}

}  // namespace
}  // namespace livo_slam
