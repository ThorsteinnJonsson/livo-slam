#include "ros/conversions.h"

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

namespace livo_slam {
namespace {

double ToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

std::uint32_t FieldOffset(const sensor_msgs::msg::PointCloud2& msg,
                          std::string_view name, std::uint8_t datatype) {
  for (const auto& field : msg.fields) {
    if (field.name != name) {
      continue;
    }
    if (field.datatype != datatype) {
      throw std::runtime_error("point field '" + std::string(name) +
                               "' has datatype " +
                               std::to_string(field.datatype) + ", expected " +
                               std::to_string(datatype));
    }
    return field.offset;
  }
  throw std::runtime_error("point cloud has no field '" + std::string(name) +
                           "'");
}

template <typename T>
T ReadAt(const std::uint8_t* base, std::uint32_t offset) {
  T value{};
  std::memcpy(&value, base + offset, sizeof(T));
  return value;
}

}  // namespace

Sophus::SE3d ToSe3(const std::vector<double>& row_major) {
  if (row_major.size() != 16) {
    return Sophus::SE3d{};
  }

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = row_major[static_cast<std::size_t>(row * 4 + col)];
    }
    translation(row) = row_major[static_cast<std::size_t>(row * 4 + 3)];
  }

  return Sophus::SE3d(Sophus::SO3d(Eigen::Quaterniond(rotation).normalized()),
                      translation);
}

ImuSample ToImuSample(const sensor_msgs::msg::Imu& msg) {
  ImuSample sample;
  sample.timestamp = ToSeconds(msg.header.stamp);
  sample.linear_acceleration =
      Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z);
  sample.angular_velocity = Eigen::Vector3d(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  return sample;
}

std::vector<Point> ToPoints(const sensor_msgs::msg::PointCloud2& msg) {
  using sensor_msgs::msg::PointField;

  const std::uint32_t x_offset = FieldOffset(msg, "x", PointField::FLOAT32);
  const std::uint32_t y_offset = FieldOffset(msg, "y", PointField::FLOAT32);
  const std::uint32_t z_offset = FieldOffset(msg, "z", PointField::FLOAT32);
  const std::uint32_t intensity_offset =
      FieldOffset(msg, "intensity", PointField::FLOAT32);
  const std::uint32_t time_offset = FieldOffset(msg, "t", PointField::UINT32);

  const double base_time = ToSeconds(msg.header.stamp);

  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(msg.width) * msg.height);

  for (std::uint32_t column = 0; column < msg.width; ++column) {
    for (std::uint32_t row = 0; row < msg.height; ++row) {
      const std::size_t index =
          static_cast<std::size_t>(row) * msg.width + column;
      const std::uint8_t* raw = msg.data.data() + index * msg.point_step;

      const float x = ReadAt<float>(raw, x_offset);
      const float y = ReadAt<float>(raw, y_offset);
      const float z = ReadAt<float>(raw, z_offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (x == 0.0F && y == 0.0F && z == 0.0F) {
        continue;
      }

      Point point;
      point.position = Eigen::Vector3d(x, y, z);
      point.intensity = ReadAt<float>(raw, intensity_offset);
      point.timestamp =
          base_time +
          static_cast<double>(ReadAt<std::uint32_t>(raw, time_offset)) * 1e-9;
      points.push_back(point);
    }
  }
  return points;
}

ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg) {
  ImageFrame frame;
  frame.timestamp = ToSeconds(msg.header.stamp);
  frame.width = msg.width;
  frame.height = msg.height;
  frame.encoding = msg.encoding;
  frame.data = msg.data;
  return frame;
}

}  // namespace livo_slam
