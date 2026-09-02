#include "ros/conversions.h"

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cv_bridge/cv_bridge.hpp>
#include <optional>
#include <print>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

namespace livo_slam {
namespace {

double ToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

std::optional<std::uint32_t> FindField(const sensor_msgs::msg::PointCloud2& msg,
                                       std::string_view name,
                                       std::uint8_t datatype) {
  for (const auto& field : msg.fields) {
    if (field.name == name && field.datatype == datatype) {
      return field.offset;
    }
  }
  return std::nullopt;
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

std::vector<Point> ToPoints(const sensor_msgs::msg::PointCloud2& msg,
                            double time_shift) {
  using sensor_msgs::msg::PointField;

  const std::uint32_t x_offset = FieldOffset(msg, "x", PointField::FLOAT32);
  const std::uint32_t y_offset = FieldOffset(msg, "y", PointField::FLOAT32);
  const std::uint32_t z_offset = FieldOffset(msg, "z", PointField::FLOAT32);
  const std::uint32_t intensity_offset =
      FieldOffset(msg, "intensity", PointField::FLOAT32);

  // Ouster stamps 't' in nanoseconds from the header, Hesai stamps 'timestamp'
  // in absolute seconds.
  const auto offset_time = FindField(msg, "t", PointField::UINT32);
  const auto absolute_time = FindField(msg, "timestamp", PointField::FLOAT64);
  if (!offset_time && !absolute_time) {
    throw std::runtime_error(
        "point cloud has neither a 't' (uint32) nor a 'timestamp' (float64) "
        "field");
  }

  const double base_time = ToSeconds(msg.header.stamp) + time_shift;

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
          offset_time ? base_time + static_cast<double>(ReadAt<std::uint32_t>(
                                        raw, *offset_time)) *
                                        1e-9
                      : ReadAt<double>(raw, *absolute_time) + time_shift;
      points.push_back(point);
    }
  }
  return points;
}

ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg, double time_shift) {
  ImageFrame frame;
  frame.timestamp = ToSeconds(msg.header.stamp) + time_shift;
  try {
    frame.image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
  } catch (const cv_bridge::Exception& error) {
    std::println("cannot convert {} to mono8: {}", msg.encoding, error.what());
  }
  return frame;
}

}  // namespace livo_slam
