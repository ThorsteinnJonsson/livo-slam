#pragma once

#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>
#include <vector>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"

namespace livo_slam {

Sophus::SE3d ToSe3(const std::vector<double>& row_major);

// Some IMUs report acceleration in g rather than m/s^2, so the caller passes
// the factor that converts.
ImuSample ToImuSample(const sensor_msgs::msg::Imu& msg,
                      double accel_scale = 1.0);

std::vector<Point> ToPoints(const sensor_msgs::msg::PointCloud2& msg,
                            double time_shift);

// Livox publishes its own CustomMsg, whose definition we do not have, so the
// CDR payload is decoded by hand. Throws if the layout does not add up.
std::vector<Point> ToPointsFromLivox(
    const rclcpp::SerializedMessage& serialized, double time_shift);

ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg, double time_shift);

ImageFrame ToImageFrame(const sensor_msgs::msg::CompressedImage& msg,
                        double time_shift);

}  // namespace livo_slam
