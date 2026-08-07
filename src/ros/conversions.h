#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"

namespace livo_slam {

ImuSample ToImuSample(const sensor_msgs::msg::Imu& msg);

std::vector<Point> ToPoints(const sensor_msgs::msg::PointCloud2& msg);

ImageFrame ToImageFrame(const sensor_msgs::msg::Image& msg);

}  // namespace livo_slam
