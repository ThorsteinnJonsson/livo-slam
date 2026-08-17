#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"

namespace livo_slam {

class Undistorter {
 public:
  explicit Undistorter(const CameraParams& params);

  Undistorter(const Undistorter&) = delete;
  Undistorter& operator=(const Undistorter&) = delete;
  Undistorter(Undistorter&&) = delete;
  Undistorter& operator=(Undistorter&&) = delete;

  cv::Mat Undistort(const cv::Mat& image) const;

 private:
  cv::Mat map_x_;
  cv::Mat map_y_;
};
}  // namespace livo_slam