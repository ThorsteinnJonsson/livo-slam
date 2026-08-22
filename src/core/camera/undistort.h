#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/types/image_frame.h"

namespace livo_slam {

class Undistorter {
 public:
  explicit Undistorter(const CameraParams& params);

  Undistorter(const Undistorter&) = delete;
  Undistorter& operator=(const Undistorter&) = delete;
  Undistorter(Undistorter&&) = delete;
  Undistorter& operator=(Undistorter&&) = delete;

  ImageFrame Undistort(const ImageFrame& image_frame) const;

 private:
  cv::Mat map_x_;
  cv::Mat map_y_;
};
}  // namespace livo_slam