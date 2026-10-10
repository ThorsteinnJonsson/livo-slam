#pragma once

#include <opencv2/core/mat.hpp>

namespace livo_slam {

struct ImageFrame {
  double timestamp = 0.0;
  cv::Mat image;  // single channel, 8 bit
  cv::Mat color;  // bgr8, empty unless the visualization asked for it
};

}  // namespace livo_slam
