#include "undistort.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace livo_slam {
Undistorter::Undistorter(const CameraParams& params) {
  // clang-format off
  const cv::Mat camera_matrix =    
      (cv::Mat_<double>(3, 3) << params.fx, 0.0, params.cx, 
                                 0.0, params.fy, params.cy, 
                                 0.0, 0.0, 1.0);
  // clang-format on

  const cv::Mat distortion =
      (cv::Mat_<double>(1, 4) << params.k1, params.k2, params.p1, params.p2);

  cv::Mat rectification_transformation{};  // Empty (identity) for single-camera

  cv::initUndistortRectifyMap(
      camera_matrix, distortion, rectification_transformation, camera_matrix,
      cv::Size(static_cast<int>(params.width), static_cast<int>(params.height)),
      CV_32FC1, map_x_, map_y_);
}

ImageFrame Undistorter::Undistort(const ImageFrame& image_frame) const {
  cv::Mat undistorted;
  cv::remap(image_frame.image, undistorted, map_x_, map_y_, cv::INTER_LINEAR);

  ImageFrame undistorted_frame{image_frame.timestamp, undistorted};
  return undistorted_frame;
}
}  // namespace livo_slam