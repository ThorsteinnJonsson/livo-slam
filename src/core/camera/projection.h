#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <optional>

#include "core/config/camera_params.h"

namespace livo_slam {

// Project 3D point (in camera frame) to pixel coordinates.
std::optional<Eigen::Vector2d> Project(const CameraParams& params,
                                       const Eigen::Vector3d& point_cam);

// Check if a given pixel is visible in the image. The margin should be used
// when checking patches so they are fully contained within the image (normally
// using patch_size / 2 + 1).
bool InFrame(const CameraParams& params, const Eigen::Vector2d& pixel,
             int margin);

double SampleBilinear(const cv::Mat& image, const Eigen::Vector2d& pixel);

Eigen::Matrix<double, 2, 3> ProjectionJacobian(
    const CameraParams& params, const Eigen::Vector3d& point_cam);

}  // namespace livo_slam