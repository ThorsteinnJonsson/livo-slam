#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <opencv2/core.hpp>
#include <optional>

#include "core/config/camera_params.h"

namespace livo_slam {

namespace impl {
// Apply the lens distortion to a normalized image point (x/z, y/z).
Eigen::Vector2d Distort(const RadTanParams& params,
                        const Eigen::Vector2d& normalized);
Eigen::Vector2d Distort(const EquidistantParams& params,
                        const Eigen::Vector2d& normalized);

// Derivative of Distort with respect to the normalized point.
Eigen::Matrix2d DistortionJacobian(const RadTanParams& params,
                                   const Eigen::Vector2d& normalized);
Eigen::Matrix2d DistortionJacobian(const EquidistantParams& params,
                                   const Eigen::Vector2d& normalized);
}  // namespace impl

// Apply the configured lens distortion to a normalized image point.
Eigen::Vector2d Distort(const CameraParams& params,
                        const Eigen::Vector2d& normalized);

// Derivative of Distort with respect to the normalized point.
Eigen::Matrix2d DistortionJacobian(const CameraParams& params,
                                   const Eigen::Vector2d& normalized);

// Project 3D point (in camera frame) to pixel coordinates.
std::optional<Eigen::Vector2d> Project(const CameraParams& params,
                                       const Eigen::Vector3d& point_cam);

// Check if a given pixel is visible in the image. The margin should be used
// when checking patches so they are fully contained within the image (normally
// using patch_size / 2 + 1).
bool InFrame(const cv::Mat& image, const Eigen::Vector2d& pixel,
             std::size_t margin);

double SampleBilinear(const cv::Mat& image, const Eigen::Vector2d& pixel);

Eigen::Matrix<double, 2, 3> ProjectionJacobian(
    const CameraParams& params, const Eigen::Vector3d& point_cam);

}  // namespace livo_slam