#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/estimator/iekf.h"
#include "core/types/cloud.h"
#include "core/types/visual_observation.h"
#include "core/types/visual_point.h"

namespace livo_slam {

struct PhotometricMatch {
  Eigen::Vector3d point_imu = Eigen::Vector3d::Zero();
  // Raw intensities. Exposure scales them in the residual.
  Eigen::VectorXf reference_patch;
  Eigen::VectorXf current_patch;
  Eigen::Matrix<double, Eigen::Dynamic, 2> gradients;  // dI/du per patch pixel
  double reference_inv_exposure = 1.0;
};

// A point this frame will use, with its reference patch and warp fixed so the
// iteration always sees the same set.
struct SelectedPoint {
  const VisualPoint* point = nullptr;
  const VisualPatch* reference_patch = nullptr;
  Eigen::Matrix2d warp = Eigen::Matrix2d::Identity();
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();  // Full resolution.
  double distance = 0.0;                            // Range from the camera.
};

const VisualPatch* SelectReferencePatch(const VisualPoint& visual_point,
                                        const Eigen::Vector3d& camera_position);

PhotometricMatch CreatePhotometricMatch(
    const cv::Mat& image, const Eigen::Vector2d& pixel,
    const Eigen::Vector3d& point_imu, const VisualPatch& reference_patch,
    std::size_t pyramid_level, std::size_t patch_size,
    const Eigen::Matrix2d& warp = Eigen::Matrix2d::Identity());

// Picks the points to use this frame, running every gate once.
std::vector<SelectedPoint> SelectCameraPoints(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params,
    std::vector<VisualObservation>* observations = nullptr,
    const cv::Mat& depth_image = cv::Mat());

// Samples the current image for points already picked.
std::vector<PhotometricMatch> BuildMatchesFromSelection(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<SelectedPoint>& selection,
    const CameraParams& camera_params, const CameraUpdateParams& update_params);

// What the current scan saw at each pixel, zero where it saw nothing. Lets us
// drop patches that straddle a depth jump.
cv::Mat BuildDepthImage(cv::Size size, const std::vector<Point>& points_imu,
                        const CameraParams& camera_params);

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state,
                             const CameraParams& camera_params,
                             double intensity_noise_std,
                             bool estimate_exposure = true);

}  // namespace livo_slam