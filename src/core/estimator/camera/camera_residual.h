#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/estimator/iekf.h"
#include "core/types/visual_point.h"

namespace livo_slam {

struct PhotometricMatch {
  Eigen::Vector3d point_imu = Eigen::Vector3d::Zero();
  Eigen::VectorXf reference_patch;
  Eigen::VectorXf current_patch;
  Eigen::Matrix<double, Eigen::Dynamic, 2> gradients;  // dI/du per patch pixel
  double reference_inv_exposure = 1.0;
};

const VisualPatch* SelectReferencePatch(const VisualPoint& visual_point,
                                        const Eigen::Vector3d& camera_position);

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params);

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state,
                             const CameraParams& camera_params,
                             double intensity_noise_std);

}  // namespace livo_slam