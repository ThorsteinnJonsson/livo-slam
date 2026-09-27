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
  // Both patches are divided by their own mean intensity, which removes the
  // brightness scale the exposure state used to absorb.
  Eigen::VectorXf reference_patch;
  Eigen::VectorXf current_patch;
  Eigen::Matrix<double, Eigen::Dynamic, 2> gradients;  // dI/du per patch pixel
  double reference_inv_exposure = 1.0;
};

// A point admitted for this frame, with the reference patch and warp chosen
// once so the iteration optimises a fixed set.
struct SelectedPoint {
  const VisualPoint* point = nullptr;
  const VisualPatch* reference_patch = nullptr;
  Eigen::Matrix2d warp = Eigen::Matrix2d::Identity();
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();  // Full resolution.
  double distance = 0.0;                            // Range from the camera.
};

const VisualPatch* SelectReferencePatch(const VisualPoint& visual_point,
                                        const Eigen::Vector3d& camera_position);

// Picks the patch with the lowest mean squared difference against the point's
// other patches.
const VisualPatch* SelectReferencePatchMedoid(const VisualPoint& visual_point);

// Picks the patch scoring highest on correlation with the point's other patches
// plus how square-on it was taken.
const VisualPatch* SelectReferencePatchScored(const VisualPoint& visual_point);

PhotometricMatch CreatePhotometricMatch(
    const cv::Mat& image, const Eigen::Vector2d& pixel,
    const Eigen::Vector3d& point_imu, const VisualPatch& reference_patch,
    std::size_t pyramid_level, std::size_t patch_size,
    const Eigen::Matrix2d& warp = Eigen::Matrix2d::Identity(),
    bool normalize = true);

std::vector<PhotometricMatch> BuildCameraMatches(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params,
    std::vector<VisualObservation>* observations = nullptr,
    const cv::Mat& depth_image = cv::Mat());

// Runs every gate once and fixes the reference patch and warp per point.
std::vector<SelectedPoint> SelectCameraPoints(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params,
    std::vector<VisualObservation>* observations = nullptr,
    const cv::Mat& depth_image = cv::Mat());

// Resamples the current image for an already selected set.
std::vector<PhotometricMatch> BuildMatchesFromSelection(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<SelectedPoint>& selection,
    const CameraParams& camera_params, const CameraUpdateParams& update_params);

// Depth of the current scan at each pixel, zero where nothing was measured.
// Used to reject patches that straddle a depth edge.
cv::Mat BuildDepthImage(cv::Size size, const std::vector<Point>& points_imu,
                        const CameraParams& camera_params);

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state,
                             const CameraParams& camera_params,
                             double intensity_noise_std, bool normalize = true,
                             bool estimate_exposure = true);

}  // namespace livo_slam